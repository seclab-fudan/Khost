#include "rt_global.h"
#include "rt_handler.h"
#include "rt_emu.h"
#include "rt_debug.h"
#include "rt_data.h"
#include "rt_cpu.h"
#include "rt_systick.h"
#include "rt_fault.h"
#include "string.h"

/// @brief check if the address is valid
/// @param addr address to be checked
/// @return 0 for invalid and 1 for valid
int check_address(uint32_t addr) {
    for (int i = 0; i < RUNTIME_ADDRESS_RANGE_SIZE; i++) {
        if (RUNTIME_address_range[i].start == 0xffffffff
            && RUNTIME_address_range[i].end == 0xffffffff
        ) {
            break;
        }
        if (RUNTIME_address_range[i].start <= addr
            && RUNTIME_address_range[i].end > addr
        ) {
            return 1;
        }
    }
    return 0;
}

/// @brief check if the firmware is in thumb state
/// @return 0 for pass, 1 for error, 2 for handled
int check_thumb_state() {
    int tbit = (RUNTIME_firmware_context.xpsr >> 5) & 0b1;
    if (tbit == 0) {
#if KVM_OPEN_DEBUG
        if (unlikely(should_output(RT_OUTPUT_HANDLER))) {
            debug_clear();
            debug_append_str("[guest-handler] firmware in ARM state, pc=0x");
            debug_append_int(RUNTIME_firmware_context.pc);
            debug_append_str("\n");
            debug_print();
        }
#endif
        // inject the invalid state usage fault to the firmware
        if (!inject_fault(USAGE_FAULT_INVSTATE, RUNTIME_firmware_context.pc)) {
            return 2;
        }
        // if we cannot inject the fault, abort the guest
#if KVM_OPEN_DEBUG
        if (unlikely(should_output(RT_OUTPUT_CPU))) {
            debug_clear();
            debug_append_str("[guest-handler] firmware in ARM state, pc=0x");
            debug_append_int(RUNTIME_firmware_context.pc);
            debug_append_str("\n");
            debug_print();
        }
#endif
        return 1;
    }
    return 0;
}

/// @brief check if the pc is in code range
void check_code_range() {
    uint32_t pc = RUNTIME_firmware_context.pc;
    if (
        (pc < RUNTIME_code_range.bin_start || pc >= RUNTIME_code_range.bin_end)
        && (pc < RUNTIME_code_range.rewrite_start || pc >= RUNTIME_code_range.rewrite_end)
    ) {
#if KVM_OPEN_DEBUG
        if (unlikely(should_output(RT_OUTPUT_HANDLER))) {
            debug_clear();
            debug_append_str("[guest-handler] firmware out of code range\n");
            debug_print();
        }
#endif
        // inject the invalid state usage fault to the firmware
        if (!inject_fault(USAGE_FAULT_INVPC, RUNTIME_firmware_context.pc)) {
            return;
        }
        // if we cannot inject the fault, abort the guest
#if KVM_OPEN_DEBUG
        if (unlikely(should_output(RT_OUTPUT_CPU))) {
            debug_clear();
            debug_append_str("[guest-handler] firmware out of code range\n");
            debug_print();
        }
#endif
        handler_dabt_fail();
    }
}

/// @brief Undefined handler of the guest firmware
void und_handler() {
    int check_thumb = check_thumb_state();
    if (check_thumb == 2) {
        return;
    }
    if (check_thumb == 1) {
        handler_und_fail();
    }

    // inject the undefined instruction fault
    if (!inject_fault(USAGE_FAULT_UNDEFINSTR, RUNTIME_firmware_context.pc)) {
        return;
    }
    handler_und_fail();
}

/// @brief DataAbort handler of the guest firmware
/// @param dfsr value of Data Fault Status Register
/// @param dfar value of Data Fault Address Register
void dabt_handler(uint32_t dfsr, uint32_t dfar) {
    int check_thumb = check_thumb_state();
    if (check_thumb == 2) {
        return;
    }
    if (check_thumb == 1) {
        handler_dabt_fail();
    }

    // we only solve first level domain fault for mmio
    uint32_t pc = RUNTIME_firmware_context.pc;
    uint32_t fs = (((dfsr >> 10) & 0b1) << 4) | (dfsr & 0b1111);
    if (fs == 0b01001) {
        if (check_address(dfar)) {
#if KVM_OPEN_DEBUG
            if (unlikely(should_output(RT_OUTPUT_HANDLER))) {
                debug_clear();
                debug_append_str("[guest-handler] MMIODataAbort: DFSR = 0x");
                debug_append_int(dfsr);
                debug_append_str(", DFAR = 0x");
                debug_append_int(dfar);
                debug_append_str(", PC = 0x");
                debug_append_int(pc);    
                debug_append_str(", SP = 0x");
                debug_append_int(RUNTIME_firmware_context.sp);
                debug_append_str(", XPSR = 0x");
                debug_append_int(RUNTIME_firmware_context.xpsr);
                debug_append_str("\n");
                debug_print();
            }
#endif
            emulate_inst(pc, dfar);
            return;
        }
    }

    // other data abort must belong to the firmware
#if KVM_OPEN_DEBUG
    if (unlikely(should_output(RT_OUTPUT_HANDLER))) {
        debug_clear();
        debug_append_str("[guest-handler] DataAbort: fs = ");
        debug_append_int(fs);
        debug_append_str(", dfar = ");
        debug_append_int(dfar);
        debug_append_str(", pc = ");
        debug_append_int(pc);
        debug_append_str("\n");
        debug_print();
    }
#endif
    if (fs == 0b00001) {
        // alignment fault is translated to unaligned abort
        if (!inject_fault(USAGE_FAULT_UNALIGNED, 0)) {
            return;
        }
    } else if (fs == 0b00100 || fs == 0b10110 || fs == 0b11000) {
        // asynchronous data abort and instruction cache maintenance abort are
        // translated to impreciserr abort
        if (!inject_fault(BUS_FAULT_IMPRECISERR, 0)) {
            return;
        }
    } else {
        // synchronous data abort are translated to preciserr abort 
        if (!inject_fault(BUS_FAULT_PRECISERR, dfar)) {
            return;
        }
    }

    // we cannot handle the data abort, exit to host
    handler_dabt_fail();
}

/// @brief Wfi instruction handler
void wfi_handler() {
    // trigger potential usefull systick
    if (RUNTIME_emu_systick_state.interrput
        && nvic_is_irq_enabled(ARMv7M_EXCP_SYSTICK)) {
        nvic_set_pending(ARMv7M_EXCP_SYSTICK);
    }
    // trigger all possible interrupts
    for (int i = ARMv7M_EXCP_SYSTICK+1; i < NVIC_MAX_VECTORS; i++) {
        if (!nvic_is_irq_enabled(i)) {
            continue;
        }
        uint32_t *vtor = (uint32_t *)RUNTIME_sysctl_state.vtor;
        uint32_t isr_addr = vtor[i] & 0xfffffffe;
        if (isr_addr != 0 && ((uint16_t *)isr_addr)[0] != 0xe7fe) {
            nvic_set_pending(i);
        }
    }
}

/// @brief Svc instruction handler
void svc_handler() {
    int check_thumb = check_thumb_state();
    if (check_thumb == 2) {
        return;
    }
    if (check_thumb == 1) {
        guest_abort();
    }

    // get svc call number
    uint32_t svc_num = 0;
    uint32_t cpsr = RUNTIME_firmware_context.xpsr;
	uint32_t is_arm = !(cpsr & PSR_AA32_T_BIT);
    uint32_t pc = RUNTIME_firmware_context.pc;
    if (is_arm) {
        svc_num = ((uint8_t *)(pc - 0x4))[0];
    } else {
        svc_num = ((uint8_t *)(pc - 0x2))[0];
    }

#if KVM_OPEN_DEBUG
    if (unlikely(should_output(RT_OUTPUT_HANDLER))) {
        debug_clear();
        debug_append_str("[guest-handler] SvcHandler: PC = 0x");
        debug_append_int(pc);    
        debug_append_str(", ARM = 0x");
        debug_append_int(is_arm);
        debug_append_str(", SVC = 0x");
        debug_append_int(svc_num);
        debug_append_str("\n");
        debug_print();
    }
#endif

    switch(svc_num) {
    case RUNTIME_SVC_IRQ:
        // we'll trigger context switch after return, so we do nothing here
        return;
    case RUNTIME_WFI:
        wfi_handler();
        return;
    case RUNTIME_INCOMPATIBLE_INSN:
        cpu_handle_incompatible_insn(is_arm ? pc - 0x4 : pc - 0x2);
        return;
    }

    // this svc must belong to the firmware
    nvic_set_pending(ARMv7M_EXCP_SVC);
    return;
}

/// @brief Prefetch abort handler
void prefetch_handler() {
    int check_thumb = check_thumb_state();
    if (check_thumb == 2) {
        return;
    }
    if (check_thumb == 1) {
        handler_prefetch_fail();
    }

    uint32_t pc = RUNTIME_firmware_context.pc;
    uint32_t exc_return_mask = 0xffffffe0;
    uint32_t exc_return = pc | 0x1;
    
    // prefetch abort because of exception return
    if ((exc_return & exc_return_mask) == exc_return_mask) {
        cpu_exception_return(exc_return);
        return;
    }

#if KVM_OPEN_DEBUG
    if (unlikely(should_output(RT_OUTPUT_HANDLER))) {
        debug_clear();
        debug_append_str("[guest-handler] PrefetchHandler: PC = 0x");
        debug_append_int(pc);
        debug_append_str("\n");
        debug_print();
    }
#endif

    // prefetch abort belongs to firmware
    if (!check_address(pc)) {
        // PC in invalid area means bus fault
        if (!inject_fault(BUS_FAULT_IBUSERR, RUNTIME_firmware_context.pc)) {
            return;
        }
    } else {
        // PC in valid area means memmanage fault
        if (!inject_fault(MEM_FAULT_IACCVIOL, RUNTIME_firmware_context.pc)) {
            return;
        } 
    }

    // we cannot handle the prefetch abort, exit the firmware
    handler_prefetch_fail();
}

extern char _end_rt_stack;

/// @brief Part of the reset handler
void reset_firmware() {
    // executed block counter
    RUNTIME_firmware_state.block_count = 0;
    // max block to be executed
    RUNTIME_firmware_state.block_limit = MAX_BLOCK;
    // last executed block
    RUNTIME_firmware_state.block_last_id = 0;
    // time to trigger next irq
    RUNTIME_firmware_state.block_next_irq = MAX_BLOCK;
    // if the mmio input has ran out
    RUNTIME_firmware_state.mmio_ranout = 0;
    // coverage map of the firmware
    memset(RUNTIME_firmware_state.coverage_map, 0, COVERAGE_MAP_SIZE);
    // set crash context invalid
    RUNTIME_first_crash_context_valid = 0;
}

// unused, only provide for gcc compiler
int main() { return 0; }
