#include "rt_cpu.h"
#include "rt_debug.h"
#include "data_def.h"

static inline uint32_t cpu_get_msp() __attribute__((always_inline));
static inline uint32_t cpu_get_psp() __attribute__((always_inline));
static inline uint32_t cpu_get_apsr() __attribute__((always_inline));
static inline uint32_t cpu_get_epsr() __attribute__((always_inline));
static inline uint32_t cpu_get_xpsr() __attribute__((always_inline));
static inline void cpu_set_msp(uint32_t) __attribute__((always_inline));
static inline void cpu_set_psp(uint32_t) __attribute__((always_inline));
static inline void cpu_set_apsr(uint32_t) __attribute__((always_inline));
static inline void cpu_set_ipsr(uint32_t) __attribute__((always_inline));
static inline void cpu_set_epsr(uint32_t) __attribute__((always_inline));
static inline void cpu_set_primask(uint32_t) __attribute__((always_inline));
static inline void cpu_set_faultmask(uint32_t) __attribute__((always_inline)); 
static inline void cpu_set_basepri(uint32_t) __attribute__((always_inline));

// load float register from address 
#define LDR_FP_REG(REG_ID, ADDR) \
    asm volatile("vldr.32 "#REG_ID", [%[addr]]" : : [addr] "r" (ADDR) : "memory")

// store float register to address
#define STR_FP_REG(REG_ID, ADDR) \
    asm volatile("vstr.32 "#REG_ID", [%[addr]]" : : [addr] "r" (ADDR) : "memory")

// total task in current firmware
static uint32_t total_task = 0;
// block id backup for isr
static uint32_t per_isr_last_block_id[512];
// block id for tasks
struct { uint32_t sp; uint32_t last_block; } per_task_last_block_id[RUNTIME_MAX_TASK_CNT];

/// @brief change the coverage map when exception enter
/// @param sp stack pointer BEFORE exception
/// @param ipsr ipsr BEFORE exception
void change_coverage_exception_enter(uint32_t sp, uint32_t ipsr) {
    // backup current state
    if (ipsr != 0) {
        // backup to per-isr last block list
        if (ipsr >= 512) {
            runtime_abort();
        }
        per_isr_last_block_id[ipsr] = RUNTIME_firmware_state.block_last_id;
    } else {
        // backup to per-task last block list
        if (RUNTIME_current_task >= total_task) {
            runtime_abort();
        }
        per_task_last_block_id[RUNTIME_current_task].sp = sp;
        per_task_last_block_id[RUNTIME_current_task].last_block = RUNTIME_firmware_state.block_last_id;
    }

    // break the edge accross the irq
    RUNTIME_current_task = 0;
    RUNTIME_firmware_state.block_last_id = 0;
}

/// @brief change the coverage map when exception return
/// @param sp stack pointer AFTER exception
/// @param ipsr ipsr AFTER exception
void change_coverage_exception_return(uint32_t sp, uint32_t ipsr) {
    // restore state
    if (ipsr != 0) {
        // restore state from per-isr last block list
        if (ipsr >= 512) {
            runtime_abort();
        }
        RUNTIME_firmware_state.block_last_id = per_isr_last_block_id[ipsr];
        // clear the current task
        RUNTIME_current_task = 0;
    } else {
        // restore state from per-task last block list
        uint32_t task_id = 0;
        while (task_id < total_task) {
            if (per_task_last_block_id[task_id].sp == sp) {
                break;
            }
            task_id++;
        }
        if (task_id != total_task) {
            // case 1: we find the task we take exception from
            // let's restore to that task
            RUNTIME_firmware_state.block_last_id = per_task_last_block_id[task_id].last_block;
            RUNTIME_current_task = task_id;
        } else {
            // case 2: we find a new task
            // let's create a new task
            if (task_id >= RUNTIME_MAX_TASK_CNT) {
                runtime_abort();
            }
            total_task += 1;
            RUNTIME_firmware_state.block_last_id = 0;
            RUNTIME_current_task = task_id;
            RUNTIME_per_task_initial_stack[task_id] = sp;
        }
    }
}

/// @brief reset the cpu states
void cpu_reset() {
    // we use MSP deault and leave PSP 0
    RUNTIME_mcu_state.msp = RUNTIME_initial_sp;
    RUNTIME_mcu_state.psp = 0;

    // set registers and states
    RUNTIME_mcu_state.current_mode = ARMv7M_THREAD;
    RUNTIME_mcu_state.ipsr = 0;
    RUNTIME_mcu_state.primask = 0;
    RUNTIME_mcu_state.faultmask = 0;
    RUNTIME_mcu_state.basepri = 0;
    RUNTIME_mcu_state.control = 0b100;

    // reset coverage backup, reset is the task-0
    RUNTIME_current_task = 0;
    total_task = 1;
    RUNTIME_per_task_initial_stack[RUNTIME_current_task] = RUNTIME_initial_sp;
}

/// @brief determines the SP bank state by the SPSEL bit in CONTROL
/// @return ARMv7M_MSP when using MSP and ARMv7M_PSP when using PSP
static int cpu_sp_select() {
    uint32_t control_spsel = (
        (RUNTIME_mcu_state.control & ARMv7M_CONTROL_SPSEL_MASK) 
        >> ARMv7M_CONTROL_SPSEL_OFFSET
    );
    if (control_spsel == 1) {
        if (unlikely(RUNTIME_mcu_state.current_mode == ARMv7M_HANDLER)) {
            // spsel should always be 0 under handler mode 
#if KVM_OPEN_DEBUG
            if (unlikely(should_output(RT_OUTPUT_CPU))) {
                debug_clear();
                debug_append_str("[guest-cpu] invalid spsel under handler mode\n");
                debug_print();
            }
#endif
            guest_abort();
        }
        // spsel = 1 and mode = thread: use psp
        return ARMv7M_PSP;
    }
    // spsel = 0 and mode = thread: use msp
    // mode = handler: use msp
    return ARMv7M_MSP;
}

/// @brief gets current MSP, updates MSP member in McuState
/// @return value of MSP
static inline uint32_t cpu_get_msp() {
    // if we are using MSP as SP now, update the MSP
    if (cpu_sp_select() == ARMv7M_MSP) {
        RUNTIME_mcu_state.msp = RUNTIME_firmware_context.sp;
    }
    return RUNTIME_mcu_state.msp;
}

/// @brief sets current MSP, maintains current SP if needed
/// @param value new value of MSP
static inline void cpu_set_msp(uint32_t value) {
    RUNTIME_mcu_state.msp = value;
    // if we are using MSP as SP now, update the SP
    if (cpu_sp_select() == ARMv7M_MSP) {
        RUNTIME_firmware_context.sp = RUNTIME_mcu_state.msp;
    }
}

/// @brief gets current PSP, updates PSP member in McuState
/// @return value of PSP
static inline uint32_t cpu_get_psp() {
    // if we are using PSP as SP now, update the PSP
    if (cpu_sp_select() == ARMv7M_PSP) {
        RUNTIME_mcu_state.psp = RUNTIME_firmware_context.sp;
    }
    return RUNTIME_mcu_state.psp;
}

/// @brief sets current PSP, maintains current SP if needed
/// @param value new value og PSP
static inline void cpu_set_psp(uint32_t value) {
    RUNTIME_mcu_state.psp = value;
    // if we are using PSP as SP now, update the SP_usr
    if (cpu_sp_select() == ARMv7M_PSP) {
        RUNTIME_firmware_context.sp = RUNTIME_mcu_state.psp;
    }
}

/// @brief gets current apsr, only return bits used in ARMv7-M profile
/// @return value of the apsr
static inline uint32_t cpu_get_apsr() {
    // ARMv7M APSR is part of ARMv8A pstate
    uint32_t pstate = RUNTIME_firmware_context.xpsr;
    return pstate & (ARMv7M_PSR_N_MASK | ARMv7M_PSR_Z_MASK | 
                     ARMv7M_PSR_C_MASK | ARMv7M_PSR_V_MASK | 
                     ARMv7M_PSR_Q_MASK | ARMv7M_PSR_GE_MASK);
}

/// @brief sets current apsr, bits only used in ARMv8-A profile is unmodified
/// @param value new value of APSR
static inline void cpu_set_apsr(uint32_t value) {
    // ARMv7M APSR is part of ARMv8A pstate
    uint32_t pstate = RUNTIME_firmware_context.xpsr;
    uint32_t apsr_mask = (ARMv7M_PSR_N_MASK | ARMv7M_PSR_Z_MASK | 
                          ARMv7M_PSR_C_MASK | ARMv7M_PSR_V_MASK | 
                          ARMv7M_PSR_Q_MASK | ARMv7M_PSR_GE_MASK);
    pstate = pstate & (~apsr_mask);
    pstate = pstate | (value & apsr_mask);
    RUNTIME_firmware_context.xpsr = pstate;
}

/// @brief sets value of IPSR
/// @param value new value of IPSR
static inline void cpu_set_ipsr(uint32_t value) {
    RUNTIME_mcu_state.ipsr = value & 0b111111111;
}

/// @brief gets value of EPSR
/// @return value of the EPSR
static inline uint32_t cpu_get_epsr() {
    uint32_t pstate = RUNTIME_firmware_context.xpsr;
    pstate &= ARMv7M_PSR_ICIIT_MASK;
    pstate |= (0b1 << ARMv7M_PSR_T_OFFSET);
    return pstate;
}

/// @brief sets value of EPSR
/// @param value new value of EPSR
static inline void cpu_set_epsr(uint32_t value) {
    uint32_t pstate = RUNTIME_firmware_context.xpsr;
    pstate &= ~(ARMv7M_PSR_ICIIT_MASK);
    pstate |= (value & ARMv7M_PSR_ICIIT_MASK);
    RUNTIME_firmware_context.xpsr = pstate;
}

/// @brief gets value of XPSR (combination of APSR, EPSR and IPSR)
/// @return value of XPSR
static inline uint32_t cpu_get_xpsr() {
    return cpu_get_apsr() | cpu_get_epsr() | cpu_get_ipsr();
}

/// @brief sets current control, changes the cpu state accoring to priv and 
///        spsel bits
/// @param value new value of the CONTROL
void cpu_set_control(uint32_t value) {
    // priv defines the execution privilege
    uint32_t control_priv = (value & ARMv7M_CONTROL_PRIV_MASK) 
                            >> ARMv7M_CONTROL_PRIV_OFFSET;
    if (RUNTIME_mcu_state.current_mode == ARMv7M_THREAD) {
        uint32_t apsr = RUNTIME_firmware_context.xpsr & (~0b11111);
        if (control_priv == 0) {
            // thread mode has privileged access,
            // use ARMv8A AArch32 Sys Mode (EL1)
            apsr |= COMPAT_PSR_MODE_SYS;
        } else {
            // thread mode has unprivileged access, 
            // use ARMv8A AArch32 User Mode (EL0)
            apsr |= COMPAT_PSR_MODE_USR;
        }
        RUNTIME_firmware_context.xpsr = apsr;
    }

    // spsel defines the stack to be used
    uint32_t control_spsel = (value & ARMv7M_CONTROL_SPSEL_MASK) 
                             >> ARMv7M_CONTROL_SPSEL_OFFSET;
    if (RUNTIME_mcu_state.current_mode == ARMv7M_THREAD) {
        // save current sp
        uint32_t current_spsel = cpu_sp_select();
        uint32_t current_sp = RUNTIME_firmware_context.sp;
        if (current_spsel == ARMv7M_PSP) {
            RUNTIME_mcu_state.psp = current_sp;
        } else {
            RUNTIME_mcu_state.msp = current_sp;
        }
        // change sp according to spsel
        if (control_spsel == 0) {
            // use sp_main as the current stack
            RUNTIME_firmware_context.sp = RUNTIME_mcu_state.msp;
        } else {
            // use sp_process as the current stack
            RUNTIME_firmware_context.sp = RUNTIME_mcu_state.psp;
        }
    }

    // fpca defines whether the fp is active 
    // note: we always enable fp in AArch32 mode, this is only used by firmware
    // note: we always set fpca when apsen is enabled
    uint32_t control_fpca = (value & ARMv7M_CONTROL_FPCA_MASK) 
                            >> ARMv7M_CONTROL_FPCA_OFFSET;
    control_fpca |= (sysctl_get(SC_REG_FPCCR) >> 31);

    // update the CONTROL register
    RUNTIME_mcu_state.control = 0;
    RUNTIME_mcu_state.control |= (control_priv) << ARMv7M_CONTROL_PRIV_OFFSET;
    RUNTIME_mcu_state.control |= (control_spsel) << ARMv7M_CONTROL_SPSEL_OFFSET;
    RUNTIME_mcu_state.control |= (control_fpca) << ARMv7M_CONTROL_FPCA_OFFSET;
}

/// @brief sets value of PRIMASK and update state of nvic 
/// @param value new value of PRIMASK
static inline void cpu_set_primask(uint32_t value) {
    RUNTIME_mcu_state.primask = value;
    nvic_irq_update();
}

/// @brief sets value of FAULTMASK and update state of nvic
/// @param value new value of FAULTMASK
static inline void cpu_set_faultmask(uint32_t value) {
    RUNTIME_mcu_state.faultmask = value;
    nvic_irq_update();
}

/// @brief sets value of BASEPRI and update state of nvic 
/// @param value new value of BASEPRI
static inline void cpu_set_basepri(uint32_t value) {
    RUNTIME_mcu_state.basepri = value;
    nvic_irq_update();
}

/// @brief emulates exception enter behavior of ARMv7-M
/// @param exc_number exception number
/// @note modifies special registers directly (without using helper functions)
///       and maintains the system state directly
/// @ref https://developer.arm.com/documentation/ddi0403/d/System-Level-Architecture/System-Level-Programmers--Model/ARMv7-M-exception-model/Exception-entry-behavior?lang=en
void cpu_exception_enter(uint32_t exc_number) {
    // [step-0] Update current state ===========================================
    uint32_t last_sp = RUNTIME_firmware_context.sp;
    uint32_t last_ipsr = RUNTIME_mcu_state.ipsr;

    uint32_t cur_psp = cpu_get_psp();   // gets and updates MSP
    uint32_t cur_msp = cpu_get_msp();   // gets and updates MSP
    uint32_t new_psp = cur_psp;
    uint32_t new_msp = cur_msp;
    uint32_t return_addr = RUNTIME_firmware_context.pc;

    // [step-1] push stack =====================================================
    // calculates the frame size and align
    uint32_t frame_size = 0;
    uint32_t force_align = 0;
    uint32_t control_fpca = (
        (RUNTIME_mcu_state.control & ARMv7M_CONTROL_FPCA_MASK) 
        >> ARMv7M_CONTROL_FPCA_OFFSET
    );
    if (control_fpca) {
        frame_size = 0x68;
        force_align = 1;
    } else {
        frame_size = 0x20;
        force_align = (sysctl_get(SC_REG_CCR) & 0b1000000000) >> 9; // CCR.STKALIGN
    }

    // calculates the frame pointer
    uint32_t spmask = ~(force_align << 2);
    uint32_t control_spsel = (
        (RUNTIME_mcu_state.control & ARMv7M_CONTROL_SPSEL_MASK) 
        >> ARMv7M_CONTROL_SPSEL_OFFSET
    );
    uint32_t frame_ptr_align = 0;
    uint32_t frame_ptr;
    if (control_spsel == 1 && RUNTIME_mcu_state.current_mode == ARMv7M_THREAD) {
        frame_ptr_align = ((cur_psp & 0b100) >> 2) & force_align;
        new_psp = (cur_psp - frame_size) & spmask;
        frame_ptr = new_psp;
    } else {
        frame_ptr_align = ((cur_msp & 0b100) >> 2) & force_align;
        new_msp = (cur_msp - frame_size) & spmask;
        frame_ptr = new_msp;
    }

    // checks if the frame pointer is valid
    if (frame_ptr < SRAM_START || frame_ptr > SRAM_END ||
        frame_ptr+frame_size < SRAM_START || frame_ptr+frame_size > SRAM_END) {
#if KVM_OPEN_DEBUG
        if (unlikely(should_output(RT_OUTPUT_CPU))) {
            debug_clear();
            debug_append_str("[guest-cpu] SP incorrect when exception enter\n");
            debug_print();
        }
#endif
        guest_abort();
    }

    // output log
#if KVM_OPEN_DEBUG
    if (unlikely(should_output(RT_OUTPUT_CPU))) {
        debug_clear();
        debug_append_str("[guest-cpu] handling exception enter: exc_number = 0x");
        debug_append_int(exc_number);
        debug_append_str(", from context: pc = 0x");
        debug_append_int(RUNTIME_firmware_context.pc);
        debug_append_str(", lr = 0x");
        debug_append_int(RUNTIME_firmware_context.lr);
        debug_append_str(", sp = 0x");
        debug_append_int(RUNTIME_firmware_context.sp);
        debug_append_str(", frame is saved at 0x");
        debug_append_int(frame_ptr);
        debug_append_str("\n");
        debug_print();
    }
#endif

    // stores the regular context
    // note: only the stack locations, not the store order, are architected  
    ((uint32_t *)frame_ptr)[0] = RUNTIME_firmware_context.r0;
    ((uint32_t *)frame_ptr)[1] = RUNTIME_firmware_context.r1;
    ((uint32_t *)frame_ptr)[2] = RUNTIME_firmware_context.r2;
    ((uint32_t *)frame_ptr)[3] = RUNTIME_firmware_context.r3;
    ((uint32_t *)frame_ptr)[4] = RUNTIME_firmware_context.r12;
    ((uint32_t *)frame_ptr)[5] = RUNTIME_firmware_context.lr;
    ((uint32_t *)frame_ptr)[6] = return_addr;
    uint32_t xpsr = cpu_get_xpsr();
    uint32_t saved_xpsr = 0;
    saved_xpsr |= (xpsr & 0b11111111111111111111110111111111);
    saved_xpsr |= (frame_ptr_align << 9);
    ((uint32_t *)frame_ptr)[7] = saved_xpsr;

    // stores the FP context
    // note: we don't use lazy save stack
    if (control_fpca == 1) {
        STR_FP_REG(s0, frame_ptr + 0x20 + 0x00);
        STR_FP_REG(s1, frame_ptr + 0x20 + 0x04);
        STR_FP_REG(s2, frame_ptr + 0x20 + 0x08);
        STR_FP_REG(s3, frame_ptr + 0x20 + 0x0c);
        STR_FP_REG(s4, frame_ptr + 0x20 + 0x10);
        STR_FP_REG(s5, frame_ptr + 0x20 + 0x14);
        STR_FP_REG(s6, frame_ptr + 0x20 + 0x18);
        STR_FP_REG(s7, frame_ptr + 0x20 + 0x1c);
        STR_FP_REG(s8, frame_ptr + 0x20 + 0x20);
        STR_FP_REG(s9, frame_ptr + 0x20 + 0x24);
        STR_FP_REG(s10, frame_ptr + 0x20 + 0x28);
        STR_FP_REG(s11, frame_ptr + 0x20 + 0x2c);
        STR_FP_REG(s12, frame_ptr + 0x20 + 0x30);
        STR_FP_REG(s13, frame_ptr + 0x20 + 0x34);
        STR_FP_REG(s14, frame_ptr + 0x20 + 0x38);
        STR_FP_REG(s15, frame_ptr + 0x20 + 0x3c);

        uint32_t fpscr = 0;
        asm volatile("vmrs %[fpscr], fpscr" : [fpscr] "=r" (fpscr) : : );
        *(uint32_t *)(frame_ptr+0x60) = fpscr;
    }

    // calculates the lr
    uint32_t lr = 0b11111111111111111111111111100000;
    if (RUNTIME_mcu_state.current_mode == ARMv7M_HANDLER) {
        lr |= (!control_fpca) << 4;
        lr |= 0b0001;
    } else {
        lr |= (!control_fpca) << 4;
        lr |= 0b1001;
        lr |= (control_spsel) << 2;
    }

    // [step-2] ExceptionTaken =================================================
    uint32_t vector_table_addr = sysctl_get(SC_REG_VTOR);
    uint32_t tmp = *((uint32_t *)(vector_table_addr + exc_number * 0x4));
    uint32_t pc = tmp & 0xFFFFFFFE;
    uint32_t tbit = tmp & 0b1;
    // current mode set to ARMv7 Handler mode
    RUNTIME_mcu_state.current_mode = ARMv7M_HANDLER;
    // exception number set to IPSR
    RUNTIME_mcu_state.ipsr = exc_number;
    // PRIMASK, FAULTMASK, BASEPRI unchanged on exception entry
    // CONTROL.nPRIV unchanged
    RUNTIME_mcu_state.control &= (~0b110);
    // current Stack is Main, Floating-point extension active
    RUNTIME_mcu_state.control |= (0b100);

    // [step-3] ModeChange =====================================================
    // Change ARMv8-A CPU mode to System mode
    // IT/ICI bits cleared, T-bit set from last bit of vector table entry
    RUNTIME_firmware_context.xpsr = COMPAT_PSR_MODE_SYS;
    RUNTIME_firmware_context.xpsr |= tbit << 5;
    
    RUNTIME_firmware_context.lr = lr;
    RUNTIME_firmware_context.pc = pc;
    RUNTIME_firmware_context.sp = new_msp;

    RUNTIME_mcu_state.msp = new_msp;
    RUNTIME_mcu_state.psp = new_psp;
    
    // STEP4: ACK IRQ ==========================================================
    // nvic side acknowledge
    nvic_acknowledge_irq();
    change_coverage_exception_enter(last_sp, last_ipsr);
    
    // output log

#if KVM_OPEN_DEBUG
    if (unlikely(should_output(RT_OUTPUT_CPU))) {
        debug_clear();
        debug_append_str("[guest-cpu] handling exception enter: to context: pc = 0x");
        debug_append_int(RUNTIME_firmware_context.pc);
        debug_append_str(", sp = 0x");
        debug_append_int(RUNTIME_firmware_context.sp);
        debug_append_str("\n");
        debug_print();
    }
#endif
}

/// @brief emulates exception exit behavior of ARMv7-M
/// @param exc_return EXC_RETURN in LR
/// @note modifies special registers directly (without using helper functions)
///       and maintains the system state directly
/// @ref https://developer.arm.com/documentation/ddi0403/d/System-Level-Architecture/System-Level-Programmers--Model/ARMv7-M-exception-model/Exception-return-behavior?lang=en
void cpu_exception_return(uint32_t exc_return) {
    // check the current mode
    if (RUNTIME_mcu_state.current_mode != ARMv7M_HANDLER) {
#if KVM_OPEN_DEBUG
        if (unlikely(should_output(RT_OUTPUT_CPU))) {
            debug_clear();
            debug_append_str("[guest-cpu] exception return from thread mode\n");
            debug_print();
        }
#endif
        guest_abort();
    }

    // [step-0] update current state ===========================================
    uint32_t cur_msp = cpu_get_msp();
    uint32_t cur_psp = cpu_get_psp();
    uint32_t new_msp = cur_msp;
    uint32_t new_psp = cur_psp;
    uint32_t returning_exception_number = cpu_get_ipsr();
    uint32_t frame_ptr = 0;
    switch (exc_return & 0b1111)
    {
    // return to Handler
    case 0b0001:
        frame_ptr = cur_msp;
        RUNTIME_mcu_state.current_mode = ARMv7M_HANDLER;
        RUNTIME_mcu_state.control &= (~ARMv7M_CONTROL_SPSEL_MASK);   // CONTROL.SPSEL = 0
        break;
    // returning to Thread using Main stack
    case 0b1001:
        frame_ptr = cur_msp;
        RUNTIME_mcu_state.current_mode = ARMv7M_THREAD;
        RUNTIME_mcu_state.control &= (~ARMv7M_CONTROL_SPSEL_MASK);   // CONTROL.SPSEL = 0
        break;
    // returning to Thread using Process stack
    case 0b1101:
        frame_ptr = cur_psp;
        RUNTIME_mcu_state.current_mode = ARMv7M_THREAD;
        RUNTIME_mcu_state.control &= (~ARMv7M_CONTROL_SPSEL_MASK);
        RUNTIME_mcu_state.control |= 0b10;                           // CONTROL.SPSEL = 1
        break;
    default:
#if KVM_OPEN_DEBUG
        if (unlikely(should_output(RT_OUTPUT_CPU))) {
            debug_clear();
            debug_append_str("[guest-cpu] illegal EXC_RETURN\n");
            debug_print();
        }
#endif
        guest_abort();
    }
    if (returning_exception_number != 0b000000010) {
        RUNTIME_mcu_state.faultmask = 0;
    }

    // [step-1] pop stack ======================================================
    uint32_t frame_size = 0;
    uint32_t force_align = 0;
    if (((exc_return & 0b10000) >> 4) == 0) {
        frame_size = 0x68;
        force_align = 1;
    } else {
        frame_size = 0x20;
        force_align = (sysctl_get(SC_REG_CCR) & 0b1000000000) >> 9; // CCR.STKALIGN
    }

    // checks frame ptr
    if (frame_ptr < SRAM_START || frame_ptr > SRAM_END ||
        frame_ptr+frame_size < SRAM_START || frame_ptr+frame_size > SRAM_END) {
#if KVM_OPEN_DEBUG
        if (unlikely(should_output(RT_OUTPUT_CPU))) {
            debug_clear();
            debug_append_str("[guest-cpu] SP incorrect when exception return\n");
            debug_print();
        }
#endif
        guest_abort();
    }

    RUNTIME_firmware_context.r0 = ((uint32_t *)frame_ptr)[0];
    RUNTIME_firmware_context.r1 = ((uint32_t *)frame_ptr)[1];
    RUNTIME_firmware_context.r2 = ((uint32_t *)frame_ptr)[2];
    RUNTIME_firmware_context.r3 = ((uint32_t *)frame_ptr)[3];
    RUNTIME_firmware_context.r12 = ((uint32_t *)frame_ptr)[4];
    RUNTIME_firmware_context.lr = ((uint32_t *)frame_ptr)[5];
    RUNTIME_firmware_context.pc = ((uint32_t *)frame_ptr)[6];
    uint32_t psr = ((uint32_t *)frame_ptr)[7];

    // pops float point state
    // note: we don't use lazy save stack
    if (((exc_return & 0b10000) >> 4) == 0) {
        LDR_FP_REG(s0, frame_ptr + 0x20 + 0x00);
        LDR_FP_REG(s1, frame_ptr + 0x20 + 0x04);
        LDR_FP_REG(s2, frame_ptr + 0x20 + 0x08);
        LDR_FP_REG(s3, frame_ptr + 0x20 + 0x0c);
        LDR_FP_REG(s4, frame_ptr + 0x20 + 0x10);
        LDR_FP_REG(s5, frame_ptr + 0x20 + 0x14);
        LDR_FP_REG(s6, frame_ptr + 0x20 + 0x18);
        LDR_FP_REG(s7, frame_ptr + 0x20 + 0x1c);
        LDR_FP_REG(s8, frame_ptr + 0x20 + 0x20);
        LDR_FP_REG(s9, frame_ptr + 0x20 + 0x24);
        LDR_FP_REG(s10, frame_ptr + 0x20 + 0x28);
        LDR_FP_REG(s11, frame_ptr + 0x20 + 0x2c);
        LDR_FP_REG(s12, frame_ptr + 0x20 + 0x30);
        LDR_FP_REG(s13, frame_ptr + 0x20 + 0x34);
        LDR_FP_REG(s14, frame_ptr + 0x20 + 0x38);
        LDR_FP_REG(s15, frame_ptr + 0x20 + 0x3c);

        uint32_t fpscr = *(uint32_t *)(frame_ptr+0x60);
        asm volatile("vmsr fpscr, %[fpscr]" : : [fpscr] "r" (fpscr) : );
    }
    RUNTIME_mcu_state.control &= (~0b100);
    RUNTIME_mcu_state.control |= (!((exc_return & 0b10000) >> 4)) << 2;

    uint32_t sp_mask = (((psr & 0b1000000000) >> 9) & force_align) << 2;
    switch (exc_return & 0b1111)
    {
    case 0b0001:
        new_msp = (cur_msp + frame_size) | sp_mask;
        break;
    case 0b1001:
        new_msp = (cur_msp + frame_size) | sp_mask;
        break;
    case 0b1101:
        new_psp = (cur_psp + frame_size) | sp_mask;
        break;
    }

    // [step-2] update state ===================================================
    uint32_t pstate = 0;
    pstate |= psr & (0b1111111 << 25);
    pstate |= psr & (0b1111111111 << 10);
    pstate |= COMPAT_PSR_T_BIT;

    uint32_t control_priv = (
        (RUNTIME_mcu_state.control & ARMv7M_CONTROL_PRIV_MASK) 
        >> ARMv7M_CONTROL_PRIV_OFFSET
    );
    if (RUNTIME_mcu_state.current_mode == ARMv7M_THREAD) {
        // return to thread mode
        if (control_priv == 0) {
            // thread mode has privileged access: 
            // use ARMv8A AArch32 Sys Mode (EL1) thumb
            pstate |= COMPAT_PSR_MODE_SYS;
        } else {
            // thread mode has unprivileged access:
            // use ARMv8A AArch32 User Mode (EL0) thumb
            pstate |= COMPAT_PSR_MODE_USR;
        }
        RUNTIME_firmware_context.xpsr = pstate;
    } else {
        // returning to handler mode, always use Sys Mode (EL1) thumb
        RUNTIME_firmware_context.xpsr = pstate | COMPAT_PSR_MODE_SYS;
    }

    RUNTIME_mcu_state.ipsr = psr & 0b111111111;
    RUNTIME_mcu_state.psp = new_psp;
    RUNTIME_mcu_state.msp = new_msp;
    if (cpu_sp_select() == ARMv7M_PSP) {
        RUNTIME_firmware_context.sp = RUNTIME_mcu_state.psp;
    } else {
        RUNTIME_firmware_context.sp = RUNTIME_mcu_state.msp;
    }

#if KVM_OPEN_DEBUG
    if (unlikely(should_output(RT_OUTPUT_CPU))) {
        debug_clear();
        debug_append_str("[guest-cpu] handling exception return: exc_retrn = 0x");
        debug_append_int(exc_return);
        debug_append_str(", ret = 0x");
        debug_append_int(RUNTIME_firmware_context.pc);
        debug_append_str(", sp = 0x");
        debug_append_int(RUNTIME_firmware_context.sp);
        debug_append_str("\n");
        debug_print();
    }
#endif

    // [step-3] deactive exception =============================================
    // nvic complete irq
    nvic_complete_irq(returning_exception_number);
    change_coverage_exception_return(
        RUNTIME_firmware_context.sp, RUNTIME_mcu_state.ipsr
    );
}

/// @brief checks if we should take exception now
void cpu_check_irq() {
    // checks pending exception number
    int pending_exc_number = nvic_get_pending_irq();
    if (pending_exc_number == 0) {
        // we directly return to the guest if there's no exception
        return;
    }
    
    // check can_take_pending_exception again because primask, faultmask and 
    // basepri may be changed between IRQ injection and VM_EXIT 
    if (!nvic_can_take_pending_exception()) {
        // we can't take exception now, go back to normal state and leave the 
        // exception pending for future activation
        return;
    }
    
    cpu_exception_enter(pending_exc_number);
}

/// @brief emulates mrs instruction of ARMv7-M
/// @param insn incompatible instruction information
static void cpu_handle_mrs(IncompatibleInsn *insn) {
    uint32_t r_id = insn->rd;
    uint32_t sys_id = insn->sysm;
    // MRS to R13 and R15 is UNPREDICTABLE
    if (r_id == 13 || r_id == 15) {
#if KVM_OPEN_DEBUG
        if (unlikely(should_output(RT_OUTPUT_CPU))) {
            debug_clear();
            debug_append_str("[guest-cpu] unpredictable mrs instruction");
            debug_print();
        }
#endif
        guest_abort();
    }

    // emulates the operation of the mrs
    // ref: https://developer.arm.com/documentation/ddi0403/d/System-Level-Architecture/System-Instruction-Details/ARMv7-M-system-instruction-descriptions/MRS?lang=en
    uint32_t rd = 0;
    switch (sys_id)
    {
    case ARMv7M_APSR:
    case ARMv7M_IAPSR:
    case ARMv7M_EAPSR:
    case ARMv7M_XPSR:
    case ARMv7M_IPSR:
    case ARMv7M_EPSR:
    case ARMv7M_IEPSR:
        if (sys_id & 0b1) {
            rd |= (cpu_get_ipsr() & 0b111111111);
        } else if ((sys_id & 0b100) == 0) {
            rd |= (cpu_get_apsr() & (0b11111 << 27));
            rd |= (cpu_get_apsr() & (0b1111 << 16));
        }
        break;
    case ARMv7M_MSP:
        rd = cpu_get_msp();
        break;
    case ARMv7M_PSP:
        rd = cpu_get_psp();
        break;
    case ARMv7M_PRIMASK:
        rd = (cpu_get_primask() & 0b1);
        break;
    case ARMv7M_BASERPI:
    case ARMv7M_BASERPI_MAX:
        rd = (cpu_get_basepri() & 0b11111111);
        break;
    case ARMv7M_FAULTMASK:
        rd = (cpu_get_faultmask() & 0b1);
        break;
    case ARMv7M_CONTROL:
        rd = (cpu_get_control() & 0b111);
        break;
    default:
#if KVM_OPEN_DEBUG
        if (unlikely(should_output(RT_OUTPUT_CPU))) {
            debug_clear();
            debug_append_str("[guest-cpu] unknown system register");
            debug_print();
        }
#endif
        guest_abort();
    }
    ((uint32_t *)&RUNTIME_firmware_context)[r_id] = rd;
}

/// @brief emulates msr instruction of ARMv7-M
/// @param insn incompatible instruction information
static void cpu_handle_msr(IncompatibleInsn *insn) {
    uint32_t r_id = insn->rn;
    uint32_t sys_id = insn->sysm;
    // MRS to R13 and R15 is UNPREDICTABLE
    if (r_id == 13 || r_id == 15) {
#if KVM_OPEN_DEBUG
        if (unlikely(should_output(RT_OUTPUT_CPU))) {
            debug_clear();
            debug_append_str("[guest-cpu] unpredictable mrs instruction");
            debug_print();
        }
#endif
        guest_abort();
    }

    // emulate the operation of the msr
    // ref: https://developer.arm.com/documentation/ddi0403/d/System-Level-Architecture/System-Instruction-Details/ARMv7-M-system-instruction-descriptions/MSR?lang=en
    uint32_t rn = ((uint32_t *)&RUNTIME_firmware_context)[r_id];
    switch (sys_id)
    {
    case ARMv7M_APSR:
    case ARMv7M_IAPSR:
    case ARMv7M_EAPSR:
    case ARMv7M_XPSR:
    case ARMv7M_IPSR:
    case ARMv7M_EPSR:
    case ARMv7M_IEPSR:
        if ((sys_id & 0b100) == 0) {
            uint32_t apsr = 0;
            if ((sys_id & 0b1) != 0) {
                apsr |= (rn & (0b1111 << 16));
            }
            if ((sys_id & 0b10) != 0) {
                apsr |= (rn & (0b11111 << 27));
            }
            cpu_set_apsr(apsr);
        }
        break;
    case ARMv7M_MSP:
        cpu_set_msp(rn);
        break;
    case ARMv7M_PSP:
        cpu_set_psp(rn);
        break;
    case ARMv7M_PRIMASK:
        cpu_set_primask(rn & 0b1);
        break;
    case ARMv7M_BASERPI:
        cpu_set_basepri(rn & 0b11111111);
        break;
    case ARMv7M_BASERPI_MAX:
        if (((rn & 0b11111111) != 0) &&
            ((rn & 0b11111111) < cpu_get_basepri() || cpu_get_basepri() == 0)) {
            cpu_set_basepri(rn & 0b11111111);
        }
        break;
    case ARMv7M_FAULTMASK:
        cpu_set_faultmask(rn & 0b1);
        break;
    case ARMv7M_CONTROL:
        cpu_set_control(rn & 0b111);
        break;
    default:
#if KVM_OPEN_DEBUG
        if (unlikely(should_output(RT_OUTPUT_CPU))) {
            debug_clear();
            debug_append_str("[guest-cpu] unknown system register");
            debug_print();
        }
#endif
        guest_abort();
    }
}

/// @brief emulates cps instruction of ARMv7-M
/// @param insn incompatible instruction information
static void cpu_handle_cps(IncompatibleInsn *insn) {
    int enable = (insn->im == 0);
    int affect_pri = (insn->i == 1);
    int affect_fault = (insn->f == 1);

    // change primask
    if (affect_pri) {
        cpu_set_primask(enable ? 0 : 1);
    }
    // change faultmask
    if (affect_fault) {
        cpu_set_faultmask(enable ? 0 : 1);
    }
}

/// @brief find incompatible insn information
/// @param pc address of the instruction
/// @return pointer to the instruction
static IncompatibleInsn* cpu_find_incompatible_insn(uint32_t pc) {
    for (int i = 0; i < RUNTIME_incompatible_insn_cnt; i++) {
        if (RUNTIME_incompatible_insn_list[i].addr == pc) {
            return &RUNTIME_incompatible_insn_list[i];
        }
    }
    return 0;
}

/// @brief emulates mrs/msr/cps instruction of ARMv7-M
/// @param pc instruction address
void cpu_handle_incompatible_insn(uint32_t pc) {
    IncompatibleInsn *insn = cpu_find_incompatible_insn(pc);
    switch (insn->type)
    {
    // mrs instruction
    case ARMv7M_MRS:
        cpu_handle_mrs(insn);
        return;
    // msr instruction
    case ARMv7M_MSR:
        cpu_handle_msr(insn);
        return;
    // cps instruction
    case ARMv7M_CPS: 
        cpu_handle_cps(insn);
        return;
    default:
#if KVM_OPEN_DEBUG
        if (unlikely(should_output(RT_OUTPUT_CPU))) {
            debug_clear();
            debug_append_str("[guest-cpu] unknown type of instruction");
            debug_print();
        }
#endif
        guest_abort();
    }
}

__attribute__((section(".rt_dmz_handler")))
/// @brief guest abort
void guest_abort() {
    asm("bkpt #0x8");
}

/// @brief runtime internal abort
void runtime_abort() {
#if KVM_OPEN_DEBUG
    if (unlikely(should_output(RT_OUTPUT_CPU))) {
        uint32_t *regs = (uint32_t *)&RUNTIME_firmware_context;
        for (int i = 0; i < 16; i++) {
            debug_clear();
            debug_append_str("R-");
            debug_append_int(i);
            debug_append_str(": ");
            debug_append_int(regs[i]);
            debug_append_str("\n");
            debug_print();
        }
    }
#endif
    asm("bkpt #0x9");
}

__attribute__((section(".rt_dmz_handler")))
/// @brief guest exit
void guest_exit() {
    asm("bkpt #0xa");
}

__attribute__((section(".rt_dmz_handler")))
/// @brief guest timeout
void guest_timeout() {
    asm("bkpt #0xb");
}
