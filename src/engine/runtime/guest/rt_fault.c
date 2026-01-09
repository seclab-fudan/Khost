#include "rt_fault.h"
#include "rt_cpu.h"
#include "rt_data.h"
#include "rt_debug.h"
#include "rt_nvic.h"
#include "string.h"

/// @brief report fault message to host
/// @param info info message
void REPORT(char *info) {
    if (unlikely(should_output(RT_OUTPUT_FAULT))) {
        debug_clear();
        debug_append_str("[guest-fault] ");
        debug_append_str(info);
        debug_append_str("\n");
        debug_print();
    }
}

/// @brief set a bit in the value
/// @param value base value
/// @param bit 1 or 0
/// @param id bit id
/// @return result value
static uint32_t set_bit(uint32_t value, int bit, int id) {
    uint32_t t = value & (~(1 << id));
    return t | ((bit & 0b1) << id);
}

/// @brief check if we can take the exception
/// @param id id of the exception
/// @return 1 on succ
static int can_take_exception(uint32_t id) {
    uint32_t *vtor = (uint32_t *)RUNTIME_sysctl_state.vtor;
    uint32_t isr_addr = vtor[id] & 0xfffffffe;
    if (isr_addr == 0) {
        // we never trigger interrupt with no isr
        return 0;
    }
    if (((uint16_t *)isr_addr)[0] == 0xe7fe) {
        // we never trigger interrupt with infinitve loop
        return 0;
    }
    if (((uint16_t *)isr_addr)[0] == 0xbe03) {
        // we never trigger exit isr
        return 0;
    }
    return 1;
}

/// @brief take fault exception and do the escalation when need
/// @param id exception id
/// @return 0 on succ, 1 on fail
static int take_exception(uint32_t id) {
    int target_exception = id;

    // if we have already crashed, directly exit to the host
    uint32_t excp_id = cpu_get_ipsr();
    if (excp_id == ARMv7M_EXCP_HARD) {
        return 1;
    }

    // check if the escalation is need
    if (id == ARMv7M_EXCP_BUS || id == ARMv7M_EXCP_MEM || id == ARMv7M_EXCP_USAGE) {
        // case 1: exception happend in exception handling
        if (excp_id == ARMv7M_EXCP_BUS || excp_id == ARMv7M_EXCP_MEM
            || excp_id == ARMv7M_EXCP_USAGE
        ) {
            target_exception = ARMv7M_EXCP_HARD;
        }
        // case 2: exception handler is not activated
        if (nvic_irq_info(id)->active == 0) {
            target_exception = ARMv7M_EXCP_HARD;
        }
        // case 3: priority unmatch
        if (nvic_irq_info(id)->prio >= nvic_exec_prio()) {
            target_exception = ARMv7M_EXCP_HARD;
        }
    }

    // do the escalation
    if (target_exception != id) {
#if KVM_OPEN_DEBUG
        REPORT("escalate to HARD FAULT");
#endif
        uint32_t fsr = RUNTIME_sysctl_state.hfsr;
        RUNTIME_sysctl_state.hfsr = set_bit(fsr, 1, HFSR_FORCED);
    }
    if (!can_take_exception(target_exception)) {
        return 1;
    }
    nvic_set_pending(target_exception);
    return 0;
}

/// @brief inject hard | memmange | usage fault back to firmware
/// @param type fault type
/// @param fpa fault physical address
/// @return 0 for succ, 1 on fail
int inject_fault(int type, uint32_t fpa) {
    if (RUNTIME_first_crash_context_valid == 0) {
        RUNTIME_first_crash_context_valid = 1;
        RUNTIME_first_crash_excp_id = RUNTIME_mcu_state.ipsr;
        RUNTIME_first_crash_task_id = RUNTIME_current_task;
        memcpy(&RUNTIME_first_crash_context, &RUNTIME_firmware_context, sizeof(FirmwareContext));
    }

    uint32_t fsr;
    switch (type)
    {
    case HARD_FAULT_VECTTBL:
#if KVM_OPEN_DEBUG
        REPORT("HardFault on vector read error");
#endif
        fsr = RUNTIME_sysctl_state.hfsr;
        RUNTIME_sysctl_state.hfsr = set_bit(fsr, 1, HFSR_VECTTBL);
        
        return take_exception(ARMv7M_EXCP_HARD);
    case HARD_FAULT_FORCED:
#if KVM_OPEN_DEBUG
        REPORT("HardFault on fault escalation");
#endif
        fsr = RUNTIME_sysctl_state.hfsr;
        RUNTIME_sysctl_state.hfsr = set_bit(fsr, 1, HFSR_FORCED);
        
        return take_exception(ARMv7M_EXCP_HARD);
    case HARD_FAULT_DEBUGEVT:
#if KVM_OPEN_DEBUG
        REPORT("HardFault on breakpoint (BKPT) escalation");
#endif
        fsr = RUNTIME_sysctl_state.hfsr;
        RUNTIME_sysctl_state.hfsr = set_bit(fsr, 1, HFSR_DEBUGEVT);
        
        return take_exception(ARMv7M_EXCP_HARD);
    case BUS_FAULT_STKERR:
#if KVM_OPEN_DEBUG
        REPORT("BusFault on exception entry stack memory operations");
#endif
        fsr = RUNTIME_sysctl_state.cfsr;
        RUNTIME_sysctl_state.cfsr = set_bit(fsr, 1, BFSR_STKERR);

        return take_exception(ARMv7M_EXCP_BUS);
    case BUS_FAULT_UNSTKERR:
#if KVM_OPEN_DEBUG
        REPORT("BusFault on exception return stack memory operations");
#endif
        fsr = RUNTIME_sysctl_state.cfsr;
        RUNTIME_sysctl_state.cfsr = set_bit(fsr, 1, BFSR_UNSTKERR);

        return take_exception(ARMv7M_EXCP_BUS);
    case BUS_FAULT_IBUSERR:
#if KVM_OPEN_DEBUG
        REPORT("BusFault on instruction fetch, precise");
#endif
        fsr = RUNTIME_sysctl_state.cfsr;
        RUNTIME_sysctl_state.cfsr = set_bit(fsr, 1, BFSR_IBUSERR);

        return take_exception(ARMv7M_EXCP_BUS);
    case BUS_FAULT_PRECISERR:
#if KVM_OPEN_DEBUG
        REPORT("BusFault on data access, precise");
#endif
        fsr = RUNTIME_sysctl_state.cfsr;
        fsr = set_bit(fsr, 1, BFSR_PRECISERR);
        RUNTIME_sysctl_state.cfsr = set_bit(fsr, 1, BFSR_BFARVALID);
        RUNTIME_sysctl_state.bfar = fpa;

        return take_exception(ARMv7M_EXCP_BUS);
    case BUS_FAULT_IMPRECISERR:
#if KVM_OPEN_DEBUG
        REPORT("BusFault, bus error on data bus, imprecise");
#endif
        fsr = RUNTIME_sysctl_state.cfsr;
        RUNTIME_sysctl_state.cfsr = set_bit(fsr, 1, BFSR_IMPRECISERR);

        return take_exception(ARMv7M_EXCP_BUS);
    case MEM_FAULT_MSTKERR:
#if KVM_OPEN_DEBUG
        REPORT("MemManage fault on exception entry stack memory operations");
#endif
        fsr = RUNTIME_sysctl_state.cfsr;
        RUNTIME_sysctl_state.cfsr = set_bit(fsr, 1, MMFSR_MSTKERR);
        
        return take_exception(ARMv7M_EXCP_MEM);
    case MEM_FAULT_MUNSTKERR:
#if KVM_OPEN_DEBUG
        REPORT("MemManage fault on exception return stack memory operations");
#endif
        fsr = RUNTIME_sysctl_state.cfsr;
        RUNTIME_sysctl_state.cfsr = set_bit(fsr, 1, MMFSR_MUNSTKERR);
        
        return take_exception(ARMv7M_EXCP_MEM);
    case MEM_FAULT_DACCVIOL:
#if KVM_OPEN_DEBUG
        REPORT("MemManage fault on data access");
#endif
        fsr = RUNTIME_sysctl_state.cfsr;
        fsr = set_bit(fsr, 1, MMFSR_DACCVIOL);
        RUNTIME_sysctl_state.cfsr = set_bit(fsr, 1, MMFSR_MMFARVALID);
        RUNTIME_sysctl_state.mmfar = fpa;

        return take_exception(ARMv7M_EXCP_MEM);
    case MEM_FAULT_IACCVIOL:
#if KVM_OPEN_DEBUG
        REPORT("MemManage fault on instruction access");
#endif
        fsr = RUNTIME_sysctl_state.cfsr;
        RUNTIME_sysctl_state.cfsr = set_bit(fsr, 1, MMFSR_IACCVIOL);

        return take_exception(ARMv7M_EXCP_MEM);
    case USAGE_FAULT_NOCP:
#if KVM_OPEN_DEBUG
        REPORT("UsageFault, No coprocessor");
#endif
        fsr = RUNTIME_sysctl_state.cfsr;
        RUNTIME_sysctl_state.cfsr = set_bit(fsr, 1, UFSR_NOCP);

        return take_exception(ARMv7M_EXCP_USAGE);
    case USAGE_FAULT_UNDEFINSTR:
#if KVM_OPEN_DEBUG
        REPORT("UsageFault, Undefined Instruction");
#endif
        fsr = RUNTIME_sysctl_state.cfsr;
        RUNTIME_sysctl_state.cfsr = set_bit(fsr, 1, UFSR_UNDEFINSTR);

        return take_exception(ARMv7M_EXCP_USAGE);
    case USAGE_FAULT_INVSTATE:
#if KVM_OPEN_DEBUG
        REPORT("UsageFault, attempt to execute an instruction when EPSR.T==0");
#endif
        fsr = RUNTIME_sysctl_state.cfsr;
        RUNTIME_sysctl_state.cfsr = set_bit(fsr, 1, UFSR_INVSTATE);
        
        return take_exception(ARMv7M_EXCP_USAGE);
    case USAGE_FAULT_INVPC:
#if KVM_OPEN_DEBUG
        REPORT("UsageFault, exception return integrity check failures");
#endif
        fsr = RUNTIME_sysctl_state.cfsr;
        RUNTIME_sysctl_state.cfsr = set_bit(fsr, 1, UFSR_INVPC);
        
        return take_exception(ARMv7M_EXCP_USAGE);
    case USAGE_FAULT_UNALIGNED:
#if KVM_OPEN_DEBUG
        REPORT("UsageFault, illegal unaligned load or store");
#endif
        fsr = RUNTIME_sysctl_state.cfsr;
        RUNTIME_sysctl_state.cfsr = set_bit(fsr, 1, UFSR_UNALIGNED);
        
        return take_exception(ARMv7M_EXCP_USAGE);
    case USAGE_FAULT_DIVBYZERO:
#if KVM_OPEN_DEBUG
        REPORT("UsageFault, divide by 0");
#endif
        fsr = RUNTIME_sysctl_state.cfsr;
        RUNTIME_sysctl_state.cfsr = set_bit(fsr, 1, UFSR_DIVBYZERO);
        
        return take_exception(ARMv7M_EXCP_USAGE);
    default:
        return 1;
    }
}
