#include "rt_sysctl.h"

/// @brief reset sysctl state
void sysctl_reset() {
    RUNTIME_sysctl_state.cpacr = 0;
    RUNTIME_sysctl_state.vtor = RUNTIME_load_base;
    RUNTIME_sysctl_state.fpccr = 0;
    RUNTIME_sysctl_state.aircr = 0;
    RUNTIME_sysctl_state.ccr = (1 << SC_CCR_STKALIGN_SHIFT);
    RUNTIME_sysctl_state.scr = 0;
    RUNTIME_sysctl_state.cfsr = 0;
    RUNTIME_sysctl_state.hfsr = 0;
}

/// @brief get registers in System Control Block
/// @param reg register id defined in global.h
/// @return value of the register
uint32_t sysctl_get(uint32_t reg) {
    switch (reg)
    {
    case SC_REG_ICTR:
        return 0b1110;
    case SC_REG_CPUID:
        return CPUID_VALUE;
    case SC_REG_VTOR:
        return RUNTIME_sysctl_state.vtor;
    case SC_REG_CPACR:
        return RUNTIME_sysctl_state.cpacr;
    case SC_REG_AIRCR:
        return (RUNTIME_sysctl_state.aircr & SC_AIRCR_PRIGROUP_MASK) | 0xfa050000;
    case SC_REG_SHPR1:
        return (
            (nvic_irq_info(7)->prio << 24) |
            (nvic_irq_info(6)->prio << 16) |
            (nvic_irq_info(5)->prio <<  8) |
            (nvic_irq_info(4)->prio <<  0)
        );
    case SC_REG_SHPR2:
        return (
            (nvic_irq_info(11)->prio << 24) |
            (nvic_irq_info(10)->prio << 16) |
            (nvic_irq_info( 9)->prio <<  8) |
            (nvic_irq_info( 8)->prio <<  0)
        );
    case SC_REG_SHPR3:
        return (
            (nvic_irq_info(15)->prio << 24) |
            (nvic_irq_info(14)->prio << 16) |
            (nvic_irq_info(13)->prio <<  8) |
            (nvic_irq_info(12)->prio <<  0)
        );
    case SC_REG_FPCCR:
        return RUNTIME_sysctl_state.fpccr;
    case SC_REG_ICSR: {
        // vectactive 
        uint32_t value = cpu_get_ipsr();    
        // rettobase
        if (nvic_ret_to_base()) {
            value |= (1 << SC_ICSR_RETTOBASE_SHIFT);
        }
        // vectpending
        if (nvic_get_pending_irq()) {
            value |= (nvic_get_pending_irq() & 0x1ff) 
                     << SC_ICSR_VECTPENDING_SHIFT;
        }
        // isrpending
        if (nvic_isr_pending()) {
            value |= (1 << SC_ICSR_ISRPENDING_SHIFT);
        }
        // isrpreempt: RES0 when halting debug not implemented
        // pendstset
        if (nvic_irq_info(ARMv7M_EXCP_SYSTICK)->pending) {
            value |= (1 << SC_ICSR_PENDSTSET_SHIFT);
        }
        // pendsvset
        if (nvic_irq_info(ARMv7M_EXCP_PENDSV)->pending) {
            value |= (1 << SC_ICSR_PENDSVSET_SHIFT);
        }
        // nmipendset
        if (nvic_irq_info(ARMv7M_EXCP_NMI)->pending) {
            value |= (1 << SC_ICSR_NMIPENDSET_SHIFT);
        }
        return value;
    }
    case SC_REG_CCR:
        return RUNTIME_sysctl_state.ccr;
    case SC_REG_SHCSR: {
        uint32_t value = 0;
        if (nvic_irq_info(ARMv7M_EXCP_MEM)->active) {
            value |= (1 << 0);
        }
        if (nvic_irq_info(ARMv7M_EXCP_USAGE)->active) {
            value |= (1 << 3);
        }
        if (nvic_irq_info(ARMv7M_EXCP_SVC)->active) {
            value |= (1 << 7);
        }
        if (nvic_irq_info(ARMv7M_EXCP_DEBUG)->active) {
            value |= (1 << 8);
        }
        if (nvic_irq_info(ARMv7M_EXCP_PENDSV)->active) {
            value |= (1 << 10);
        }
        if (nvic_irq_info(ARMv7M_EXCP_SYSTICK)->active) {
            value |= (1 << 11);
        }
        if (nvic_irq_info(ARMv7M_EXCP_USAGE)->pending) {
            value |= (1 << 12);
        }
        if (nvic_irq_info(ARMv7M_EXCP_MEM)->pending) {
            value |= (1 << 13);
        }
        if (nvic_irq_info(ARMv7M_EXCP_SVC)->pending) {
            value |= (1 << 15);
        }
        if (nvic_irq_info(ARMv7M_EXCP_MEM)->enabled) {
            value |= (1 << 16);
        }
        if (nvic_irq_info(ARMv7M_EXCP_USAGE)->enabled) {
            value |= (1 << 18);
        }
        return value;
    }
    case SC_REG_SCR:
        return RUNTIME_sysctl_state.scr;
    case SC_REG_CFSR:
        return RUNTIME_sysctl_state.cfsr;
    case SC_REG_HFSR:
        return RUNTIME_sysctl_state.hfsr;
    default:
#if KVM_OPEN_DEBUG
        if (unlikely(should_output(RT_OUTPUT_SYSCTL))) {
            debug_clear();
            debug_append_str("[emu-sysctl] unhandled system register: 0x");
            debug_append_int(reg);
            debug_append_str("\n");
            debug_print();
        }
#endif
        guest_abort();
        return 0;
    }
}

/// @brief set value of registers in System Control Block
/// @param reg register id defined in global.h
/// @param value new register value
void sysctl_set(uint32_t reg, uint32_t value) {
    switch (reg)
    {
    case SC_REG_ICTR:
        return;
    case SC_REG_CPUID:
        return;
    case SC_REG_VTOR:
        RUNTIME_sysctl_state.vtor = value;
        return;
    case SC_REG_CPACR: {
        // we implement only the Floating Point extension's CP10/CP11
        RUNTIME_sysctl_state.cpacr = value & (0xf << 20);
        return;
    }
    case SC_REG_AIRCR: {
        if ((value >> SC_AIRCR_VECTKEY_SHIFT) != 0x05fa) {
            return;
        }
        if (value & SC_AIRCR_SYSRESETREQ_MASK) {
#if KVM_OPEN_DEBUG
            if (unlikely(should_output(RT_OUTPUT_SYSCTL))) {
                debug_clear();
                debug_append_str("[emu-sysctl] system request reset\n");
                debug_print();
            }
#endif
            guest_abort();
        }
        if (value & SC_AIRCR_VECTCLRACTIVE_MASK) {
#if KVM_OPEN_DEBUG
            if (unlikely(should_output(RT_OUTPUT_SYSCTL))) {
                debug_clear();
                debug_append_str("[emu-sysctl] setting VECTCLRACTIVE when not in DEBUG mode is UNPREDICTABLE");
                debug_print();
            }
#endif
            guest_abort();
        }
        if (value & SC_AIRCR_VECTRESET_MASK) {
#if KVM_OPEN_DEBUG
            if (unlikely(should_output(RT_OUTPUT_SYSCTL))) {
                debug_clear();
                debug_append_str("[emu-sysctl] setting VECTRESET when not in DEBUG mode is UNPREDICTABLE");
                debug_print();
            }           
#endif
            guest_abort();
        }
        // handling PRIGROUP
        RUNTIME_sysctl_state.aircr = value & SC_AIRCR_PRIGROUP_MASK;
        nvic_irq_update();
        return;
    }
    case SC_REG_SHPR1:
        nvic_irq_info(7)->prio = (value >> 24) & 0xff;
        nvic_irq_info(6)->prio = (value >> 16) & 0xff;
        nvic_irq_info(5)->prio = (value >> 8) & 0xff;
        nvic_irq_info(4)->prio = (value >> 0) & 0xff;
        nvic_irq_update();
        return;
    case SC_REG_SHPR2:
        nvic_irq_info(11)->prio = (value >> 24) & 0xff;
        nvic_irq_info(10)->prio = (value >> 16) & 0xff;
        nvic_irq_info(9)->prio = (value >> 8) & 0xff;
        nvic_irq_info(8)->prio = (value >> 0) & 0xff;
        nvic_irq_update();
        return;
    case SC_REG_SHPR3:
        nvic_irq_info(15)->prio = (value >> 24) & 0xff;
        nvic_irq_info(14)->prio = (value >> 16) & 0xff;
        nvic_irq_info(13)->prio = (value >> 8) & 0xff;
        nvic_irq_info(12)->prio = (value >> 0) & 0xff;
        nvic_irq_update();
        return;
    case SC_REG_FPCCR:
        /* check apsen, we directly set fpca bit enable in control */
        if (value >> 31) {
            uint32_t control = cpu_get_control();
            cpu_set_control(control | 0b100);
        } 
        RUNTIME_sysctl_state.fpccr = value;
        return;
    case SC_REG_ICSR:
        if (value & (1 << 31)) {
            nvic_set_pending(ARMv7M_EXCP_NMI);
        }
        if (value & (1 << 28)) {
		    /* exit execution if reach DEFAULT_MAX_INTERRUPTS */
            RUNTIME_nvic_state.irq_cnt += 1;
            if (RUNTIME_nvic_state.irq_cnt >= DEFAULT_MAX_INTERRUPTS) {
#if KVM_OPEN_DEBUG
                if (unlikely(should_output(RT_OUTPUT_NVIC))) {
                    debug_clear();
                    debug_append_str("[guest-nvic] max interrupts\n");
                    debug_print();
                }
#endif
                guest_exit();
            }

            nvic_set_pending(ARMv7M_EXCP_PENDSV);
        } else if (value & (1 << 27)) {
            nvic_clear_pending(ARMv7M_EXCP_PENDSV);
        }
        if (value & (1 << 26)) {
            nvic_set_pending(ARMv7M_EXCP_SYSTICK);
        } else if (value & (1 << 25)) {
            nvic_clear_pending(ARMv7M_EXCP_SYSTICK);
        }
        return;
    case SC_REG_CCR: {
        uint32_t mask = SC_CCR_STKALIGN_MASK | SC_CCR_BFHFNMIGN_MASK |
                        SC_CCR_DIV_0_TRP_MASK | SC_CCR_UNALIGN_TRP_MASK |
                        SC_CCR_USERSETMPEND_MASK | SC_CCR_NONBASETHRDENA_MASK;
        RUNTIME_sysctl_state.ccr = value & mask;
        return;
    }
    case SC_REG_SHCSR: {
        nvic_irq_info(ARMv7M_EXCP_MEM)->active = (value & (1 << 0)) != 0;
        nvic_irq_info(ARMv7M_EXCP_USAGE)->active = (value & (1 << 3)) != 0;
        nvic_irq_info(ARMv7M_EXCP_SVC)->active = (value & (1 << 7)) != 0;
        nvic_irq_info(ARMv7M_EXCP_PENDSV)->active = (value & (1 << 10)) != 0;
        nvic_irq_info(ARMv7M_EXCP_SYSTICK)->active = (value & (1 << 11)) != 0;
        nvic_irq_info(ARMv7M_EXCP_USAGE)->pending = (value & (1 << 12)) != 0;
        nvic_irq_info(ARMv7M_EXCP_MEM)->pending = (value & (1 << 13)) != 0;
        nvic_irq_info(ARMv7M_EXCP_SVC)->pending = (value & (1 << 15)) != 0;
        nvic_irq_info(ARMv7M_EXCP_MEM)->enabled = (value & (1 << 16)) != 0;
        nvic_irq_info(ARMv7M_EXCP_USAGE)->enabled = (value & (1 << 18)) != 0;
        nvic_irq_info(ARMv7M_EXCP_DEBUG)->active = (value & (1 << 8)) != 0;
        nvic_irq_update();
        return;
    }
    case SC_REG_SCR: {
        RUNTIME_sysctl_state.scr = value;
        return;
    }
    case SC_REG_CFSR: {
        RUNTIME_sysctl_state.cfsr = value;
        return;
    }
    case SC_REG_HFSR: {
        RUNTIME_sysctl_state.hfsr = value;
        return;
    }
    default:
#if KVM_OPEN_DEBUG
        if (unlikely(should_output(RT_OUTPUT_SYSCTL))) {
            debug_clear();
            debug_append_str("[emu-sysctl] unhandled system register: 0x");
            debug_append_int(reg);
            debug_print();
        }
#endif
        guest_abort();
    }
}

/// @brief mmio read handler
/// @param addr physical address of the read
/// @param pc program counter of the read
/// @param data pointer to the read data
/// @param len access length
/// @return 0 on success, other value on fail
int sysctl_handle_mmio_read(uint32_t addr, uint32_t pc, uint32_t *data, uint32_t len) {
    // check valid access length
    if (len > 0x4) {
#if KVM_OPEN_DEBUG
        if (unlikely(should_output(RT_OUTPUT_SYSCTL))) {
            debug_clear();
            debug_append_str("[emu-sysctl] read with wrong length: address = 0x");
            debug_append_int(addr);
            debug_append_str(", length = 0x");
            debug_append_int(len);
            debug_append_str(", PC = 0x");
            debug_append_int(pc);
            debug_append_str("\n");
            debug_print();
        }
#endif
        return -1;
    }

    // get register base address and the access offset in the register
    uint32_t value;
    uint32_t reg_base_addr = addr & 0xfffffffc;
    int access_offset_by_len = (addr - reg_base_addr) / len;

    switch (reg_base_addr) {
    case SC_REG_ICTR:
    case SC_REG_CCR:
    case SC_REG_CPUID:
    case SC_REG_ICSR:
    case SC_REG_FPCCR:
    case SC_REG_SHPR1:
    case SC_REG_SHPR2:
    case SC_REG_SHPR3:
    case SC_REG_VTOR:
    case SC_REG_CFSR:
    case SC_REG_AIRCR:
    case SC_REG_HFSR:
    case SC_REG_SHCSR:
    case SC_REG_SCR:
    case SC_REG_CPACR: {
        value = sysctl_get(reg_base_addr);
        break;
    }
    default: {
#if KVM_OPEN_DEBUG
        if (unlikely(should_output(RT_OUTPUT_SYSCTL))) {
            debug_clear();
            debug_append_str("[emu-sysctl] read wrong position: address = 0x");
            debug_append_int(addr);
            debug_append_str(", length = 0x");
            debug_append_int(len);
            debug_append_str(", PC = 0x");
            debug_append_int(pc);
            debug_append_str("\n");
            debug_print();
        }
#endif
        return -1;       
    }
    }

    // adjust value
    if (len == 0x1)
        ((uint8_t *)data)[0] = ((uint8_t *)&value)[access_offset_by_len];
    else if (len == 0x2)
        ((uint16_t *)data)[0] = ((uint16_t *)&value)[access_offset_by_len];
    else
        ((uint32_t *)data)[0] = ((uint32_t *)&value)[access_offset_by_len];

#if KVM_OPEN_DEBUG
    if (unlikely(should_output(RT_OUTPUT_SYSCTL))) {
        debug_clear();
        debug_append_str("[emu-sysctl] read register: address = 0x");
        debug_append_int(addr);
        debug_append_str(", length = 0x");
        debug_append_int(len);
        debug_append_str(", PC = 0x");
        debug_append_int(pc);
        debug_append_str(", value = 0x");
        debug_append_int(value);
        debug_append_str("\n");
        debug_print();
    }
#endif
    return 0;
}

/// @brief mmio write handler
/// @param addr physical address of the write
/// @param pc program counter of the write
/// @param data pointer to the write data
/// @param len access length
/// @return 0 on success, other value on fail
int sysctl_handle_mmio_write(uint32_t addr, uint32_t pc, uint32_t *data, uint32_t len) {
    if (len > 0x4) {
#if KVM_OPEN_DEBUG
        if (unlikely(should_output(RT_OUTPUT_SYSCTL))) {
            debug_clear();
            debug_append_str("[emu-sysctl] write with wrong length: address = 0x");
            debug_append_int(addr);
            debug_append_str(", length = 0x");
            debug_append_int(len);
            debug_append_str(", PC = 0x");
            debug_append_int(pc);
            debug_append_str("\n");
            debug_print();
        }
#endif
        return -1;
    }

    uint32_t value;
    uint32_t reg_base_addr = addr & 0xfffffffc;
    int access_offset_by_len = (addr - reg_base_addr) / len;

    switch (reg_base_addr) {
    case SC_REG_ICTR:
    case SC_REG_CCR:
    case SC_REG_CPUID:
    case SC_REG_ICSR:
    case SC_REG_FPCCR:
    case SC_REG_SHPR1:
    case SC_REG_SHPR2:
    case SC_REG_SHPR3:
    case SC_REG_SCR:
    case SC_REG_VTOR:
    case SC_REG_SHCSR:
    case SC_REG_AIRCR:
    case SC_REG_HFSR:
    case SC_REG_CFSR:
    case SC_REG_CPACR: {
        value = sysctl_get(reg_base_addr);
        if (len == 0x1)
            ((uint8_t *)&value)[access_offset_by_len] = ((uint8_t *)data)[0];
        else if (len == 0x2)
            ((uint16_t *)&value)[access_offset_by_len] = ((uint16_t *)data)[0];
        else
            ((uint32_t *)&value)[access_offset_by_len] = ((uint32_t *)data)[0];
        sysctl_set(reg_base_addr, value);
        break;
    }
    default: {
#if KVM_OPEN_DEBUG
        if (unlikely(should_output(RT_OUTPUT_SYSCTL))) {
            debug_clear();
            debug_append_str("[emu-sysctl] write wrong position: address = 0x");
            debug_append_int(addr);
            debug_append_str(", length = 0x");
            debug_append_int(len);
            debug_append_str(", PC = 0x");
            debug_append_int(pc);
            debug_append_str("\n");
            debug_print();
        }
#endif
        return -1;
    }
    }

#if KVM_OPEN_DEBUG
    if (unlikely(should_output(RT_OUTPUT_SYSCTL))) {
        debug_clear();
        debug_append_str("[emu-sysctl] write register: address = 0x");
        debug_append_int(addr);
        debug_append_str(", length = 0x");
        debug_append_int(len);
        debug_append_str(", PC = 0x");
        debug_append_int(pc);
        debug_append_str(", value = 0x");
        debug_append_int(value);
        debug_append_str("\n");
        debug_print();
    }
#endif
    return 0;
}
