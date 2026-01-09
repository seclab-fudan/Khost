#include "rt_nvic.h"
#include "rt_trigger.h"
#include "string.h"

/// @brief reset nvic state
void nvic_reset() {
    RUNTIME_nvic_state.vectpending = 0;
    RUNTIME_nvic_state.num_irq = RUNTIME_num_irq;
    RUNTIME_nvic_state.num_prio_bits = NVIC_PRIO_BITS;
    RUNTIME_nvic_state.vectpending_prio = NVIC_NOEXC_PRIO;
    RUNTIME_nvic_state.exception_prio = NVIC_NOEXC_PRIO;
    RUNTIME_nvic_state.irq_cnt = 0;

    memset(RUNTIME_nvic_state.vectors, 0, sizeof(RUNTIME_nvic_state.vectors));
    RUNTIME_nvic_state.vectors[ARMv7M_EXCP_NMI].enabled     =  1;
    RUNTIME_nvic_state.vectors[ARMv7M_EXCP_SVC].enabled     =  1;
    RUNTIME_nvic_state.vectors[ARMv7M_EXCP_PENDSV].enabled  =  1;
    RUNTIME_nvic_state.vectors[ARMv7M_EXCP_SYSTICK].enabled =  1;
    RUNTIME_nvic_state.vectors[ARMv7M_EXCP_DEBUG].enabled   =  0;
    RUNTIME_nvic_state.vectors[ARMv7M_EXCP_RESET].prio      = -3;
    RUNTIME_nvic_state.vectors[ARMv7M_EXCP_NMI].prio        = -2;
    RUNTIME_nvic_state.vectors[ARMv7M_EXCP_HARD].prio       = -1;
    RUNTIME_nvic_state.vectors[ARMv7M_EXCP_HARD].enabled    =  1;
}

/// @brief nvic mmio read handler
/// @param phys_addr read address
/// @param pc read pc
/// @param data read data
/// @param len read length
/// @return 0 on succ, other on fail
int nvic_handle_mmio_read(uint32_t phys_addr, uint32_t pc, uint32_t *data, uint32_t len) {
    uint32_t val;
    uint32_t offset = phys_addr - NVIC_BASE_ADDR;
    unsigned i, startvec, end, size = len;

    switch (offset) {
    case 0x100 ... 0x13f: // NVIC Set enable
        offset += 0x80;
        /* fall through */
    case 0x180 ... 0x1bf: // NVIC Clear enable
        val = 0;
        startvec = 8 * (offset - 0x180) + NVIC_FIRST_IRQ;

        for (i = 0, end = size * 8; i < end && startvec + i < RUNTIME_nvic_state.num_irq; i++) {
            if (RUNTIME_nvic_state.vectors[startvec + i].enabled) {
                val |= (1 << i);
            }
        }
        break;
    case 0x200 ... 0x23f: // NVIC Set pending
        offset += 0x80;
    case 0x280 ... 0x2bf: // NVIC Clear pending
        val = 0;
        startvec = 8 * (offset - 0x280) + NVIC_FIRST_IRQ;

        for (i = 0, end = size * 8; i < end && startvec + i < RUNTIME_nvic_state.num_irq; i++) {
            if (RUNTIME_nvic_state.vectors[startvec + i].pending) {
                val |= (1 << i);
            }
        }
        break;
    case 0x300 ... 0x33f: // NVIC Active
        val = 0;
        startvec = 8 * (offset - 0x300) + NVIC_FIRST_IRQ;

        for (i = 0, end = size * 8; i < end && startvec + i < RUNTIME_nvic_state.num_irq; i++) {
            if (RUNTIME_nvic_state.vectors[startvec + i].active) {
                val |= (1 << i);
            }
        }
        break;
    case 0x400 ... 0x5ef: // NVIC Priority
        val = 0;
        startvec = offset - 0x400 + NVIC_FIRST_IRQ;

        for (i = 0; i < size && startvec + i < RUNTIME_nvic_state.num_irq; i++) {
            val |= RUNTIME_nvic_state.vectors[startvec + i].prio << (8 * i);
        }
        break;
    default:  
#if KVM_OPEN_DEBUG
        if (unlikely(should_output(RT_OUTPUT_NVIC))) {
            debug_clear();
            debug_append_str("[guest-nvic] failed to read from 0x");
            debug_append_int(phys_addr);
            debug_append_str("\n");
            debug_print();
        }
#endif
        return -1;
    }
#if KVM_OPEN_DEBUG
    if (unlikely(should_output(RT_OUTPUT_NVIC))) {
        debug_clear();
        debug_append_str("[guest-nvic] read from 0x");
        debug_append_int(phys_addr);
        debug_append_str("\n");
        debug_print();
    }
#endif
    return 0;
}

/// @brief nvic mmio write handler
/// @param phys_addr write address
/// @param pc write pc
/// @param data write data buffer
/// @param len write length
/// @return 0 on succ, other on fail
int nvic_handle_mmio_write(uint32_t phys_addr, uint32_t pc, uint32_t *data, uint32_t len) {
    uint32_t setval = 0;
    uint32_t offset = phys_addr - 0xE000E000;
    uint32_t value  = ((uint32_t*)data)[0];
    unsigned i, startvec, end, size = len;

    switch (offset) {
    case 0x100 ... 0x13f: // NVIC Set enable
        offset += 0x80;
        setval = 1;
        /* fall through */
    case 0x180 ... 0x1bf: // NVIC Clear enable
        startvec = 8 * (offset - 0x180) + NVIC_FIRST_IRQ;

        for (i = 0, end = size * 8; i < end && startvec + i < RUNTIME_nvic_state.num_irq; i++) {
            if (value & (1 << i)) {
                if (RUNTIME_nvic_state.vectors[startvec + i].enabled != setval) {
                    RUNTIME_nvic_state.vectors[startvec + i].enabled = setval;
#if KVM_OPEN_DEBUG
                    if (unlikely(should_output(RT_OUTPUT_NVIC))) {
                        debug_clear();
                        debug_append_str("[guest-nvic] change irq state: irq = 0x");
                        debug_append_int(startvec + i);
                        debug_append_str(", enable = 0x");
                        debug_append_int(setval);
                        debug_append_str(", pc = 0x");
                        debug_append_int(RUNTIME_firmware_context.pc);
                        debug_append_str("\n");
                        debug_print();
                    }
#endif
                    if (setval && startvec + i > ARMv7M_EXCP_SYSTICK) {
                        calc_next_irq_time(startvec + i);
                    }
                }
            }
        }
        nvic_irq_update();
        goto exit_ok;
    case 0x200 ... 0x23f: // NVIC Set pending
        offset += 0x80;
        setval = 1;
        /* fall through */
    case 0x280 ... 0x2bf: // NVIC Clear pending
        startvec = 8 * (offset - 0x280) + NVIC_FIRST_IRQ;

        for (i = 0, end = size * 8; i < end && startvec + i < RUNTIME_nvic_state.num_irq; i++) {
            if (value & (1 << i) 
                && !(setval == 0 && RUNTIME_nvic_state.vectors[startvec + i].level 
                && !RUNTIME_nvic_state.vectors[startvec + i].active)) {
#if KVM_OPEN_DEBUG
                if (unlikely(should_output(RT_OUTPUT_NVIC))) {
                    if (RUNTIME_nvic_state.vectors[startvec + i].pending != setval) {
                        debug_clear();
                        debug_append_str("[guest-nvic] change irq state: irq = 0x");
                        debug_append_int(startvec + i);
                        debug_append_str(", pending = 0x");
                        debug_append_int(setval);
                        debug_append_str(", pc = 0x");
                        debug_append_int(RUNTIME_firmware_context.pc);
                        debug_append_str("\n");
                        debug_print();
                    }
                }
#endif
                RUNTIME_nvic_state.vectors[startvec + i].pending = setval;
            }
        }
        nvic_irq_update();
        goto exit_ok;
    case 0x300 ... 0x33f: // NVIC Active
        goto exit_ok;
    case 0x400 ... 0x5ef: // NVIC Priority
        startvec = (offset - 0x400) + NVIC_FIRST_IRQ;

        for (i = 0; i < size && startvec + i < RUNTIME_nvic_state.num_irq; i++) {
            nvic_set_prio(startvec + i, 0, (value >> (i * 8)) & 0xff);
        }
        nvic_irq_update();
        goto exit_ok;
    }
#if KVM_OPEN_DEBUG
    if (unlikely(should_output(RT_OUTPUT_NVIC))) {
        debug_clear();
        debug_append_str("[guest-nvic] cannot handle write: value = 0x");
        debug_append_int(value);
        debug_append_str(", addr = 0x");
        debug_append_int(phys_addr);
        debug_append_str("\n");
        debug_print();
    }
#endif
    return -1;
exit_ok:
#if KVM_OPEN_DEBUG
    if (unlikely(should_output(RT_OUTPUT_NVIC))) {
        debug_clear();
        debug_append_str("[guest-nvic] write to 0x");
        debug_append_int(phys_addr);
        debug_append_str("\n");
        debug_print();
    }
#endif
    return 0;
}

/// @brief mark the specified exception as pending
/// @param irq pending irq
void nvic_set_pending(int irq) {
    // check irq number
    if (irq <= ARMv7M_EXCP_RESET || irq >= RUNTIME_nvic_state.num_irq) {
#if KVM_OPEN_DEBUG
        if (unlikely(should_output(RT_OUTPUT_NVIC))) {
            debug_clear();
            debug_append_str("[guest-nvic] set invalid pending irq\n");
            debug_print();
        }
#endif
        guest_abort();
    }

    NvicVecInfo *vec = &RUNTIME_nvic_state.vectors[irq];
#if KVM_OPEN_DEBUG
    if (unlikely(should_output(RT_OUTPUT_NVIC))) {
        debug_clear();
        debug_append_str("[guest-nvic] change irq pending: irq = 0x");
        debug_append_int(irq);
        debug_append_str(", enabled = 0x");
        debug_append_int(vec->enabled);
        debug_append_str(", prio = 0x");
        debug_append_int(vec->prio);
        debug_append_str("\n");
        debug_print();
    }
#endif

    // never set pending same irq twice 
    if (!vec->pending) {
        vec->pending = 1;
        nvic_irq_update();
    }
}

/// @brief clear pending state of exception
/// @param irq target irq
void nvic_clear_pending(int irq) {
    // check irq number
    if (irq <= ARMv7M_EXCP_RESET || irq >= RUNTIME_nvic_state.num_irq) {
#if KVM_OPEN_DEBUG
        if (unlikely(should_output(RT_OUTPUT_NVIC))) {
            debug_clear();
            debug_append_str("[guest-nvic] clear invalid pending irq\n");
            debug_print();
        }
#endif
        guest_abort();
    }
    
    NvicVecInfo *vec = &RUNTIME_nvic_state.vectors[irq];
    if (vec->pending) {
        vec->pending = 0;
        nvic_irq_update();
    }
}

__attribute__((section(".rt_dmz_handler")))
/// @brief get pending irq
/// @return highest priority pending
int nvic_get_pending_irq() {
    // return pending irq number
    return RUNTIME_nvic_state.vectpending;
}

/// @brief make highest priority pending exception active
void nvic_acknowledge_irq() {
    const int pending = RUNTIME_nvic_state.vectpending;
    const int running = nvic_exec_prio();
    NvicVecInfo *vec = &RUNTIME_nvic_state.vectors[pending];

    // check irq number
    if (pending <= ARMv7M_EXCP_RESET || pending >= RUNTIME_nvic_state.num_irq) {
#if KVM_OPEN_DEBUG
        if (unlikely(should_output(RT_OUTPUT_NVIC))) {
            debug_clear();
            debug_append_str("[guest-nvic] invalid acknowledge irq\n");
            debug_print();
        }
#endif
        guest_abort();    
    }

    // irq must be enabled first
    if (!vec->enabled) {    
#if KVM_OPEN_DEBUG
        if (unlikely(should_output(RT_OUTPUT_NVIC))) {
            debug_clear();
            debug_append_str("[guest-nvic] acknowledgeding irq not enabled\n");
            debug_print();
        }
#endif
        guest_abort();         
    }

    // irq must be pending first
    if (!vec->pending) {
#if KVM_OPEN_DEBUG
        if (unlikely(should_output(RT_OUTPUT_NVIC))) {
            debug_clear();
            debug_append_str("[guest-nvic] acknowledge irq not pending\n");
            debug_print();
        }
#endif
        guest_abort();  
    }

    // priority of the irq shoud not lower than current priority
    if (RUNTIME_nvic_state.vectpending_prio >= running) {
#if KVM_OPEN_DEBUG
        if (unlikely(should_output(RT_OUTPUT_NVIC))) {
            debug_clear();
            debug_append_str("[guest-nvic] acknowledge irq prio lower than running\n");
            debug_print();
        }
#endif
        guest_abort();  
    }

#if KVM_OPEN_DEBUG
    if (unlikely(should_output(RT_OUTPUT_NVIC))) {
        debug_clear();
        debug_append_str("[guest-nvic] acknowledge irq: 0x");
        debug_append_int(pending);
        debug_append_str(", prio = 0x");
        debug_append_int(RUNTIME_nvic_state.vectpending_prio);
        debug_append_str("\n");
        debug_print();
    }
#endif

    // set the irq to active mode
    vec->active = 1;
    vec->pending = 0;
    nvic_irq_update();
}

/// @brief complete specified interrupt or exception
/// @param irq irq to be complete
/// @return 
//   -1 if the irq was not active
//    1 if completing this irq brought us back to base (no active irqs)
//    0 if there is still an irq active after this one was completed 
int nvic_complete_irq(int irq) {
    // check the irq number
    if (irq <= ARMv7M_EXCP_RESET || irq >= RUNTIME_nvic_state.num_irq) {        
#if KVM_OPEN_DEBUG
        if (unlikely(should_output(RT_OUTPUT_NVIC))) {
            debug_clear();
            debug_append_str("[guest-nvic] invalid complete irq\n");
            debug_print();
        }
#endif
        guest_abort(); 
    }

#if KVM_OPEN_DEBUG
    if (unlikely(should_output(RT_OUTPUT_NVIC))) {
        debug_clear();
        debug_append_str("[guest-nvic] complete irq 0x");
        debug_append_int(irq);
        debug_append_str("\n");
        debug_print();
    }
#endif

    // get return info
    NvicVecInfo *vec = &RUNTIME_nvic_state.vectors[irq];
    int ret = vec->active ? nvic_ret_to_base() : -1;
    
    // deactive the irq
    vec->active = 0;

    // check if the irq is pending again during exception handling
    if (vec->level) {
        // Re-pending the exception if it's still held high 
        if (irq < NVIC_FIRST_IRQ) {
#if KVM_OPEN_DEBUG
            if (unlikely(should_output(RT_OUTPUT_NVIC))) {
                debug_clear();
                debug_append_str("[guest-nvic] invalid repending irq 0x");
                debug_append_int(irq);
                debug_append_str("\n");
                debug_print();
            }
#endif
            guest_abort();   
        }
        vec->pending = 1;
    }

    nvic_irq_update();
    return ret;
}

/// @brief get gprio mask
/// @return the group mask of the exception priority
uint32_t nvic_gprio_mask() { 
    // get prigroup from AIRCR.PRIGROUP
    uint32_t prigroup = (sysctl_get(SC_REG_AIRCR) >> 8) & 0b111;
    // return group mask
    return ~0U << (prigroup + 1); 
}

/// @brief get excution group priority
/// @param rawprio raw (group and subgroup) priority value
/// @return the group priority for the exception
int nvic_exc_group_prio(int rawprio) {
    // reset, NMI and HardFault have only group priority 
    if (rawprio < 0) {
        return rawprio;
    }
    rawprio &= nvic_gprio_mask();
    return rawprio;
}

/// @brief get execution priority
/// @return the current execution priority of the CPU
int nvic_exec_prio() {
    int boost_prio = NVIC_NOEXC_PRIO;
    
    // get cpu states
    uint32_t basepri = cpu_get_basepri();
    uint32_t faultmask = cpu_get_faultmask();
    uint32_t primask = cpu_get_primask();

    // check baserpi
    if (basepri > 0) {
        boost_prio = nvic_exc_group_prio(basepri);
    }
    // ckeck primask
    if (primask > 0) {
        boost_prio = 0;
    }
    // check faultmask
    if (faultmask > 0) {
        boost_prio = -1;
    }

    // return the highest priority of the exception and running
    // note: _exception_prio is cache of the highest active exception priority 
    return boost_prio < RUNTIME_nvic_state.exception_prio ? 
           boost_prio : RUNTIME_nvic_state.exception_prio;
}

/// @brief check primask, faultmask and basepri
/// @return 1 for can take pending exception
int nvic_can_take_pending_exception() {
    return nvic_exec_prio() > RUNTIME_nvic_state.vectpending_prio;
}

/// @brief recompute state and assert irq line accordingly
void nvic_irq_update() {
    // recompute state
    nvic_recompute_state();

    // check if we can trigger the irq
    if (RUNTIME_nvic_state.vectpending_prio < RUNTIME_nvic_state.exception_prio 
        && RUNTIME_nvic_state.vectpending_prio < nvic_exec_prio()) {
        RUNTIME_firmware_state.pending_irq = RUNTIME_nvic_state.vectpending;
    } else {
        RUNTIME_firmware_state.pending_irq = 0;
    }
}

/// @brief recompute cached states
///      _vectpending      = pending irq number;
///      _vectpending_prio = pending irq priority;
///      _exception_prio   = active irq priority;
void nvic_recompute_state() {
    int pend_prio   = NVIC_NOEXC_PRIO;
    int active_prio = NVIC_NOEXC_PRIO;
    int pend_irq    = 0;

    for (int i = 1; i < RUNTIME_nvic_state.num_irq; i++) {
        NvicVecInfo *vec = &RUNTIME_nvic_state.vectors[i];
        if (vec->enabled && vec->pending && vec->prio < pend_prio) {
            pend_prio = vec->prio;
            pend_irq = i;
        }
        if (vec->active && vec->prio < active_prio) {
            active_prio = vec->prio;
        }
    }
    if (active_prio > 0) {
        active_prio &= nvic_gprio_mask();
    }
    if (pend_prio > 0) {
        pend_prio &= nvic_gprio_mask();
    }

    RUNTIME_nvic_state.vectpending      = pend_irq;
    RUNTIME_nvic_state.vectpending_prio = pend_prio;
    RUNTIME_nvic_state.exception_prio   = active_prio;
}

/// @brief if return to base
/// @return value of the ISCR RETTOBASE bit:
///   true if there is exactly one active exception
///   false if there is more than one active exception or there are no active 
///   exceptions
int nvic_ret_to_base() {
    int nhand = 0;
    for (int irq = ARMv7M_EXCP_RESET; irq < RUNTIME_nvic_state.num_irq; irq++) {
        if (RUNTIME_nvic_state.vectors[irq].active) {
            nhand++;
            if (nhand == 2) {
                return 0;
            }
        }
    }
    return 1;
}

/// @brief set_prio
/// @param irq target irq number
/// @param secure security (always 0)
/// @param prio priority value
void nvic_set_prio(unsigned irq, int secure, uint8_t prio) {
    if (irq <= ARMv7M_EXCP_NMI || irq >= RUNTIME_nvic_state.num_irq) {
#if KVM_OPEN_DEBUG
        if (unlikely(should_output(RT_OUTPUT_NVIC))) {
            debug_clear();
            debug_append_str("[guest-nvic] invalid irq number 0x");
            debug_append_int(irq);
            debug_append_str("\n");
            debug_print();
        }
#endif
    }

    prio &= (((~0ULL) >> (64 - (RUNTIME_nvic_state.num_prio_bits))) << (8-RUNTIME_nvic_state.num_prio_bits));
    RUNTIME_nvic_state.vectors[irq].prio = prio;

#if KVM_OPEN_DEBUG
    if (unlikely(should_output(RT_OUTPUT_NVIC))) {
        debug_clear();
        debug_append_str("[guest-nvic] change irq 0x");
        debug_append_int(irq);
        debug_append_str(", prio = 0x");
        debug_append_int(prio);
        debug_append_str(", pc = 0x");
        debug_append_int(RUNTIME_firmware_context.pc);
        debug_append_str("\n");
        debug_print();
    }
#endif
}

/// @brief isr_pending
/// @return if isr is pending
int nvic_isr_pending() {
    if (RUNTIME_nvic_state.vectpending > NVIC_FIRST_IRQ) {
        return 1;
    }
    for (int irq = NVIC_FIRST_IRQ; irq < RUNTIME_nvic_state.num_irq; irq++) {
        if (RUNTIME_nvic_state.vectors[irq].pending) {
            return 1;
        }
    }
    return 0;
}

/// @brief get irq info structure 
/// @param irq irq number
/// @return pointer to info structure
NvicVecInfo* nvic_irq_info(int irq) { 
    return &RUNTIME_nvic_state.vectors[irq];
}

__attribute__((section(".rt_dmz_handler")))
/// @brief check if the irq is enabled
/// @param irq irq number
/// @return 1 for enabled, 0 for disabled
int nvic_is_irq_enabled(int irq) {
    return RUNTIME_nvic_state.vectors[irq].enabled;
}
