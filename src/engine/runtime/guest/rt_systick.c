#include "rt_systick.h"

/// @brief reset systick timer
void systick_reset() {
    RUNTIME_emu_systick_state.enabled = 0;
    RUNTIME_emu_systick_state.interrput = 0;
    RUNTIME_emu_systick_state.core_lock = 0;
    RUNTIME_emu_systick_state.count_flag = 0;
    RUNTIME_emu_systick_state.last_block_cnt = 0;
    RUNTIME_emu_systick_state.irq_cnt = 0;
    RUNTIME_emu_systick_state.reload_value = SYSTICK_MIN_VALUE;
    systick_reload();
}

__attribute__((section(".rt_dmz_handler")))
/// @brief handle systick when timeout 
/// @return 0 on succ (current always succ)
int systick_handle_tick() {
#ifndef PARAVIRT
    asm volatile (
        "mov     r0, #0xf\n\t"
        "mcr     p15, 0, r0, c3, c0, 0\n\t"
        "isb\n\t"
        :
        :
        : "r0", "memory"
    );
#endif

#if KVM_OPEN_DEBUG
    if (unlikely(should_output(RT_OUTPUT_SYSTICK))) {
        debug_clear();
        debug_append_str("[guest-systick] handle systick\n");
        debug_print();
    }
#endif

    // only trigger after a sepecific init period
    if (RUNTIME_firmware_state.block_count >= RUNTIME_init_period) {
        if (nvic_is_irq_enabled(ARMv7M_EXCP_SYSTICK)
            && RUNTIME_emu_systick_state.interrput
        ) {
#if KVM_OPEN_DEBUG
            if (unlikely(should_output(RT_OUTPUT_SYSTICK))) {
                debug_clear();
                debug_append_str("[guest-systick] pending irq-15\n");
                debug_print();
            }
#endif
            nvic_set_pending(ARMv7M_EXCP_SYSTICK);
        }
    }

    /* exit execution if reach DEFAULT_MAX_INTERRUPTS */
    RUNTIME_emu_systick_state.irq_cnt += 1;
    if (RUNTIME_emu_systick_state.irq_cnt >= DEFAULT_MAX_INTERRUPTS) {
#if KVM_OPEN_DEBUG
        if (unlikely(should_output(RT_OUTPUT_SYSTICK))) {
            debug_clear();
            debug_append_str("[guest-systick] max interrupts\n");
            debug_print();
        }
#endif
        guest_exit();
    }

    /* update next irq cnt */
    systick_reload();

#ifndef PARAVIRT
    asm volatile (
        "mov     r0, #0x1\n\t"
        "mcr     p15, 0, r0, c3, c0, 0\n\t"
        "isb\n\t"
        :
        :
        : "r0", "memory"
    );
#endif
    return 0;
}

/// @brief reload timer value
void systick_reload() {
    RUNTIME_emu_systick_state.last_block_cnt = RUNTIME_firmware_state.block_count;
    RUNTIME_firmware_state.block_next_irq = RUNTIME_emu_systick_state.last_block_cnt + RUNTIME_emu_systick_state.reload_value;

#if KVM_OPEN_DEBUG
    if (unlikely(should_output(RT_OUTPUT_SYSTICK))) {
        debug_clear();
        debug_append_str("[guest-systick] set next irq to 0x");
        debug_append_int(RUNTIME_firmware_state.block_next_irq);
        debug_append_str("\n");
        debug_print();
    }
#endif
}

/// @brief handle systick mmio write
/// @param addr write adress
/// @param pc write pc
/// @param data write data buffer
/// @param len write length
/// @return 0 on succ, other on fail
int systick_handle_mmio_write(uint32_t addr, uint32_t pc, uint32_t *data, 
                              uint32_t len) {
    addr = addr - 0xe000e010;
    uint32_t value = ((uint32_t *)data)[0];

    switch(addr) {
    // Systick Control and Status
    case 0x0: { 
        // SysTick is only concerned with writing the 3 lowest bits
        // ENABLE, TICKINT, CLKSOURCE
        int enabled = value & SYSTICK_ENABLE;
        int interrupt = value & SYSTICK_TICKINT;
        int core_lock = value & SYSTICK_CLKSOURCE;

        // Did the enable status change?
        // Did the clock source change?
        if (enabled && (
            !RUNTIME_emu_systick_state.enabled 
            || core_lock != RUNTIME_emu_systick_state.core_lock)
        ) {
            systick_reload();
        }

        RUNTIME_emu_systick_state.enabled = enabled;
        RUNTIME_emu_systick_state.interrput = interrupt;
        RUNTIME_emu_systick_state.core_lock = core_lock;
        // We will react to TICKINT as soon as the timer expires
        break;
    }
    // SysTick Reload Value
    case 0x4: {
        // restrict the value to something that makes sense to the emulator
        uint32_t tick = value / SYSTICK_SCALE;
        tick = SYSTICK_MIN_VALUE > tick ? SYSTICK_MIN_VALUE : tick;
        tick = SYSTICK_MAX_VALUE > tick ? tick : SYSTICK_MAX_VALUE;
        RUNTIME_emu_systick_state.reload_value = tick & SYSTICK_RELOAD_VAL_MASK;
        break;
    }
    // SysTick Current Value
    case 0x8: {
        RUNTIME_emu_systick_state.count_flag = 0;
        systick_reload();
        break;
    }
    default:
#if KVM_OPEN_DEBUG
        if (unlikely(should_output(RT_OUTPUT_SYSTICK))) {
            debug_clear();
            debug_append_str("[guest-systick] bad write offset at 0x");
            debug_append_int(addr);
            debug_append_str("\n");
            debug_print();
        }
#endif
        return -1;
    }
    return 0;
}

/// @brief handle systick mmio read access
/// @param addr read address
/// @param pc read pc
/// @param data read data buffer
/// @param len read length
/// @return 0 for succ, other for fail
int systick_handle_mmio_read(uint32_t addr, uint32_t pc, uint32_t *data, 
                             uint32_t len) {
    uint32_t val = 0;
    addr = addr - 0xe000e010;
    switch (addr) {
    // Systick Control and Status
    case 0x0:
        if (RUNTIME_emu_systick_state.enabled) {
            val |= SYSTICK_ENABLE;
        }
        if (RUNTIME_emu_systick_state.interrput) {
            val |= SYSTICK_TICKINT;
        }
        if (RUNTIME_emu_systick_state.core_lock) {
            val |= SYSTICK_CLKSOURCE;
        }
        if (RUNTIME_emu_systick_state.count_flag) {
            val |= SYSTICK_COUNTFLAG;
        }
        // HACK (non-standard behavior):
        // In case firmware explicitly asks whether time has passed
        // multiple times within one systick period, indicate that it has.
        // This makes time go faster for firmware waiting in busy loops via
        // a SysTick polling mechanism (which we want it to get out of).
        RUNTIME_emu_systick_state.count_flag = 1;
        break;
    // Systick Reload Value
    case 0x4: 
        // Strictly speaking only 24 bits are used for the reload val
        val = (RUNTIME_emu_systick_state.reload_value * SYSTICK_SCALE) & SYSTICK_RELOAD_VAL_MASK;
        break;
    // Systick Current Value
    case 0x8: {
        uint32_t delta = 0, tick = 0;
        if (RUNTIME_firmware_state.block_count > RUNTIME_emu_systick_state.last_block_cnt) {
            delta = RUNTIME_firmware_state.block_count - RUNTIME_emu_systick_state.last_block_cnt;
        }
        if (delta < RUNTIME_emu_systick_state.reload_value) {
            tick = RUNTIME_emu_systick_state.reload_value - delta;
        }
        // Strictly speaking only 24 bits are used for the reload val
        val = (tick * SYSTICK_SCALE) & SYSTICK_RELOAD_VAL_MASK;
        break;
    }
    case 0xc:
        val = SYSTICK_CALIBRATION_VALUE;
        break;
    default:
#if KVM_OPEN_DEBUG
        if (unlikely(should_output(RT_OUTPUT_SYSTICK))) {
            debug_clear();
            debug_append_str("[guest-systick] bad read offset at 0x");
            debug_append_int(addr);
            debug_append_str("\n");
            debug_print();
        }
#endif
        return -1;
    }
    ((uint32_t *)data)[0] = val;
    return 0;
}
