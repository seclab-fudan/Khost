#include "en_board_para.h"
#include "dev_systick.h"
#include "data_def.h"

namespace khost {

#define EXTERNAL_IRQ_START 16

EmuSysTick::EmuSysTick(BoardPara *board) 
    : _board(board) {
    _inner = (EmuSystickState *)_board->gpa_to_hva(RT_PARA_emu_systick_state);
    _inner->enabled = 0;
    _inner->interrput = 0;
    _inner->core_lock = 0;
    _inner->count_flag = 0;
    _inner->last_block_cnt = 0;
    _inner->irq_cnt = 0;
    _inner->irq = EXTERNAL_IRQ_START;
    _inner->reload_value = SYSTICK_MIN_VALUE;
    reload();
}

EmuSysTick::~EmuSysTick() {

}

int EmuSysTick::handle_tick() {
#if KVM_OPEN_DEBUG
    info("handling systick!\n");
#endif

    if (_inner->interrput) {
#if KVM_OPEN_DEBUG
        info("pending irq-15\n");
#endif
        _board->nvic()->set_pending(15);
    }

    // trigger other irq
    int last_irq = _inner->irq;
    while (!_board->nvic()->is_irq_enabled(_inner->irq)) {
        // move on to next
        _inner->irq = (_inner->irq + 1) % NVIC_MAX_VECTORS;
        if (_inner->irq == 0)
            _inner->irq = EXTERNAL_IRQ_START;
        // we have already check a full round, exit loop
        if (_inner->irq == (uint32_t)last_irq) {
            last_irq = -1;
            break;
        }
    }
    if (last_irq != -1) {
        // trigger current enabled irq
#if KVM_OPEN_DEBUG
        info("pending irq-%u", _inner->irq);
#endif
        _board->nvic()->set_pending(_inner->irq);
        // move to next irq
        _inner->irq = (_inner->irq + 1) % NVIC_MAX_VECTORS;
        if (_inner->irq == 0) {
            _inner->irq = EXTERNAL_IRQ_START;
        }
    }
    // exit execution if reach DEFAULT_MAX_INTERRUPTS
    _inner->irq_cnt += 1;
    if (_inner->irq_cnt >= DEFAULT_MAX_INTERRUPTS) {
        return EXIT_TIMEOUT;
    }

    // update next irq cnt
    reload();

    return HANDLED;
}

void EmuSysTick::reload() {
    FirmwareState *fs = _board->firmware_state();

    _inner->last_block_cnt = fs->block_count;
    fs->block_next_irq = _inner->last_block_cnt + _inner->reload_value;
    flush_state();

#if KVM_OPEN_DEBUG
    info("set next irq to %u", _inner->last_block_cnt + _inner->reload_value);
#endif
}

int EmuSysTick::handle_read(uint64_t phys_addr, uint8_t *data, uint32_t len, 
                         uint32_t pc) {
    uint32_t val = 0, addr = phys_addr - 0xe000e010;
    switch (addr) {
    // Systick Control and Status
    case 0x0:
        if (_inner->enabled) {
            val |= SYSTICK_ENABLE;
        }
        if (_inner->interrput) {
            val |= SYSTICK_TICKINT;
        }
        if (_inner->core_lock) {
            val |= SYSTICK_CLKSOURCE;
        }
        if (_inner->count_flag) {
            val |= SYSTICK_COUNTFLAG;
        }
        // HACK (non-standard behavior):
        // In case firmware explicitly asks whether time has passed
        // multiple times within one systick period, indicate that it has.
        // This makes time go faster for firmware waiting in busy loops via
        // a SysTick polling mechanism (which we want it to get out of).
        _inner->count_flag = true;
        flush_state();
        break;
    // Systick Reload Value
    case 0x4: 
        // Strictly speaking only 24 bits are used for the reload val
        val = (_inner->reload_value * SYSTICK_SCALE) & SYSTICK_RELOAD_VAL_MASK;
        break;
    // Systick Current Value
    case 0x8: {
        uint32_t delta = 0, tick = 0;
        if (_board->firmware_state()->block_count > _inner->last_block_cnt) {
            delta = _board->firmware_state()->block_count - _inner->last_block_cnt;
        }
        if (delta < _inner->reload_value) {
            tick = _inner->reload_value - delta;
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
        error("bad read offset at 0x%x", addr);
#endif
        return EXIT_FIRMWARE_CRASH;
    }
    ((uint32_t *)data)[0] = val;
    return HANDLED;
}

int EmuSysTick::handle_write(uint64_t phys_addr, uint8_t *data, uint32_t len, 
                             uint32_t pc) {
    uint32_t addr = phys_addr - 0xe000e010;
    uint32_t value = ((uint32_t *)data)[0];

    switch(addr) {
    // Systick Control and Status
    case 0x0: { 
        // SysTick is only concerned with writing the 3 lowest bits
        // ENABLE, TICKINT, CLKSOURCE
        bool enabled = value & SYSTICK_ENABLE;
        bool interrupt = value & SYSTICK_TICKINT;
        bool core_lock = value & SYSTICK_CLKSOURCE;

        // Did the enable status change?
        // Did the clock source change?
        if (enabled && (!_inner->enabled || core_lock != _inner->core_lock)) {
            reload();
        }

        _inner->enabled = enabled;
        _inner->interrput = interrupt;
        _inner->core_lock = core_lock;
        // We will react to TICKINT as soon as the timer expires
        break;
    }
    // SysTick Reload Value
    case 0x4: {
        // restrict the value to something that makes sense to the emulator
        uint32_t tick = value / SYSTICK_SCALE;
        tick = std::max((uint32_t)SYSTICK_MIN_VALUE, tick);
        tick = std::min((uint32_t)SYSTICK_MAX_VALUE, tick);
        _inner->reload_value = tick & SYSTICK_RELOAD_VAL_MASK;
        break;
    }
    // SysTick Current Value
    case 0x8: {
        _inner->count_flag = false;
        reload();
        break;
    }
    default:
#if KVM_OPEN_DEBUG
        error("bad write offset at 0x%x", addr);
#endif
        return EXIT_FIRMWARE_CRASH;
    }
    flush_state();
    return HANDLED;
}

void EmuSysTick::flush_state() {
    _board->flush_memory(RT_PARA_emu_systick_state, sizeof(EmuSystickState));
}

/// @brief log info message
/// @param s info message
void EmuSysTick::info(const char *s, ...) { 
    if (unlikely(CONFIG.option_should_target_output(EN_OUTPUT_SYSTICK))) {
        if (CONFIG.option_output_level() <= OUTPUT_LEVEL_INFO) {
            char buffer[256];
            va_list args;
            va_start(args, s);
            vsprintf(buffer, s, args);
            va_end(args); 
            print_mutex.lock();
            printf("INFO  | host-systick: %s\n", buffer);
            print_mutex.unlock();
        }
    }
}

/// @brief log error message
/// @param s error message
void EmuSysTick::error(const char *s, ...) { 
    if (unlikely(CONFIG.option_should_target_output(EN_OUTPUT_SYSTICK))) {
        if (CONFIG.option_output_level() <= OUTPUT_LEVEL_ERROR) {
            char buffer[256];
            va_list args;
            va_start(args, s);
            vsprintf(buffer, s, args);
            va_end(args); 
            print_mutex.lock();
            printf("ERROR | host-systick: %s\n", buffer);
            print_mutex.unlock();
        }
    }
}

}  // namespace khost
