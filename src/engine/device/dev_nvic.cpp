#include "en_board_para.h"
#include "dev_nvic.h"

namespace khost {

NVIC::NVIC(BoardPara *board, uint32_t base_addr) 
    : MMIORegion(base_addr), _board(board) {
    _inner = (NvicState *)_board->gpa_to_hva(RT_PARA_nvic_state);
}

bool NVIC::reset() {
    _inner->vectpending      = 0;
    _inner->num_irq          = CONFIG.firmware_info()->num_irq;
    _inner->num_prio_bits    = NVIC_PRIO_BITS;
    _inner->vectpending_prio = NVIC_NOEXC_PRIO;
    _inner->exception_prio   = NVIC_NOEXC_PRIO;
    _inner->irq_cnt   		 = 0;

    memset(_inner->vectors, 0, sizeof(_inner->vectors));
    _inner->vectors[ARMv7M_EXCP_NMI].enabled     =  1;
    _inner->vectors[ARMv7M_EXCP_SVC].enabled     =  1;
    _inner->vectors[ARMv7M_EXCP_PENDSV].enabled  =  1;
    _inner->vectors[ARMv7M_EXCP_SYSTICK].enabled =  1;
    _inner->vectors[ARMv7M_EXCP_DEBUG].enabled   =  0;
    _inner->vectors[ARMv7M_EXCP_RESET].prio      = -3;
    _inner->vectors[ARMv7M_EXCP_NMI].prio        = -2;
    _inner->vectors[ARMv7M_EXCP_HARD].prio       = -1;
    _inner->vectors[ARMv7M_EXCP_HARD].enabled    =  1;

    return true;
}

int NVIC::handle_read(uint64_t phys_addr, uint8_t *data, uint32_t len, 
                       uint32_t pc) { 
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

        for (i = 0, end = size * 8; i < end && startvec + i < (unsigned)_inner->num_irq; i++) {
            if (_inner->vectors[startvec + i].enabled) {
                val |= (1 << i);
            }
        }
        break;
    case 0x200 ... 0x23f: // NVIC Set pending
        offset += 0x80;
    case 0x280 ... 0x2bf: // NVIC Clear pending
        val = 0;
        startvec = 8 * (offset - 0x280) + NVIC_FIRST_IRQ;

        for (i = 0, end = size * 8; i < end && startvec + i < (unsigned)_inner->num_irq; i++) {
            if (_inner->vectors[startvec + i].pending) {
                val |= (1 << i);
            }
        }
        break;
    case 0x300 ... 0x33f: // NVIC Active
        val = 0;
        startvec = 8 * (offset - 0x300) + NVIC_FIRST_IRQ;

        for (i = 0, end = size * 8; i < end && startvec + i < (unsigned)_inner->num_irq; i++) {
            if (_inner->vectors[startvec + i].active) {
                val |= (1 << i);
            }
        }
        break;
    case 0x400 ... 0x5ef: // NVIC Priority
        val = 0;
        startvec = offset - 0x400 + NVIC_FIRST_IRQ;

        for (i = 0; i < size && startvec + i < (unsigned)_inner->num_irq; i++) {
            val |= _inner->vectors[startvec + i].prio << (8 * i);
        }
        break;
    default:  
#if KVM_OPEN_DEBUG
        error("failed to read from 0x%x (+0x%x)", phys_addr, offset);
#endif
        return EXIT_FIRMWARE_CRASH;
    }
#if KVM_OPEN_DEBUG
    info("read from 0x%x (=%x)", phys_addr, val);
#endif
    return HANDLED;
}

int NVIC::handle_write(uint64_t phys_addr, uint8_t *data, uint32_t len, 
                       uint32_t pc) { 
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

        for (i = 0, end = size * 8; i < end && startvec + i < (unsigned)_inner->num_irq; i++) {
            if (value & (1 << i)) {
                if (_inner->vectors[startvec + i].enabled != setval) {
                    _inner->vectors[startvec + i].enabled = setval;
#if KVM_OPEN_DEBUG
                    info("change irq-%d: enable=%d", startvec+i, setval);
#endif
                    if (setval && startvec + i >= NVIC_FIRST_IRQ) {
                        set_pending(startvec + i);
                    }
                }
            }
        }
        irq_update();
        goto exit_ok;
    case 0x200 ... 0x23f: // NVIC Set pending
        offset += 0x80;
        setval = 1;
        /* fall through */
    case 0x280 ... 0x2bf: // NVIC Clear pending
        startvec = 8 * (offset - 0x280) + NVIC_FIRST_IRQ;

        for (i = 0, end = size * 8; i < end && startvec + i < (unsigned)_inner->num_irq; i++) {
            if (value & (1 << i) 
                && !(setval == 0 && _inner->vectors[startvec + i].level 
                && !_inner->vectors[startvec + i].active)) {
                if (_inner->vectors[startvec + i].pending != setval) {
#if KVM_OPEN_DEBUG
                    info("change irq-%d: pending=%d", startvec+i, setval);
#endif
                }
                _inner->vectors[startvec + i].pending = setval;
            }
        }
        irq_update();
        goto exit_ok;
    case 0x300 ... 0x33f: // NVIC Active
        goto exit_ok;
    case 0x400 ... 0x5ef: // NVIC Priority
        startvec = (offset - 0x400) + NVIC_FIRST_IRQ;

        for (i = 0; i < size && startvec + i < (unsigned)_inner->num_irq; i++) {
            set_prio(startvec + i, false, (value >> (i * 8)) & 0xff);
        }
        irq_update();
        goto exit_ok;
    }
#if KVM_OPEN_DEBUG
    error("cannot handle write value=0x%x to addr=0x%x", value, phys_addr);
#endif
    return EXIT_FIRMWARE_CRASH;
exit_ok:
    flush_state();
    return HANDLED;
}

// set_pending
// irq: pending irq
// mark the specified exception as pending
void NVIC::set_pending(int irq) {
    // check irq number
    if (irq <= ARMv7M_EXCP_RESET || irq >= _inner->num_irq) {
#if KVM_OPEN_DEBUG
        error("set invalid pending irq");
#endif
        abort();
    }

    NvicVecInfo *vec = &_inner->vectors[irq];
#if KVM_OPEN_DEBUG
    debug("change irq-%d pending: enabled=%d, prio=%d", irq, vec->enabled, 
          vec->prio);
#endif

    // never set pending same irq twice 
    if (!vec->pending) {
        vec->pending = 1;
        irq_update();
    }
}

// clear_pending
// irq: target irq
// clear pending state of exception
void NVIC::clear_pending(int irq) {
    // check irq number
    if (irq <= ARMv7M_EXCP_RESET || irq >= _inner->num_irq) {
#if KVM_OPEN_DEBUG
        error("clear invalid pending irq");
#endif
        abort();
    }
    
    NvicVecInfo *vec = &_inner->vectors[irq];
    if (vec->pending) {
        vec->pending = 0;
        irq_update();
    }
}

// get_pending_irq
// return: return highest priority pending
int NVIC::get_pending_irq() {
    // return pending irq number
    return _inner->vectpending;
}

// acknowledge_irq
// make highest priority pending exception active
void NVIC::acknowledge_irq() {
    const int pending = _inner->vectpending;
    const int running = exec_prio();
    NvicVecInfo *vec = &_inner->vectors[pending];

    // check irq number
    if (pending <= ARMv7M_EXCP_RESET || pending >= _inner->num_irq) {
#if KVM_OPEN_DEBUG
        error("invalid acknowledge irq");
#endif
        abort();      
    }

    // irq must be enabled first
    if (!vec->enabled) {
#if KVM_OPEN_DEBUG
        error("acknowledgeding irq not enabled");
#endif
        abort();          
    }

    // irq must be pending first
    if (!vec->pending) {
#if KVM_OPEN_DEBUG
        error("acknowledge irq not pending");
#endif
        abort();          
    }

    // priority of the irq shoud not lower than current priority
    if (_inner->vectpending_prio >= running) {
#if KVM_OPEN_DEBUG
        error("acknowledge irq prio %d lower than running %d",
              _inner->vectpending_prio, running);
#endif
        abort();             
    }

#if KVM_OPEN_DEBUG
    debug("acknowledge irq-%d: prio=%d", pending, _inner->vectpending_prio);
#endif

    // set the irq to active mode
    vec->active = 1;
    vec->pending = 0;
    irq_update();
}

// complete_irq
// return: 
//  -1 if the irq was not active
//   1 if completing this irq brought us back to base (no active irqs)
//   0 if there is still an irq active after this one was completed
// complete specified interrupt or exceptio
int NVIC::complete_irq(int irq) {
    // check the irq number
    if (irq <= ARMv7M_EXCP_RESET || irq >= _inner->num_irq) {
#if KVM_OPEN_DEBUG
        error("invalid complete irq");
#endif
        abort();        
    }
#if KVM_OPEN_DEBUG
    debug("complete irq-%d", irq);
#endif

    // get return info
    NvicVecInfo *vec = &_inner->vectors[irq];
    int ret = vec->active ? ret_to_base() : -1;
    
    // deactive the irq
    vec->active = 0;

    // check if the irq is pending again during exception handling
    if (vec->level) {
        // Re-pending the exception if it's still held high 
        if (irq < NVIC_FIRST_IRQ) {
#if KVM_OPEN_DEBUG
            error("invalid repending irq-%d", irq);
#endif
            abort();              
        }
        vec->pending = 1;
    }

    irq_update();
    return ret;
}

// gprio_mask
// return: the group mask of the exception priority
uint32_t NVIC::gprio_mask() { 
    // get prigroup from AIRCR.PRIGROUP
    uint32_t prigroup = (_board->sysctl()->get(SC_REG_AIRCR) >> 8) & 0b111;
    // return group mask
    return ~0U << (prigroup + 1); 
}

// exc_group_prio
// rawprio: raw (group and subgroup) priority value
// return: the group priority for the exception
int NVIC::exc_group_prio(int rawprio) {
    // reset, NMI and HardFault have only group priority 
    if (rawprio < 0) {
        return rawprio;
    }
    rawprio &= gprio_mask();
    return rawprio;
}

// exec_prio
// return: the current execution priority of the CPU
int NVIC::exec_prio() {
    int boost_prio = NVIC_NOEXC_PRIO;
    
    // get cpu states
    uint32_t basepri = _board->cpu()->basepri();
    uint32_t faultmask = _board->cpu()->faultmask();
    uint32_t primask = _board->cpu()->primask();

    // check baserpi
    if (basepri > 0) {
        boost_prio = exc_group_prio(basepri);
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
    return std::min(boost_prio, _inner->exception_prio);
}

// can_take_pending_exception
// check primask, faultmask and basepri
bool NVIC::can_take_pending_exception() {
    return exec_prio() > _inner->vectpending_prio;
}

// irq_update
// recompute state and assert irq line accordingly
void NVIC::irq_update() {
    // recompute state
    recompute_state();

    // check if we can trigger the irq
    if (_inner->vectpending_prio < _inner->exception_prio 
        && _inner->vectpending_prio < exec_prio()) {
        _board->irq_trigger(_inner->vectpending);
    } 
}

// recompute_state
// recompute cached states
//  _vectpending      = pending irq number;
//  _vectpending_prio = pending irq priority;
//  _exception_prio   = active irq priority;
void NVIC::recompute_state() {
    int pend_prio   = NVIC_NOEXC_PRIO;
    int active_prio = NVIC_NOEXC_PRIO;
    int pend_irq    = 0;

    for (int i = 1; i < _inner->num_irq; i++) {
        NvicVecInfo *vec = &_inner->vectors[i];
        if (vec->enabled && vec->pending && vec->prio < pend_prio) {
            pend_prio = vec->prio;
            pend_irq = i;
        }
        if (vec->active && vec->prio < active_prio) {
            active_prio = vec->prio;
        }
    }
    if (active_prio > 0) {
        active_prio &= gprio_mask();
    }
    if (pend_prio > 0) {
        pend_prio &= gprio_mask();
    }

    _inner->vectpending      = pend_irq;
    _inner->vectpending_prio = pend_prio;
    _inner->exception_prio   = active_prio;
    flush_state();

    if (_inner->vectpending != 0) {
#if KVM_OPEN_DEBUG
        debug("recompute state: pending irq-%d(prior=%d), active_prio=%d, exec_prior=%d", 
              _inner->vectpending, _inner->vectpending_prio, _inner->exception_prio, exec_prio());
#endif
    } else {
#if KVM_OPEN_DEBUG
        debug("recompute state: no irq pending, active_prio=%d, exec_prio=%d", 
            _inner->exception_prio, exec_prio());
#endif
    }
}

// ret_to_base:
// return: value of the ISCR RETTOBASE bit:
//   true if there is exactly one active exception
//   false if there is more than one active exception or there are no active 
//   exceptions
bool NVIC::ret_to_base() {
    int nhand = 0;
    for (int irq = ARMv7M_EXCP_RESET; irq < _inner->num_irq; irq++) {
        if (_inner->vectors[irq].active) {
            nhand++;
            if (nhand == 2) {
                return false;
            }
        }
    }
    return true;
}

// set_irq_level
// irqn: only user irq number
// level: 1 for high and 0 for low
void NVIC::set_irq_level(int irqn, int level) {
    if (irqn < NVIC_FIRST_IRQ || irqn >= _inner->num_irq) {
#if KVM_OPEN_DEBUG
        error("invalid irq number %d", irqn);
#endif
        abort();
    }
#if KVM_OPEN_DEBUG
    debug("change irq-%d: level=%d\n", irqn, level);
#endif

    NvicVecInfo *vec = &_inner->vectors[irqn];
    if (level != vec->level) {
        vec->level = level;
        if (level) {
            set_pending(irqn);
        }
    }
}

// systick_trigger
// level: 1 for high and 0 for low
// set systick irq level
void NVIC::systick_trigger(int level) {
#if KVM_OPEN_DEBUG
    debug("trigger systick: level=%d", level);
#endif
    if (level) {
        set_pending(ARMv7M_EXCP_SYSTICK);
    }
}

// svc_trigger
// level: 1 for high and 0 for low
// set svc irq level
void NVIC::svc_trigger(int level) {
#if KVM_OPEN_DEBUG
    debug("trigger systick: level=%d", level);
#endif
    if (level) {
        set_pending(ARMv7M_EXCP_SVC);
    }
}

// nmi trigger
// level: 1 for high and 0 for low
// set nmi irq level
void NVIC::nmi_trigger(int level) {
#if KVM_OPEN_DEBUG
    debug("trigger nmi: level=%d", level);
#endif
    if (level) {
        set_pending(ARMv7M_EXCP_NMI);
    }
}

// pendsv trigger
// level: 1 for high and 0 for low
// set nmi irq level
void NVIC::pendsv_trigger(int level) {
#if KVM_OPEN_DEBUG
    debug("trigger pendsv: level=%d", level);
#endif
    if (level) {
        set_pending(ARMv7M_EXCP_PENDSV);
    }
}

// set_prio
// irq: target irq number
// prio: priority value
void NVIC::set_prio(unsigned irq, bool secure, uint8_t prio) {
    if (irq <= ARMv7M_EXCP_NMI || irq >= (unsigned)_inner->num_irq) {
#if KVM_OPEN_DEBUG
        error("invalid irq number %d", irq);
#endif
        abort();
    }

    prio &= (((~0ULL) >> (64 - (_inner->num_prio_bits))) << (8-_inner->num_prio_bits));
    _inner->vectors[irq].prio = prio;

#if KVM_OPEN_DEBUG
    info("change irq-%d: prio=%d", irq, prio);
#endif
}

// isr_pending
// return: if isr is pending
bool NVIC::isr_pending() {
    if (_inner->vectpending > NVIC_FIRST_IRQ) {
        return true;
    }
    for (int irq = NVIC_FIRST_IRQ; irq < _inner->num_irq; irq++) {
        if (_inner->vectors[irq].pending) {
            return true;
        }
    }
    return false;
}

void NVIC::flush_state() {
    _board->flush_memory(RT_PARA_nvic_state, sizeof(NvicState));
}

/// @brief log error message
/// @param s error message
void NVIC::error(const char *s, ...) { 
    if (unlikely(CONFIG.option_should_target_output(EN_OUTPUT_NVIC))) {
        if (CONFIG.option_output_level() <= OUTPUT_LEVEL_ERROR) {
            char buffer[256];
            va_list args;
            va_start(args, s);
            vsprintf(buffer, s, args);
            va_end(args); 
            print_mutex.lock();
            printf("ERROR | host-nvic: %s\n", buffer);
            print_mutex.unlock();
        }
    }
}

/// @brief log info message
/// @param s info message
void NVIC::info(const char *s, ...) { 
    if (unlikely(CONFIG.option_should_target_output(EN_OUTPUT_NVIC))) {
        if (CONFIG.option_output_level() <= OUTPUT_LEVEL_INFO) {
            char buffer[256];
            va_list args;
            va_start(args, s);
            vsprintf(buffer, s, args);
            va_end(args); 
            print_mutex.lock();
            printf("INFO  | host-nvic: %s\n", buffer);
            print_mutex.unlock();
        }
    }
}

/// @brief log debug message
/// @param s debug message
void NVIC::debug(const char *s, ...) { 
    if (unlikely(CONFIG.option_should_target_output(EN_OUTPUT_NVIC))) {
        if (CONFIG.option_output_level() <= OUTPUT_LEVEL_DEBUG) {
            char buffer[256];
            va_list args;
            va_start(args, s);
            vsprintf(buffer, s, args);
            va_end(args); 
            print_mutex.lock();
            printf("DEBUG | host-nvic: %s\n", buffer);
            print_mutex.unlock();
        }
    }
}

}  // namespace khost
