#include "en_board_para.h"
#include "dev_sysctl.h"
#include "dev_nvic.h"

namespace khost {

SysCtl::SysCtl(BoardPara *board, uint32_t base_addr)
    : MMIORegion(base_addr), _board(board) {
    _inner = (SysctlState *)_board->gpa_to_hva(RT_PARA_sysctl_state);
    init_reg_names();
}

// reset
// reset all registers
bool SysCtl::reset() {
    _inner->cpacr = 0;
    _inner->vtor = CONFIG.firmware_info()->load_base;
    _inner->fpccr = 0;
    _inner->aircr = 0;
    _inner->ccr = (1 << SC_CCR_STKALIGN_SHIFT);
    _inner->scr = 0;
    _inner->cfsr = 0;
    _inner->hfsr = 0;
    _inner->bfar = 0;
    _inner->mmfar = 0;
    return true;
}

// get
// read register
uint32_t SysCtl::get(uint32_t reg) {
    switch (reg)
    {
    case SC_REG_ICTR:
        return 0b1110;
    case SC_REG_CPUID:
        return CPUID_VALUE;
    case SC_REG_VTOR:
        return _inner->vtor;
    case SC_REG_CPACR:
        return _inner->cpacr;
    case SC_REG_AIRCR:
        return (_inner->aircr & SC_AIRCR_PRIGROUP_MASK) | 0xfa050000;
    case SC_REG_SHPR1:
        return (
            (_board->nvic()->irq_info(7)->prio << 24) |
            (_board->nvic()->irq_info(6)->prio << 16) |
            (_board->nvic()->irq_info(5)->prio <<  8) |
            (_board->nvic()->irq_info(4)->prio <<  0)
        );
    case SC_REG_SHPR2:
        return (
            (_board->nvic()->irq_info(11)->prio << 24) |
            (_board->nvic()->irq_info(10)->prio << 16) |
            (_board->nvic()->irq_info( 9)->prio <<  8) |
            (_board->nvic()->irq_info( 8)->prio <<  0)
        );
    case SC_REG_SHPR3:
        return (
            (_board->nvic()->irq_info(15)->prio << 24) |
            (_board->nvic()->irq_info(14)->prio << 16) |
            (_board->nvic()->irq_info(13)->prio <<  8) |
            (_board->nvic()->irq_info(12)->prio <<  0)
        );
    case SC_REG_FPCCR:
        return _inner->fpccr;
    case SC_REG_ICSR: {
        // vectactive 
        uint32_t value = _board->cpu()->ipsr();    
        // rettobase
        if (_board->nvic()->ret_to_base()) {
            value |= (1 << SC_ICSR_RETTOBASE_SHIFT);
        }
        // vectpending
        if (_board->nvic()->get_pending_irq()) {
            value |= (_board->nvic()->get_pending_irq() & 0x1ff) 
                     << SC_ICSR_VECTPENDING_SHIFT;
        }
        // isrpending
        if (_board->nvic()->isr_pending()) {
            value |= (1 << SC_ICSR_ISRPENDING_SHIFT);
        }
        // isrpreempt: RES0 when halting debug not implemented
        // pendstset
        if (_board->nvic()->irq_info(ARMv7M_EXCP_SYSTICK)->pending) {
            value |= (1 << SC_ICSR_PENDSTSET_SHIFT);
        }
        // pendsvset
        if (_board->nvic()->irq_info(ARMv7M_EXCP_PENDSV)->pending) {
            value |= (1 << SC_ICSR_PENDSVSET_SHIFT);
        }
        // nmipendset
        if (_board->nvic()->irq_info(ARMv7M_EXCP_NMI)->pending) {
            value |= (1 << SC_ICSR_NMIPENDSET_SHIFT);
        }
        return value;
    }
    case SC_REG_CCR:
        return _inner->ccr;
    case SC_REG_SHCSR: {
        uint32_t value = 0;
        if (_board->nvic()->irq_info(ARMv7M_EXCP_MEM)->active) {
            value |= (1 << 0);
        }
        if (_board->nvic()->irq_info(ARMv7M_EXCP_USAGE)->active) {
            value |= (1 << 3);
        }
        if (_board->nvic()->irq_info(ARMv7M_EXCP_SVC)->active) {
            value |= (1 << 7);
        }
        if (_board->nvic()->irq_info(ARMv7M_EXCP_DEBUG)->active) {
            value |= (1 << 8);
        }
        if (_board->nvic()->irq_info(ARMv7M_EXCP_PENDSV)->active) {
            value |= (1 << 10);
        }
        if (_board->nvic()->irq_info(ARMv7M_EXCP_SYSTICK)->active) {
            value |= (1 << 11);
        }
        if (_board->nvic()->irq_info(ARMv7M_EXCP_USAGE)->pending) {
            value |= (1 << 12);
        }
        if (_board->nvic()->irq_info(ARMv7M_EXCP_MEM)->pending) {
            value |= (1 << 13);
        }
        if (_board->nvic()->irq_info(ARMv7M_EXCP_SVC)->pending) {
            value |= (1 << 15);
        }
        if (_board->nvic()->irq_info(ARMv7M_EXCP_MEM)->enabled) {
            value |= (1 << 16);
        }
        if (_board->nvic()->irq_info(ARMv7M_EXCP_USAGE)->enabled) {
            value |= (1 << 18);
        }
        return value;
    }
    case SC_REG_SCR:
        return _inner->scr;
    case SC_REG_CFSR:
        return _inner->cfsr;
    case SC_REG_HFSR:
        return _inner->hfsr;
    default:
#if KVM_OPEN_DEBUG
        error("unhandled system register: %s", reg_name(reg));
#endif
        abort();
    }
}

// set 
// write registr
void SysCtl::set(uint32_t reg, uint32_t value) {
    switch (reg)
    {
    case SC_REG_ICTR:
        return;
    case SC_REG_CPUID:
        return;
    case SC_REG_VTOR:
        _inner->vtor = value;
        flush_state();
        return;
    case SC_REG_CPACR: {
        // we implement only the Floating Point extension's CP10/CP11
        _inner->cpacr = value & (0xf << 20);
        flush_state();
        return;
    }
    case SC_REG_AIRCR: {
        if ((value >> SC_AIRCR_VECTKEY_SHIFT) != 0x05fa) {
            return;
        }
        if (value & SC_AIRCR_SYSRESETREQ_MASK) {
#if KVM_OPEN_DEBUG
            error("system request reset");
#endif
            abort();
        }
        if (value & SC_AIRCR_VECTCLRACTIVE_MASK) {
#if KVM_OPEN_DEBUG
            error("setting VECTCLRACTIVE when not in DEBUG mode is UNPREDICTABLE");           
#endif
        }
        if (value & SC_AIRCR_VECTRESET_MASK) {
#if KVM_OPEN_DEBUG
            error("setting VECTRESET when not in DEBUG mode is UNPREDICTABLE");           
#endif
        }
        // handling PRIGROUP
        _inner->aircr = value & SC_AIRCR_PRIGROUP_MASK;
        flush_state();
        _board->nvic()->irq_update();
        return;
    }
    case SC_REG_SHPR1:
        _board->nvic()->irq_info(7)->prio = (value >> 24) & 0xff;
        _board->nvic()->irq_info(6)->prio = (value >> 16) & 0xff;
        _board->nvic()->irq_info(5)->prio = (value >> 8) & 0xff;
        _board->nvic()->irq_info(4)->prio = (value >> 0) & 0xff;
        _board->nvic()->irq_update();
        return;
    case SC_REG_SHPR2:
        _board->nvic()->irq_info(11)->prio = (value >> 24) & 0xff;
        _board->nvic()->irq_info(10)->prio = (value >> 16) & 0xff;
        _board->nvic()->irq_info(9)->prio = (value >> 8) & 0xff;
        _board->nvic()->irq_info(8)->prio = (value >> 0) & 0xff;
        _board->nvic()->irq_update();
        return;
    case SC_REG_SHPR3:
        _board->nvic()->irq_info(15)->prio = (value >> 24) & 0xff;
        _board->nvic()->irq_info(14)->prio = (value >> 16) & 0xff;
        _board->nvic()->irq_info(13)->prio = (value >> 8) & 0xff;
        _board->nvic()->irq_info(12)->prio = (value >> 0) & 0xff;
        _board->nvic()->irq_update();
        return;
    case SC_REG_FPCCR:
        // check apsen, we directly set fpca bit enable in control
        if (value >> 31) {
            uint32_t control = _board->cpu()->control();
            _board->cpu()->set_control(control | 0b100);
        } 
        _inner->fpccr = value;
        flush_state();
        return;
    case SC_REG_ICSR:
        if (value & (1 << 31)) {
            _board->nvic()->set_pending(ARMv7M_EXCP_NMI);
        }
        if (value & (1 << 28)) {
            _board->nvic()->set_pending(ARMv7M_EXCP_PENDSV);
        } else if (value & (1 << 27)) {
            _board->nvic()->clear_pending(ARMv7M_EXCP_PENDSV);
        }
        if (value & (1 << 26)) {
            _board->nvic()->set_pending(ARMv7M_EXCP_SYSTICK);
        } else if (value & (1 << 25)) {
            _board->nvic()->clear_pending(ARMv7M_EXCP_SYSTICK);
        }
        return;
    case SC_REG_CCR: {
        uint32_t mask = int(SC_CCR_STKALIGN_MASK) | int(SC_CCR_BFHFNMIGN_MASK) |
                        int(SC_CCR_DIV_0_TRP_MASK) | int(SC_CCR_UNALIGN_TRP_MASK) |
                        int(SC_CCR_USERSETMPEND_MASK) | int(SC_CCR_NONBASETHRDENA_MASK);
        _inner->ccr = value & mask;
        flush_state();
        return;
    }
    case SC_REG_SHCSR: {
        _board->nvic()->irq_info(ARMv7M_EXCP_MEM)->active = (value & (1 << 0)) != 0;
        _board->nvic()->irq_info(ARMv7M_EXCP_USAGE)->active = (value & (1 << 3)) != 0;
        _board->nvic()->irq_info(ARMv7M_EXCP_SVC)->active = (value & (1 << 7)) != 0;
        _board->nvic()->irq_info(ARMv7M_EXCP_PENDSV)->active = (value & (1 << 10)) != 0;
        _board->nvic()->irq_info(ARMv7M_EXCP_SYSTICK)->active = (value & (1 << 11)) != 0;
        _board->nvic()->irq_info(ARMv7M_EXCP_USAGE)->pending = (value & (1 << 12)) != 0;
        _board->nvic()->irq_info(ARMv7M_EXCP_MEM)->pending = (value & (1 << 13)) != 0;
        _board->nvic()->irq_info(ARMv7M_EXCP_SVC)->pending = (value & (1 << 15)) != 0;
        _board->nvic()->irq_info(ARMv7M_EXCP_MEM)->enabled = (value & (1 << 16)) != 0;
        _board->nvic()->irq_info(ARMv7M_EXCP_USAGE)->enabled = (value & (1 << 18)) != 0;
        _board->nvic()->irq_info(ARMv7M_EXCP_DEBUG)->active = (value & (1 << 8)) != 0;
        _board->nvic()->irq_update();
        return;
    }
    case SC_REG_SCR: {
        _inner->scr = value;
        flush_state();
        return;
    }
    case SC_REG_CFSR: {
        _inner->cfsr = value;
        flush_state();
        return;
    }
    case SC_REG_HFSR: {
        _inner->hfsr = value;
        flush_state();
        return;
    }
    case SC_REG_BFAR: {
        _inner->bfar = value;
        flush_state();
        return;
    }
    case SC_REG_MMFAR: {
        _inner->mmfar = value;
        flush_state();
        return;
    }
    default:
#if KVM_OPEN_DEBUG
        error("unhandled system register: %s", reg_name(reg));
#endif
        abort();
    }
}

// handle_read
// handle mmio read and change access bytes to 4
int SysCtl::handle_read(uint64_t phys_addr, uint8_t *data, uint32_t len, uint32_t pc) {
    if (len > 0x4) {
#if KVM_OPEN_DEBUG
        error("read 0x%x[%s] with wrong length 0x%x", 
              phys_addr, reg_name(phys_addr), len);
#endif
        return EXIT_FIRMWARE_CRASH;
    }

    uint8_t value[] = {0, 0, 0, 0};
    uint32_t reg_base_addr = phys_addr & 0xfffffffc;
    int access_offset_by_len = (phys_addr - reg_base_addr) / len;

    switch (reg_base_addr) {
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
        *((uint32_t *)value) = get(reg_base_addr);
        break;
    }
    default: {
#if KVM_OPEN_DEBUG
        error("read wrong position 0x%x[%s] with length 0x%x, PC=0x%08x", 
              phys_addr, reg_name(reg_base_addr), len, pc);
#endif
        return EXIT_INTERNAL_CRASH;        
    }
    }

    if (len == 0x1)
        ((uint8_t *)data)[0] = ((uint8_t *)value)[access_offset_by_len];
    else if (len == 0x2)
        ((uint16_t *)data)[0] = ((uint16_t *)value)[access_offset_by_len];
    else
        ((uint32_t *)data)[0] = ((uint32_t *)value)[access_offset_by_len];

#if KVM_OPEN_DEBUG
    info("read 0x%x[%s] with length 0x%x, PC=0x%08x (value=0x%x)", phys_addr, 
         reg_name(reg_base_addr), len, pc, value);
#endif
    return HANDLED;
}

// handle_write
// handle mmio write and change access bytes to 4
int SysCtl::handle_write(uint64_t phys_addr, uint8_t *data, uint32_t len, uint32_t pc) {
    if (len > 0x4) {
#if KVM_OPEN_DEBUG
        error("read 0x%x[%s] with wrong length 0x%x", 
              phys_addr, reg_name(phys_addr), len);
#endif
        return EXIT_FIRMWARE_CRASH;
    }

    uint32_t value;
    uint32_t reg_base_addr = phys_addr & 0xfffffffc;
    int access_offset_by_len = (phys_addr - reg_base_addr) / len;

    switch (reg_base_addr) {
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
        value = get(reg_base_addr);
        if (len == 0x1)
            ((uint8_t *)&value)[access_offset_by_len] = ((uint8_t *)data)[0];
        else if (len == 0x2)
            ((uint16_t *)&value)[access_offset_by_len] = ((uint16_t *)data)[0];
        else
            ((uint32_t *)&value)[access_offset_by_len] = ((uint32_t *)data)[0];
        set(reg_base_addr, value);
        break;
    }
    default: {
#if KVM_OPEN_DEBUG
        error("write wrong position 0x%x[%s] with length 0x%x, PC=0x%08x", 
              phys_addr, reg_name(reg_base_addr), len, pc);
#endif
        return EXIT_INTERNAL_CRASH;
    }
    }

#if KVM_OPEN_DEBUG
    info("write 0x%x[%s] with length 0x%x, PC=0x%08x (value=0x%x)", 
         phys_addr, reg_name(reg_base_addr), len, pc, value);
#endif
    return HANDLED;
}

// reg_name
// return human name of system control block registers
const char *SysCtl::reg_name(uint32_t reg) {
    static char unknown[] = "unknown";
    auto i = _reg_names.find(reg);
    if (i == _reg_names.end())
        return unknown;
    else
        return i->second.c_str();
}

// init_reg_names
// init register name of system control block registers 
void SysCtl::init_reg_names() {
    _reg_names[SC_REG_CPUID] = std::string("CPUID");
    _reg_names[SC_REG_ICSR] = std::string("ICSR");
    _reg_names[SC_REG_VTOR] = std::string("VTOR");
    _reg_names[SC_REG_AIRCR] = std::string("AIRCR");
    _reg_names[SC_REG_SCR] = std::string("SCR");
    _reg_names[SC_REG_CCR] = std::string("CCR");
    _reg_names[SC_REG_SHPR1] = std::string("SHPR1");
    _reg_names[SC_REG_SHPR2] = std::string("SHPR2");
    _reg_names[SC_REG_SHPR3] = std::string("SHPR3");
    _reg_names[SC_REG_SHCSR] = std::string("SHCSR");
    _reg_names[SC_REG_CFSR] = std::string("CFSR");
    _reg_names[SC_REG_HFSR] = std::string("HFSR");
    _reg_names[SC_REG_DFSR] = std::string("DFSR");
    _reg_names[SC_REG_MMFAR] = std::string("MMFAR");
    _reg_names[SC_REG_BFAR] = std::string("BFAR");
    _reg_names[SC_REG_AFSR] = std::string("AFSR");
    _reg_names[SC_REG_CPACR] = std::string("CPACR");
    _reg_names[SC_REG_FPCCR] = std::string("FPCCR");
}

void SysCtl::flush_state() {
    _board->flush_memory(RT_PARA_sysctl_state, sizeof(SysctlState));
}

/// @brief log info message
/// @param s info message
void SysCtl::info(const char *s, ...) { 
    if (unlikely(CONFIG.option_should_target_output(EN_OUTPUT_SYSCTL))) {
        if (CONFIG.option_output_level() <= OUTPUT_LEVEL_INFO) {
            char buffer[256];
            va_list args;
            va_start(args, s);
            vsprintf(buffer, s, args);
            va_end(args); 
            print_mutex.lock();
            printf("INFO  | host-sysctl: %s\n", buffer);
            print_mutex.unlock();
        }
    }
}

/// @brief log error message
/// @param s error message
void SysCtl::error(const char *s, ...) { 
    if (unlikely(CONFIG.option_should_target_output(EN_OUTPUT_SYSCTL))) {
        if (CONFIG.option_output_level() <= OUTPUT_LEVEL_ERROR) {
            char buffer[256];
            va_list args;
            va_start(args, s);
            vsprintf(buffer, s, args);
            va_end(args); 
            print_mutex.lock();
            printf("ERROR | host-sysctl: %s\n", buffer);
            print_mutex.unlock();
        }
    }
}

}  // namespace khost
