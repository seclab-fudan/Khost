#include "en_cpu.h"
#include "en_board.h"

#include <errno.h>
#include <sys/prctl.h>
#include <sys/time.h>
#include <signal.h>
#include <capstone/capstone.h>

namespace khost {

//==============================================================================
// construct and destruct operations

/// @brief constructor
/// @param board belonging board
CPU::CPU(Board *board) : _board(board) {    
    _init_succ = false;

    // create vcpu, get vcpu file descriptor
    _vcpu_fd = ioctl(board->vm_fd(), KVM_CREATE_VCPU, 0);
    if (_vcpu_fd < 0) {
#if KVM_OPEN_DEBUG
        error("kvm failed to create vcpu");
#endif
        _vcpu_fd = -1;
        return;
    }

    // get kvm_run struct size
    _kvm_run_size = ioctl(board->sys_fd(), KVM_GET_VCPU_MMAP_SIZE, 0);
    if (_kvm_run_size < 0) {
#if KVM_OPEN_DEBUG
        error("kvm failed to get vcpu mmap size");
#endif
        _kvm_run_size = -1;
        return;
    }

    // map kvm_run to userspace
    _kvm_run = (struct kvm_run *)mmap(NULL, _kvm_run_size, 
                                      PROT_READ | PROT_WRITE, 
                                      MAP_SHARED, _vcpu_fd, 0);
    if (_kvm_run == MAP_FAILED) {
#if KVM_OPEN_DEBUG
        error("failed to map kvm_run struct");
#endif
        _kvm_run = nullptr;
        return;
    }

    // reset kvm exit reason
    _kvm_run->exit_reason = KVM_EXIT_UNKNOWN;
    
    // init cpu model, enable EL1, EL0 32bit
    struct kvm_vcpu_init vcpu_init;
    bzero(&vcpu_init, sizeof(vcpu_init));
    vcpu_init.target = KVM_ARM_TARGET_GENERIC_V8;
    vcpu_init.features[0] = (1 << KVM_ARM_VCPU_EL1_32BIT);
    if (ioctl(_vcpu_fd, KVM_ARM_VCPU_INIT, &vcpu_init) < 0) {
#if KVM_OPEN_DEBUG
        error("failed to init vcpu");
#endif
        return;
    }

    // init cpu pstate, set to MODE SYS
    if (!set_kvm_reg(ARMv8A_APSR, COMPAT_PSR_MODE_SYS)) {
#if KVM_OPEN_DEBUG
        error("failed to init pstate");
#endif
        return;
    }

    // init debug: we use bkpt to exit from guest
    if (!init_debug()) {
#if KVM_OPEN_DEBUG
        error("failed to enable debug");
#endif
        return;
    }

    _init_succ = true;
}

/// @brief destructor
CPU::~CPU() {
    // unmap kvm_struct memory
    if (_kvm_run_size >= 0) {
        munmap((void *)_kvm_run, _kvm_run_size);
    }
    // destory vcpu
    if (_vcpu_fd >= 0) {
        close(_vcpu_fd);
    }
}

/// @brief reset cpu state and registers 
/// @return true on success, false on fail
bool CPU::reset() {
    // make sure we are in ARM Sys Mode again, intrrupt masked
    if (!set_kvm_reg(
        ARMv8A_APSR, 
        COMPAT_PSR_MODE_SYS | COMPAT_PSR_F_BIT | COMPAT_PSR_I_BIT)
    ) {
#if KVM_OPEN_DEBUG
        error("failed to set pstate");
#endif
        return false;
    }

    // note: clear exit_reason because we may exit because of mmio
    _kvm_run->exit_reason = KVM_EXIT_UNKNOWN;

    // set sp value (directly write)
    uint32_t entry_sp = CONFIG.firmware_info()->initial_sp & 0xFFFFFFFC;
    if (!set_kvm_reg(ARMv8A_SP_usr, entry_sp)) {
#if KVM_OPEN_DEBUG
        error("failed to set sp");
#endif
        return false;
    }

    set_running(false);
    return true;
}

//==============================================================================
// debug stuff

/// @brief return if the cpu is single stepping
/// @return true for single stepping
bool CPU::is_single_stepping() {
    return _single_stepping;
}

/// @brief set if the cpu should single stepping
/// @param enable true for enable single stepping
/// @return true for success, false when fail
bool CPU::set_single_stepping(bool enable) {
    if (_single_stepping != enable) {
        _single_stepping = enable;
        return update_debug();
    }
    return true;
}

/// @brief enable software breakpoint for kvm
/// @return true on success, false on fail
bool CPU::init_debug() {
    _max_hw_wps = board()->check_extension(KVM_CAP_GUEST_DEBUG_HW_WPS);
    _max_hw_bps = board()->check_extension(KVM_CAP_GUEST_DEBUG_HW_BPS);
    _single_stepping = false;
    _hw_bps.clear();
    _hw_wps.clear();
    return update_debug();
}

/// @brief update debug state for kvm
/// @return true on success, false on fail
bool CPU::update_debug() {
    unsigned int debug_control = KVM_GUESTDBG_ENABLE | KVM_GUESTDBG_USE_SW_BP;
    if (_single_stepping) {
        debug_control |= KVM_GUESTDBG_SINGLESTEP;
    }
    if (_hw_bps.size() > 0 || _hw_wps.size() > 0) {
        debug_control |= KVM_GUESTDBG_USE_HW;
    }

	struct kvm_guest_debug debug = { .control = debug_control };
    memset(&debug.arch, 0, sizeof(struct kvm_guest_debug_arch));
    for (size_t i = 0; i < _hw_wps.size(); i++) {
        debug.arch.dbg_wcr[i] = _hw_wps[i].wcr;
        debug.arch.dbg_wvr[i] = _hw_wps[i].wvr;
    }
    for (size_t i = 0; i < _hw_bps.size(); i++) {
        debug.arch.dbg_bcr[i] = _hw_bps[i].bcr;
        debug.arch.dbg_bvr[i] = _hw_bps[i].bvr;
    }

	if (ioctl(_vcpu_fd, KVM_SET_GUEST_DEBUG, &debug) < 0) {
        return false;
    }
    return true;
}

static inline uint32_t extract32(uint32_t value, int start, int length) {
    assert(start >= 0 && length > 0 && length <= 32 - start);
    return (value >> start) & (~0U >> (32 - length));
}

static inline int64_t sextract64(uint64_t value, int start, int length) {
    assert(start >= 0 && length > 0 && length <= 64 - start);
    /* Note that this implementation relies on right shift of signed
     * integers being an arithmetic shift.
     */
    return ((int64_t)(value << (64 - length - start))) >> (64 - length);
}

static inline uint32_t deposit32(uint32_t value, int start, int length,
                                 uint32_t fieldval) {
    uint32_t mask;
    assert(start >= 0 && length > 0 && length <= 32 - start);
    mask = (~0U >> (32 - length)) << start;
    return (value & ~mask) | ((fieldval << start) & mask);
}

static inline uint64_t deposit64(uint64_t value, int start, int length,
                                 uint64_t fieldval) {
    uint64_t mask;
    assert(start >= 0 && length > 0 && length <= 64 - start);
    mask = (~0ULL >> (64 - length)) << start;
    return (value & ~mask) | ((fieldval << start) & mask);
}

static inline bool is_power_of_2(uint64_t value) {
    if (!value) {
        return false;
    }
    return !(value & (value - 1));
}

static inline int ctz32(uint32_t val) {
    return val ? __builtin_ctz(val) : 32;
}

static inline int clz32(uint32_t val) {
    return val ? __builtin_clz(val) : 32;
}

static inline int clo32(uint32_t val) {
    return clz32(~val);
}

static inline int ctz64(uint64_t val) {
    return val ? __builtin_ctzll(val) : 64;
}

/// @brief insert hardware breakpoint at addr
/// @param addr breakpoint address
/// @return true on success, false on fail
bool CPU::insert_hw_breakpoints(uint32_t addr) {
    for (auto i: *CONFIG.reloc_map()) {
        if (i.second == addr) {
            addr = i.first;
        }
    }
    HWBreakpoint brk = {
        .bcr = 0x1,                             /* BCR E=1, enable */
        .bvr = addr
    };
    if (_hw_bps.size() >= (size_t)_max_hw_bps) {
        return false;
    }
    brk.bcr = deposit32(brk.bcr, 1, 2, 0x3);   /* PMC = 11 */    
    if (addr % 4 == 0x2) {
        brk.bcr = deposit32(brk.bcr, 5, 4, 0b1100);   /* BAS = HIGH */
    } else {
        brk.bcr = deposit32(brk.bcr, 5, 4, 0b0011);   /* BAS = LOW */
    }
    _hw_bps.push_back(brk);
    return true;
}

/// @brief delete hardware breakpoint at addr
/// @param addr breakpoint address
/// @return true on success, false on fail
bool CPU::delete_hw_breakpoints(uint32_t addr) {
    for (auto i: *CONFIG.reloc_map()) {
        if (i.second == addr) {
            addr = i.first;
        }
    }
    for (auto i = _hw_bps.begin(); i != _hw_bps.end(); ++i) {
        if (i->bvr == addr) {
            _hw_bps.erase(i);
            return true;
        }
    }
    return false;
}

#define FIELD_DP64(storage, reg, field, val) ({                           \
    struct {                                                              \
        uint64_t v:SC_ ## reg ## _ ## field ## _LENGTH;                    \
    } _v = { .v = val };                                                  \
    uint64_t _d;                                                          \
    _d = deposit64((storage), SC_ ## reg ## _ ## field ## _SHIFT,          \
                  SC_ ## reg ## _ ## field ## _LENGTH, _v.v);              \
    _d; })

bool CPU::insert_hw_watchpoints(uint32_t addr, uint32_t len, int type) {
    HWWatchpoint wp = {
        .wcr = SC_DBGWCR_E_MASK, /* E=1, enable */
        .wvr = addr & (~0x7ULL),
        .details = { .vaddr = addr, .len = len }
    };

    if (_hw_wps.size() >= (size_t)_max_hw_wps) {
        return false;
    }

    /*
     * HMC=0 SSC=0 PAC=3 will hit EL0 or EL1, any security state,
     * valid whether EL3 is implemented or not
     */
    wp.wcr = FIELD_DP64(wp.wcr, DBGWCR, PAC, 3);

    switch (type) {
    case GDB_WATCHPOINT_READ:
        wp.wcr = FIELD_DP64(wp.wcr, DBGWCR, LSC, 1);
        wp.details.flags = BP_MEM_READ;
        break;
    case GDB_WATCHPOINT_WRITE:
        wp.wcr = FIELD_DP64(wp.wcr, DBGWCR, LSC, 2);
        wp.details.flags = BP_MEM_WRITE;
        break;
    case GDB_WATCHPOINT_ACCESS:
        wp.wcr = FIELD_DP64(wp.wcr, DBGWCR, LSC, 3);
        wp.details.flags = BP_MEM_ACCESS;
        break;
    default:
        return false;
    }
    if (len <= 8) {
        /* we align the address and set the bits in BAS */
        int off = addr & 0x7;
        int bas = (1 << len) - 1;

        wp.wcr = deposit32(wp.wcr, 5 + off, 8 - off, bas);
    } else {
        /* For ranges above 8 bytes we need to be a power of 2 */
        if (is_power_of_2(len)) {
            uint64_t bits = ctz64(len);

            wp.wvr &= ~((1 << bits) - 1);
            wp.wcr = FIELD_DP64(wp.wcr, DBGWCR, MASK, bits);
            wp.wcr = FIELD_DP64(wp.wcr, DBGWCR, BAS, 0xff);
        } else {
            return false;
        }
    }

    _hw_wps.push_back(wp);
    return true;
}

bool CPU::check_watchpoint_in_range(HWWatchpoint wp, uint32_t addr, uint32_t size) {
    uint64_t addr_top, addr_bottom = wp.wvr;
    int bas = extract32(wp.wcr, 5, 8);
    int mask = extract32(wp.wcr, 24, 4);

    if (mask) {
        addr_top = addr_bottom + (1 << mask);
    } else {
        /* BAS must be contiguous but can offset against the base
         * address in DBGWVR */
        addr_bottom = addr_bottom + ctz32(bas);
        addr_top = addr_bottom + ctz32(~(bas >> ctz32(bas)));
    }

    /* we consider the length of the memory access */
    if (addr < addr_top && addr + size > addr_bottom) {
        return true;
    }
    return false;
}

bool CPU::delete_hw_watchpoints(uint32_t addr, uint32_t len, int type) {
    for (auto i = _hw_wps.begin(); i != _hw_wps.end(); ++i) {
        if (check_watchpoint_in_range(*i, addr, len)) {
            _hw_wps.erase(i);
            return true;
        }
    }
    return false;
}

HWBreakpoint CPU::hit_hw_breakpoint() {
    if (_hit_point_id >= 0 && _hit_point_id < (int)_hw_bps.size()) {
        return _hw_bps[_hit_point_id];
    }
    return HWBreakpoint();
}

HWWatchpoint CPU::hit_hw_watchpoint() {
    if (_hit_point_id >= 0 && _hit_point_id < (int)_hw_wps.size()) {
        return _hw_wps[_hit_point_id];
    }
    return HWWatchpoint();
}

void CPU::remove_all_hw_breakpoints() {
    _hw_bps.clear();
    _hw_wps.clear();
}

//==============================================================================
// Execution

/// @brief set alarm for timeout
/// @param usec timeout interbal usec
/// @return true on success, false on fail
bool CPU::set_alarm(uint64_t usec) {
    struct itimerval new_timer;
    new_timer.it_interval.tv_sec  = 0;
    new_timer.it_interval.tv_usec = 0;
    new_timer.it_value.tv_sec     = usec / 1000000;
    new_timer.it_value.tv_usec    = usec % 1000000;

    if (setitimer(ITIMER_REAL, &new_timer, NULL)) {
        error("failed to set timeout");
        return false;
    }
    return true;
}

/// @brief main loop of the virtual cpu
/// @return 
///   0(EXIT_OK): firmware request to exit or input is used up
///   1(EXIT_FIRMWARE_CRASH): crash inside the firmware
///   2(EXIT_INTERNAL_CRASH): crash because of the engine
///   3(EXIT_TIMEOUT): thread killed by user or fuzzer
int CPU::loop() {
    int ret = EXIT_OK;
#if KVM_OPEN_DEBUG
    if (!update_debug()) {
        return EXIT_INTERNAL_CRASH;
    }
#endif
    set_running(true);
    
	while (running()) {
        // enter and exit vcpu
        ret = enter_exit();
        
        // handle KVM_RUN result
        if (ret == UNHANDLED) {
            switch (_kvm_run->exit_reason) {
            // handle exit by bkpt instruction
            case KVM_EXIT_DEBUG:
                ret = handle_bkpt_exit();
                break;
            // handle mmio exit
            case KVM_EXIT_MMIO:
                ret = handle_mmio_exit();
                break;
            // other exit will be treated as EXIT_INTERNAL_CRASH
            default:
#if KVM_OPEN_DEBUG
                error("Unhandled exit reason: %d", _kvm_run->exit_reason);
#endif
                ret = EXIT_INTERNAL_CRASH;            
            }
        }
        if (ret != HANDLED) {
            // check if we are return from error handler
            if (ret == EXIT_OK || ret == EXIT_TIMEOUT) {
                uint32_t excp_id = ipsr(); 
                if (excp_id == ARMv7M_EXCP_BUS || excp_id == ARMv7M_EXCP_MEM 
                    || excp_id == ARMv7M_EXCP_USAGE 
                    || excp_id == ARMv7M_EXCP_HARD
                ) {
                    ret = EXIT_FIRMWARE_CRASH;
                }
            }
            break;
        }
    }

    // for EXIT_* result or timeout, exit the loop
    if (!running()) {
        // cpu is not running, terminated by user or fuzzer
#if KVM_OPEN_DEBUG
        debug("cpu terminated by user or fuzzer, pc=0x%08x", get_kvm_reg(ARMv8A_PC));
#endif
        ret = EXIT_TIMEOUT;
    }
    set_running(false);
    return ret;
}

/// @brief enter and running the kvm guest and exit when there's exception to handle
/// @return 
///  -2(UNHANDLE): exception is not handled
///  -1(HANDLED): exception is handled, reenter the guest
///   1(EXIT_FIRMWARE_CRASH): crash inside the firmware
///   2(EXIT_INTERNAL_CRASH): crash because of the engine
///   3(EXIT_TIMEOUT): thread killed by user or fuzzer
int CPU::enter_exit() {
#if KVM_OPEN_DEBUG
    debug("kvm enter, pc=0x%08x", get_kvm_reg(ARMv8A_PC));
#endif
    int err = ioctl(_vcpu_fd, KVM_RUN, 0);    
#if KVM_OPEN_DEBUG
    debug("kvm exit, pc=0x%08x", get_kvm_reg(ARMv8A_PC));
#endif
    // handle exit reason
    if (err < 0 && (errno != EINTR && errno != EAGAIN)) {
#if KVM_OPEN_DEBUG
        error("KVM_RUN failed with code %d", err);
#endif
        dump_state();
        return EXIT_FIRMWARE_CRASH;
    }

    // handle EINTR
    if (err < 0 && errno == EINTR) {
        return HANDLED;;
    }
    return UNHANDLED;
}

/// @brief handle KVM_RUN exit reason KVM_EXIT_DEBUG
/// @return 
///  -1(HANDLED): reenter the guest
///   0(EXIT_OK): firmware request to exit
///   1(EXIT_FIRMWARE_CRASH): crash inside the firmware
///   2(EXIT_INTERNAL_CRASH): crash because of the engine
///   3(EXIT_TIMEOUT): thread killed by user or fuzzer
int CPU::handle_bkpt_exit() {
    uint32_t pc             = get_kvm_reg(ARMv8A_PC);
    uint32_t arm_asm_code   = _board->read_u32(pc);
    uint32_t thumb_asm_code = arm_asm_code & 0xffff;
    int ret                 = HANDLED;

    uint32_t hsr_ec = ESR_ELx_EC(_kvm_run->debug.arch.hsr);
    switch (hsr_ec)
    {
    case 0b110010: /* Software Step exception from a lower Exception level */
        if (_single_stepping) {
            if (pc >= RUNTIME_START && pc <= RUNTIME_END) {
                return HANDLED;
            }
            if (pc >= CONFIG.firmware_info()->addition_start && pc < CONFIG.firmware_info()->addition_start + CONFIG.firmware_info()->addition_size) {
                if (CONFIG.reloc_map()->find(pc) == CONFIG.reloc_map()->end()) {
                    // pc is not relocated from firmware
                    return HANDLED;
                }
            }
#if KVM_OPEN_DEBUG
            error("Single Step: PC=0x%08x", CONFIG.reloc_pc(pc));
#endif
            return EXIT_DEBUG_SINGLESTEP;
        }
        break;
    case 0b111000: /* BKPT instruction execution in AArch32 state */
        // handle arm bkpt instructions
        ret = handle_arm_bkpt_exit(arm_asm_code);
        if (ret != UNHANDLED) {
            return ret;
        }
        // handle thumb bkpt instructions
        ret = handle_thumb_bkpt_exit(pc, thumb_asm_code);
        if (ret != UNHANDLED) {
            return ret;
        }
        break;
    case 0b110000: /* Breakpoint exception from a lower Exception level */
        for (size_t i = 0; i < _hw_bps.size(); i++) {
            if (_hw_bps[i].bvr == pc) {
                _hit_point_id = i;
#if KVM_OPEN_DEBUG
                info("hwbp: PC=0x%08x i=%d", CONFIG.reloc_pc(pc), i);
#endif
                return EXIT_DEBUG_BREAKPOINT;
            }
        }
#if KVM_OPEN_DEBUG
        info("hwbp: PC=0x%08x unknown breakpoint", CONFIG.reloc_pc(pc));
#endif
        break;
    case 0b110100: { /* Watchpoint from a lower Exception level */
        uint64_t addr, access_len;
        addr = decode_insn(&access_len);
        if (addr != (uint64_t)-1) {
            for (size_t i = 0; i < _hw_wps.size(); i++) {
                if (check_watchpoint_in_range(_hw_wps[i], addr, access_len)) {
                    _hit_point_id = i;
#if KVM_OPEN_DEBUG
                    info("hwwp: PC=0x%08x i=%d", CONFIG.reloc_pc(pc), i);
#endif
                    return EXIT_DEBUG_WATCHPOINT;
                }
            }
        }
#if KVM_OPEN_DEBUG
        info("hwwp: PC=0x%08x unknown breakpoint", CONFIG.reloc_pc(pc));
#endif
        break;
    }
    default:
        break;
    }
#if KVM_OPEN_DEBUG
    error("should not reach here, ec=0x%x, pc=0x%08x, code=0x%08x", 
          hsr_ec, pc, arm_asm_code);
    dump_state();
#endif
    return EXIT_FIRMWARE_CRASH;
}

/// @brief handle KVM_RUN exit reason KVM_EXIT_MMIO
/// @return 
///  -1(HANDLED): reenter the guest
///   0(EXIT_OK): firmware request to exit or input is used up
///   2(EXIT_INTERNAL_CRASH): crash because of the engine
int CPU::handle_mmio_exit() {
    // uint32_t gpa  = _kvm_run->mmio.phys_addr;
    // uint32_t len  = _kvm_run->mmio.len;

    // error("invalid mmio access with pc=0x%x", get_kvm_reg(ARMv8A_PC));
    // error("    gpa:  0x%x", gpa);
    // error("    len:  0x%x", len);
    // dump_state();
    // return EXIT_FIRMWARE_CRASH;
    return HANDLED;
}

/// @brief get the value of the _is_running
/// @return true for running, false for exiting
bool CPU::running() {
    return __atomic_load_n(&_is_running, __ATOMIC_RELAXED);
}

/// @brief set _is_running in CPU instance, vcpu will exit next loop if the value is false
/// @param value true for running, false for exiting
void CPU::set_running(bool value) {
    __atomic_store_n(&_is_running, value, __ATOMIC_RELAXED);    
}

//==============================================================================
// Register Operations

/// @brief read 32bit registers in AArch32, this function maps AArch32 to AArch64
/// @param id ARMv8A_* registers
/// @return  value of the register
uint32_t CPU::get_kvm_reg(uint64_t id) {
    switch (id)
    {
    case ARMv8A_FPSCR: {
        // FPSCR is a merge of FPSR and FPCR
        // ref: https://developer.arm.com/documentation/ddi0595/2020-12/AArch32-Registers/FPSCR--Floating-Point-Status-and-Control-Register?lang=en
        uint32_t fpsr = do_get_kvm_reg(ARMv8A_FPSR);
        uint32_t fpcr = do_get_kvm_reg(ARMv8A_FPCR);
        uint32_t fpscr = 0;
        fpscr |= fpsr & 0b11111000000000000000000010011111;
        fpscr |= fpcr & 0b00000111111111111001111100000000;
        return fpscr;
    }
    // ARMv8A_* equals to kvm id
    default:
        return do_get_kvm_reg(id);
    }
}

/// @brief return floating point register value S*
/// @param id VFP register S*
/// @return value of the floating point register 
uint32_t CPU::get_kvm_fp_reg(int id) {
    uint64_t v_id = id / 4;
    uint64_t offset_id = id % 4;
    uint32_t data[4];
    struct kvm_one_reg reg = {
        .id   = ARMv8A_V0 + ARMv8A_V_OFFSET * v_id,
        .addr = (uint64_t) (unsigned long)&data,
    };
	if (ioctl(_vcpu_fd, KVM_GET_ONE_REG, &reg) < 0) {
#if KVM_OPEN_DEBUG
        error("failed to get kvm fp register %d", id);
#endif
		return -1;
    }
    return data[offset_id];
}

/// @brief read 32bit register with KVM API (AArch64 mode)
/// @param kvm_id  KVM_GET_ONE_REG id
/// @return value of the kvm registe
uint32_t CPU::do_get_kvm_reg(uint64_t kvm_id) {
    uint32_t data[2];
    struct kvm_one_reg reg = {
        .id   = kvm_id,
        .addr = (uint64_t) (unsigned long)&data,
    };
	if (ioctl(_vcpu_fd, KVM_GET_ONE_REG, &reg) < 0) {
#if KVM_OPEN_DEBUG
        error("failed to get kvm register 0x%016llx", kvm_id);
#endif
        return -1;
    }
    return data[0];
}

/// @brief write 32bit registers in AArch32, this function maps AArch32 to AArch64
/// @param id ARMv8A_* registers
/// @param value new value of register
/// @return true on success, false on fail
bool CPU::set_kvm_reg(uint64_t id, uint32_t value) {
    switch (id)
    {
    case ARMv8A_FPSCR: {
        // FPSCR is a merge of FPSR and FPCR
        // ref: https://developer.arm.com/documentation/ddi0595/2020-12/AArch32-Registers/FPSCR--Floating-Point-Status-and-Control-Register?lang=en
        uint32_t fpsr_mask = 0b11111000000000000000000010011111;
        uint32_t fpcr_mask = 0b00000111111111111001111100000000;
        uint32_t fpsr = do_get_kvm_reg(ARMv8A_FPSR) & (~fpsr_mask);
        uint32_t fpcr = do_get_kvm_reg(ARMv8A_FPCR) & (~fpcr_mask);
        fpsr |= value & fpsr_mask;
        fpcr |= value & fpcr_mask;
        return do_set_kvm_reg(ARMv8A_FPSR, fpsr) &&
               do_set_kvm_reg(ARMv8A_FPCR, fpcr);
    }
    // ARMv8A_* equals to kvm id
    default:
        return do_set_kvm_reg(id, value);
    }
}

/// @brief write 32bit floating point registers
/// @param id VFP register S*
/// @param value new value of register
/// @return true on success, false on fail
bool CPU::set_kvm_fp_reg(int id, uint32_t value) {
    uint64_t v_id = id / 4;
    uint64_t offset_id = id % 4;
    uint32_t data[4];
    struct kvm_one_reg reg = {
        .id   = ARMv8A_V0 + ARMv8A_V_OFFSET * v_id,
        .addr = (uint64_t) (unsigned long)&data,
    };
	if (ioctl(_vcpu_fd, KVM_GET_ONE_REG, &reg) < 0) {
		return false;
    }
    data[offset_id] = value;
	if (ioctl(_vcpu_fd, KVM_SET_ONE_REG, &reg) < 0) {
#if KVM_OPEN_DEBUG
        error("failed to write S%u", id);
#endif
        return false;
    }
    return true;
}

/// @brief write 32bit register with KVM API (AArch64 mode)
/// @param kvm_id KVM_GET_ONE_REG id
/// @param value new value of register 
/// @return true on success, false on fail
bool CPU::do_set_kvm_reg(uint64_t kvm_id, uint32_t value) {
    uint32_t data[2] = {value, 0};
    struct kvm_one_reg reg = {
        .id = kvm_id,
        .addr = (uint64_t) (unsigned long)&data,
    };
	if (ioctl(_vcpu_fd, KVM_SET_ONE_REG, &reg) < 0) {
#if KVM_OPEN_DEBUG
        error("failed to write register 0x%016llx", kvm_id);
#endif
        return false;
    }
    return true;
}

/// @brief gets current apsr, only return bits used in ARMv7-M profile
/// @return value of the apsr
uint32_t CPU::apsr() {
    // ARMv7M APSR is part of ARMv8A pstate
    uint32_t pstate = get_kvm_reg(ARMv7M_APSR);
    return pstate & (ARMv7M_PSR_N_MASK | ARMv7M_PSR_Z_MASK | ARMv7M_PSR_C_MASK | 
                     ARMv7M_PSR_V_MASK | ARMv7M_PSR_Q_MASK | ARMv7M_PSR_GE_MASK);
}

/// @brief gets current ipsr
/// @return value of the ipsr
uint32_t CPU::ipsr() { 
    return inner()->ipsr; 
}

/// @brief gets current epsr
/// @return value of the epsr
uint32_t CPU::epsr() {
    uint32_t pstate = get_kvm_reg(ARMv7M_APSR);
    pstate &= ARMv7M_PSR_ICIIT_MASK;
    pstate |= (0b1 << ARMv7M_PSR_T_OFFSET);
    return pstate;
}

/// @brief gets current valuse of xpsr
/// @return value of the xpsr
uint32_t CPU::xpsr() { 
    return apsr() | ipsr() | epsr(); 
}

//==============================================================================
// Output Functions

/// @brief log error message
/// @param s error message
void CPU::error(const char *s, ...) { 
    if (unlikely(CONFIG.option_should_target_output(EN_OUTPUT_CPU))) {
        if (CONFIG.option_output_level() <= OUTPUT_LEVEL_ERROR) {
            char buffer[256];
            va_list args;
            va_start(args, s);
            vsprintf(buffer, s, args);
            va_end(args); 
            print_mutex.lock();
            printf("ERROR | host-cpu: %s\n", buffer);
            print_mutex.unlock();
        }
    }
}

/// @brief log info message
/// @param s info message
void CPU::info(const char *s, ...) { 
    if (unlikely(CONFIG.option_should_target_output(EN_OUTPUT_CPU))) {
        if (CONFIG.option_output_level() <= OUTPUT_LEVEL_INFO) {
            char buffer[256];
            va_list args;
            va_start(args, s);
            vsprintf(buffer, s, args);
            va_end(args); 
            print_mutex.lock();
            printf("INFO  | host-cpu: %s\n", buffer);
            print_mutex.unlock();
        }
    }
}

/// @brief log debug message
/// @param s debug message
void CPU::debug(const char *s, ...) { 
    if (unlikely(CONFIG.option_should_target_output(EN_OUTPUT_CPU))) {
        if (CONFIG.option_output_level() <= OUTPUT_LEVEL_DEBUG) {
            char buffer[256];
            va_list args;
            va_start(args, s);
            vsprintf(buffer, s, args);
            va_end(args); 
            print_mutex.lock();
            printf("DEBUG | host-cpu: %s\n", buffer);
            print_mutex.unlock();
        }
    }
}

/// @brief dump all CPU states and registers
void CPU::dump_state() {
    error("====================== PC @ 0x%08x ======================", CONFIG.reloc_pc(get_kvm_reg(ARMv8A_PC)));
    // basic registers
    error("ARMv7-M Core Registers:");
    error("   R0=0x%08x   R1=0x%08x   R2=0x%08x", 
          get_kvm_reg(ARMv8A_R0), get_kvm_reg(ARMv8A_R1), get_kvm_reg(ARMv8A_R2));
    error("   R3=0x%08x   R4=0x%08x   R5=0x%08x", 
          get_kvm_reg(ARMv8A_R3), get_kvm_reg(ARMv8A_R4), get_kvm_reg(ARMv8A_R5));
    error("   R6=0x%08x   R7=0x%08x   R8=0x%08x", 
          get_kvm_reg(ARMv8A_R6), get_kvm_reg(ARMv8A_R7), get_kvm_reg(ARMv8A_R8_usr));
    error("   R9=0x%08x  R10=0x%08x  R11=0x%08x", 
          get_kvm_reg(ARMv8A_R9_usr), get_kvm_reg(ARMv8A_R10_usr), get_kvm_reg(ARMv8A_R11_usr));
    error("  R12=0x%08x   SP=0x%08x   LR=0x%08x", 
          get_kvm_reg(ARMv8A_R12_usr), get_kvm_reg(ARMv8A_SP_usr), get_kvm_reg(ARMv8A_LR_usr));
    // state registers
    error("ARMv7-M CPU State:");
    error(" CurrentMode=%s", inner()->current_mode == ARMv7M_THREAD ? "THREAD" : "HANDLER");
    error(" N=%d Z=%d C=%d V=%d Q=%d GE=0x%01x IPSR=0x%08x, EPSR=0x%08x",
          (apsr() >> 31) & 0b1, (apsr() >> 30) & 0b1, (apsr() >> 29) & 0b1,
          (apsr() >> 28) & 0b1, (apsr() >> 27) & 0b1, (apsr() >> 16) & 0b1111,
          ipsr(), epsr());
    error(" CONTROL .nPRIV=%d .SPSEL=%d .FPCA=%d", 
          inner()->control & 0b1, (inner()->control >> 1) & 0b1, (inner()->control >> 2) & 0b1);
    error(" PRIMASK=0x%08x FAULTMASK=0x%08x BASEPRI=0x%08x", inner()->primask, inner()->faultmask, inner()->basepri);
    // fpu registers
    error("ARMv7-M FPU State:");
    for (int i = 0; i < 32; i+=4) {
        error(" S%02d=0x%08x S%02d=0x%08x S%02d=0x%08x S%02d=0x%08x",
              i, get_kvm_fp_reg(i), i+1, get_kvm_fp_reg(i+1),
              i+2, get_kvm_fp_reg(i+2), i+3, get_kvm_fp_reg(i+3)
        );
    }
    // emulate state
    uint32_t pstate = get_kvm_reg(ARMv8A_APSR);
    error("ARMv8-A Emulate State:");
    error(" PSTATE=0x%08x", pstate);
    error(" PSTATE .E=%d .A=%d .I=%d .F=%d .T=%d .M=0x%02x",
          (pstate >> 9) & 0b1, (pstate >> 8) & 0b1, (pstate >> 7) & 0b1,
          (pstate >> 6) & 0b1, (pstate >> 5) & 0b1, pstate & 0x1f);
    error("=============================================================");
}

//==============================================================================
// work around watchpoint

/* instruction type */
#define INSN_ARM            0
#define INSN_THUMB_16       1
#define INSN_THUMB_32       2

static csh handle;

#define REG(id, length) \
    ((uint##length##_t)arm_reg_to_kvm((id), &reg_len))

#define REG_SIZE(id) \
    (arm_reg_size((id)))

/// @brief translate capstone reg to kvm
/// @param reg capstone arm register id
/// @param reg_len length of the register
/// @return pointer to the register in qemu cpu state
uint64_t CPU::arm_reg_to_kvm(arm_reg reg, int *reg_len) {
    /* general arm registers */
    switch (reg) {
        case ARM_REG_R0:  *reg_len = 0x4; return get_kvm_reg(ARMv8A_R0);
        case ARM_REG_R1:  *reg_len = 0x4; return get_kvm_reg(ARMv8A_R1);
        case ARM_REG_R2:  *reg_len = 0x4; return get_kvm_reg(ARMv8A_R2);
        case ARM_REG_R3:  *reg_len = 0x4; return get_kvm_reg(ARMv8A_R3);
        case ARM_REG_R4:  *reg_len = 0x4; return get_kvm_reg(ARMv8A_R4);
        case ARM_REG_R5:  *reg_len = 0x4; return get_kvm_reg(ARMv8A_R5);
        case ARM_REG_R6:  *reg_len = 0x4; return get_kvm_reg(ARMv8A_R6);
        case ARM_REG_R7:  *reg_len = 0x4; return get_kvm_reg(ARMv8A_R7);
        case ARM_REG_R8:  *reg_len = 0x4; return get_kvm_reg(ARMv8A_R8_usr);
        case ARM_REG_R9:  *reg_len = 0x4; return get_kvm_reg(ARMv8A_R9_usr);
        case ARM_REG_R10: *reg_len = 0x4; return get_kvm_reg(ARMv8A_R10_usr);
        case ARM_REG_R11: *reg_len = 0x4; return get_kvm_reg(ARMv8A_R11_usr);
        case ARM_REG_R12: *reg_len = 0x4; return get_kvm_reg(ARMv8A_R12_usr);
        case ARM_REG_SP:  *reg_len = 0x4; return get_kvm_reg(ARMv8A_SP_usr);
        case ARM_REG_LR:  *reg_len = 0x4; return get_kvm_reg(ARMv8A_LR_usr);
        case ARM_REG_PC:  *reg_len = 0x4; return get_kvm_reg(ARMv8A_PC);
        default: break;
    }

    /* general float registers */
    if (ARM_REG_S0 <= (int)reg && (int)reg <= ARM_REG_S31) {
        *reg_len = 0x4;
        return get_kvm_fp_reg(reg - ARM_REG_S0);
    }

    return 0x0;
}

/// @brief get size of the arm register
/// @param reg capstone arm register id
/// @return length of the capstone arm register
int CPU::arm_reg_size(arm_reg reg) {
    int reg_len;
    arm_reg_to_kvm(reg, &reg_len);
    return reg_len;
}

/// @brief get the instruction type
/// @param opcode instruction opcode
/// @return  type of the instruction
int CPU::get_insn_type(uint32_t opcode) {
    uint32_t pstate = get_kvm_reg(ARMv8A_APSR);
    bool thumb = (pstate >> 5) & 0b1;

    if (thumb) {
        /* A Thumb instructionL opcode[15:11] == 0b11101, 0b11110, 0b11111 is 32bit */    
        if (((opcode >> 11) & 0b11111) == 0b11101 ||
            ((opcode >> 11) & 0b11111) == 0b11110 ||
            ((opcode >> 11) & 0b11111) == 0b11111) {
            return INSN_THUMB_32;
        } else {
            return INSN_THUMB_16;
        }
    } else {
        return INSN_ARM;
    }
}

/// @brief decode arm instruction 
/// @param insn capstone instruction info
/// @param access_len memory access length
/// @return start address of the memory access
uint64_t CPU::decode_arm_insn(cs_insn *insn, uint64_t *access_len) {
    cs_detail *detail = insn->detail;
    cs_arm *detail_arm = &detail->arm;
    int reg_cnt = 0, scale = 1, disp = 0, shift = 0, reg_len = 0;
    arm_reg reg_list[32] = {};
    arm_reg base_reg_id = ARM_REG_INVALID, index_reg_id = ARM_REG_INVALID;

    uint32_t access_addr = 0;
    *access_len = 0;

    /* dump instruction operands */
    for (int i = 0; i < detail_arm->op_count; i++) {
        cs_arm_op *arm_op = &detail_arm->operands[i];
        if (arm_op->type == ARM_OP_REG) {
            reg_list[reg_cnt++] = (arm_reg)arm_op->reg;
        }
        if (arm_op->type == ARM_OP_MEM) {
            base_reg_id = arm_op->mem.base;
            disp = arm_op->mem.disp;
            scale = arm_op->mem.scale;
            if (arm_op->shift.type == ARM_SFT_LSL) {
                shift = arm_op->shift.value;
            }
            if (arm_op->mem.index != ARM_REG_INVALID) {
                index_reg_id = arm_op->mem.index;
            }
        }
    }

    /* for instructions with memory operand */
    if (base_reg_id != ARM_REG_INVALID) {
        access_addr = REG(base_reg_id, 32);
        if (index_reg_id != ARM_REG_INVALID) {
            access_addr += (REG(index_reg_id, 32) << shift) * scale;
        }
        if (!insn->detail->arm.post_index) {
            access_addr += disp;
        }
        
        if (strcmp(insn->mnemonic, "str") == 0
            || strcmp(insn->mnemonic, "str.w") == 0
            || strcmp(insn->mnemonic, "ldr") == 0
            || strcmp(insn->mnemonic, "ldr.w") == 0
        ) {
            *access_len = 0x4;
            return access_addr;
        }
        if (strcmp(insn->mnemonic, "strb") == 0
            || strcmp(insn->mnemonic, "strb.w") == 0
            || strcmp(insn->mnemonic, "ldrb") == 0
            || strcmp(insn->mnemonic, "ldrb.w") == 0
        ) {
            *access_len = 0x1;
            return access_addr;
        }
        if (strcmp(insn->mnemonic, "strh") == 0
            || strcmp(insn->mnemonic, "strh.w") == 0
            || strcmp(insn->mnemonic, "ldrh") == 0
            || strcmp(insn->mnemonic, "ldrh.w") == 0
        ) {
            *access_len = 0x2;
            return access_addr;
        }
        if (strcmp(insn->mnemonic, "strd") == 0
            || strcmp(insn->mnemonic, "strd.w") == 0) {
            *access_len = 0x8;
            return access_addr;
        }
        
        return (uint64_t)-1;
    }

    /* push and pop instruction family */
    if (strcmp(insn->mnemonic, "push") == 0 || 
        strcmp(insn->mnemonic, "push.w") == 0 ||
        strcmp(insn->mnemonic, "vpush") == 0) {
        for (int i = reg_cnt-1; i >= 0; i--) {
            *access_len += REG_SIZE(reg_list[i]);
        }
        return get_kvm_reg(ARMv8A_SP_usr) - *access_len; /*lowest address: sp after push*/
    }
    if (strcmp(insn->mnemonic, "pop") == 0 ||
        strcmp(insn->mnemonic, "pop.w") == 0 ||
        strcmp(insn->mnemonic, "vpop") == 0) {
        for (int i = 0; i < reg_cnt; i++) {
            *access_len += REG_SIZE(reg_list[i]);
        }
        return get_kvm_reg(ARMv8A_SP_usr); /*lowest address: sp before pop*/ 
    }
    
    /* multistore and multiload instruction family */
    if (strcmp(insn->mnemonic, "stmdb") == 0 ||
        strcmp(insn->mnemonic, "vstmdb") == 0 ||
        strcmp(insn->mnemonic, "ldmdb") == 0 ||
        strcmp(insn->mnemonic, "vldmdb") == 0
    ) {
        for (int i = reg_cnt-1; i >= 1; i--) {
            *access_len += REG_SIZE(reg_list[i]);
        }
        return REG(reg_list[0], 32) - *access_len;
    }
    if (strcmp(insn->mnemonic, "stm") == 0 ||
        strcmp(insn->mnemonic, "vstm") == 0 ||
        strcmp(insn->mnemonic, "ldm") == 0 ||
        strcmp(insn->mnemonic, "vldm") == 0
    ) {
        for (int i = 1; i < reg_cnt; i++) {
            *access_len += REG_SIZE(reg_list[i]);
        }
        return REG(reg_list[0], 32);
    }

    return (uint64_t)-1;
}

/// @brief decode instruction to get the memory address
/// @param access_len memory access length
/// @return start address of the memory access
uint64_t CPU::decode_insn(uint64_t *access_len) {
    cs_arch engine_arch;
    cs_mode engine_mode;
    cs_insn *insn;
    
    /* get current pc and read the opcode */
    uint64_t pc = get_kvm_reg(ARMv8A_PC);
    uint32_t opcode = board()->read_u32(pc);

    /* decode instruction type */
    int insn_type = get_insn_type(opcode), insn_len;
    switch(insn_type) {
    case INSN_THUMB_16:
        insn_len = 0x2;
        engine_arch = CS_ARCH_ARM;
        engine_mode = CS_MODE_THUMB;
        break;
    case INSN_THUMB_32:
        insn_len = 0x4;
        engine_arch = CS_ARCH_ARM;
        engine_mode = CS_MODE_THUMB;
        break;
    default:
        insn_len = 0x4;
        engine_arch = CS_ARCH_ARM;
        engine_mode = CS_MODE_ARM;
        break;
    }

    /* start capstone engine */
    if (cs_open(engine_arch, engine_mode, &handle) != CS_ERR_OK) {
#if KVM_OPEN_DEBUG
        error("failed to open capstone engine");
#endif
        return -1;
    }
    cs_option(handle, CS_OPT_DETAIL, CS_OPT_ON);
    size_t count = cs_disasm(handle, (uint8_t *)&opcode, insn_len, pc, 0, &insn);
    if (count != 1) {
#if KVM_OPEN_DEBUG
        error("failed to disasm the opcode");
#endif
        if (count > 1) {
            cs_free(insn, count);
        }
        cs_close(&handle);
        return -1;
    }

    /* decode instruction */
    uint64_t ret = -1;
    ret = decode_arm_insn(insn, access_len);
    
#if KVM_OPEN_DEBUG
    info("memory access: insn=%s addr=0x%08lx len=0x%08lx", insn->mnemonic, ret, *access_len);
#endif
    cs_free(insn, count);
    cs_close(&handle);
    return ret;
}

}  // namespace khost
