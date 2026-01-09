#include "en_cpu_para.h"
#include "en_board_para.h"
#if KVM_OPEN_DEBUG
    #include "autogen_rt_para_debug.h"
#else
    #include "autogen_rt_para_normal.h"
#endif


namespace khost {

static clock_t clock_last;
static clock_t clock_current;

//==============================================================================
// construct and destruct operations

/// @brief constructor
/// @param board belonging board
CPUPara::CPUPara(BoardPara *board) : CPU(board) {
    _board_para = board;
    _inner = (McuState *)board->gpa_to_hva(RT_PARA_mcu_state);
}

/// @brief destructor
CPUPara::~CPUPara() {

}

/// @brief reset cpu state and registers 
/// @return true on success, false on fail
bool CPUPara::reset() {
    if (!CPU::reset()) {
        return false;
    }
    
    // set pc value (directly write)
    uint32_t entry_pc = RT_PARA_vector_table;
    if (!set_kvm_reg(ARMv8A_PC, entry_pc)) {
        error("failed to set pc");
        return false;
    }
    
    // clear basic block stack
    while (!_last_bb_stack.empty()) {
        _last_bb_stack.pop();
    }
    
    // clear mmio ranout mark
    _mmio_ran_out = false;

    // set registers and states 
    _inner->msp =  CONFIG.firmware_info()->initial_sp & 0xfffffffc;
    _inner->psp = 0;
    _inner->current_mode = ARMv7M_THREAD;
    _inner->ipsr = 0;
    set_epsr(0b1 << ARMv7M_PSR_T_OFFSET);
    _inner->primask = 0;
    _inner->faultmask = 0;
    _inner->basepri = 0;
    _inner->control = 0b100;          // enable fp extension
    return true;
}

//==============================================================================
// Execution

/// @brief handle KVM_RUN exit reason KVM_EXIT_MMIO
/// @return 
///  -1(HANDLED): reenter the guest
///   0(EXIT_OK): firmware request to exit or input is used up
///   2(EXIT_INTERNAL_CRASH): crash because of the engine
int CPUPara::handle_mmio_exit() {
    uint32_t gpa  = _kvm_run->mmio.phys_addr;
    uint8_t* data = _kvm_run->mmio.data;
    uint32_t len  = _kvm_run->mmio.len;

    // Access SPECIAL_MMIO_HLT
    if (gpa == SPECIAL_MMIO_HLT) {
#if KVM_OPEN_DEBUG
        info("guest request exit");
#endif
        return EXIT_OK;
    }

    // Access SPECIAL_MMIO_DEBUG_OUTPUT
    if (gpa == SPECIAL_MMIO_DEBUG_OUTPUT) {
#if KVM_OPEN_DEBUG
        print_mutex.lock();
        printf("%c", ((uint32_t *)data)[0]);
        fflush(stdout);
        print_mutex.unlock();
#endif
        return HANDLED;
    }

    // Simple MMIO Access
    auto addr_info = CONFIG.address_info(gpa);
    MMIORegion *handler = _board_para->find_mmio_handler(gpa);
    if (!handler || (addr_info.end == 0x0 && addr_info.start == 0x0)) {
        if (!inject_fault(BUS_FAULT_PRECISERR, gpa)) {
#if KVM_OPEN_DEBUG
            info("invalid mmio access with pc=0x%x", get_kvm_reg(ARMv8A_PC));
#endif
            _kvm_run->exit_reason = KVM_EXIT_UNKNOWN;
            return HANDLED;
        }
#if KVM_OPEN_DEBUG
        error("invalid mmio access with pc=0x%x", get_kvm_reg(ARMv8A_PC));
        error("    gpa:  0x%x", gpa);
        error("    len:  0x%x", len);
        dump_state();
#endif
        return EXIT_FIRMWARE_CRASH;
    }

    int ret = (
        _kvm_run->mmio.is_write ?
        handler->handle_write(gpa, data, len, get_kvm_reg(ARMv8A_PC)) :
        handler->handle_read(gpa, data, len, get_kvm_reg(ARMv8A_PC))
    );
    return ret;
}

/// @brief emulate irq handling of ARMv7-M
/// @return 
///  -1(HANDLED): reenter the guest
///   1(EXIT_FIRMWARE_CRASH): crash inside the firmware
///   2(EXIT_INTERNAL_CRASH): crash because of the engine
int CPUPara::handle_irq() {
    // checks pending exception number
    int pending_exc_number = _board_para->nvic()->get_pending_irq();
    if (pending_exc_number == 0) {
        // we directly return to the guest if there's no exception
        return HANDLED;
    }
    
    // check can_take_pending_exception again because primask, faultmask and 
    // basepri may be changed between IRQ injection and VM_EXIT 
    if (!_board_para->nvic()->can_take_pending_exception()) {
        // we can't take exception now, go back to normal state and leave the 
        // exception pending for future activation
        return HANDLED;
    }
    return handle_exception_enter(pending_exc_number);
}

#define ARM64_SYSREG_ESR_EL1 \
  (0x6030000000130000ULL | (3ULL << 14) | (0ULL << 11) | (5ULL << 7) | (2ULL << 3) | (0ULL << 0))
#define ARM64_SYSREG_FAR_EL1 \
  (0x6030000000130000ULL | (3ULL << 14) | (0ULL << 11) | (6ULL << 7) | (0ULL << 3) | (0ULL << 0))

/// @brief handle DEBUG exit with ARM BKPT instruction
/// @param arm_asm_code instruction code
/// @return 
///  -2(UNHANDLE): not handled
///  -1(HANDLED): reenter the guest
///   1(EXIT_FIRMWARE_CRASH): crash inside the firmware
///   2(EXIT_INTERNAL_CRASH): crash because of the engine
/// @note ARM BKPT instructions are used in ARMv8-A style exception vector
int CPUPara::handle_arm_bkpt_exit(uint32_t arm_asm_code) {
    switch (arm_asm_code)
    {
    // DEBUG because of UND handler enter 
    case BKPT_ARM_UND: {
#if KVM_OPEN_DEBUG
        error("Undefined Instruction");
#endif
        // lr is the address of the undefined instruction + 0x2
        uint32_t pc = get_kvm_reg(ARMv8A_LR_und) - 0x2;
        set_kvm_reg(ARMv8A_PC, pc);
        set_kvm_reg(ARMv8A_APSR, get_kvm_reg(ARMv8A_SPSR_und));

        if (!inject_fault(USAGE_FAULT_UNDEFINSTR, pc)) {
            return HANDLED;
        }
#if KVM_OPEN_DEBUG
        error("Instruction: 0x%08x", _board_para->read_u32(get_kvm_reg(ARMv8A_PC)));
        dump_state();
#endif
        return EXIT_FIRMWARE_CRASH;
    }
    // DEBUG because of SVC handler enter
    case BKPT_ARM_SVC: {
#if KVM_OPEN_DEBUG
        debug("handling SVC exception");
#endif
        // restore the state of the firmware
        // LR is the address of the instruction after the SVC instruction + 0x2
        bool ok = (
            set_kvm_reg(ARMv8A_PC, get_kvm_reg(ARMv8A_LR_svc))
            && set_kvm_reg(ARMv8A_APSR, get_kvm_reg(ARMv8A_SPSR_svc)) 
        );
        if (!ok) {
            return EXIT_INTERNAL_CRASH;
        }
        return handle_svc();
    }
    // DEBUG because of PREFETCH abort handler
    case BKPT_ARM_PREFETCH: {
        // LR is the address of aborted instruction fetch + 0x4
        // note: exc_return is odd number which will be treated as thumb 
        // instruction address, so we only -0x3 to get the real exc_return
        const uint32_t exc_return_mask = 0xffffffe0;
        uint32_t pc = get_kvm_reg(ARMv8A_LR_abt) - 0x3;
        if ((pc & exc_return_mask) == exc_return_mask) {
            return handle_exception_return(pc);
        } else {
#if KVM_OPEN_DEBUG
            error("Prefetch Abort");
#endif
            set_kvm_reg(ARMv8A_PC, get_kvm_reg(ARMv8A_LR_abt) - 0x4);
            set_kvm_reg(ARMv8A_APSR, get_kvm_reg(ARMv8A_SPSR_abt));
            
            // prefetch abort belongs to firmware
            auto info = CONFIG.address_info(pc);
            if (info.start == 0x0 && info.end == 0x0) {
                // PC in invalid area means bus fault
                if (!inject_fault(BUS_FAULT_IBUSERR, pc)) {
                    return HANDLED;
                }
            } else {
                // PC in valid area means memmanage fault
                if (!inject_fault(MEM_FAULT_IACCVIOL, pc)) {
                    return HANDLED;
                } 
            }
#if KVM_OPEN_DEBUG
            dump_state();
#endif
            return EXIT_FIRMWARE_CRASH;
        }
    }
    // DEBUG because of DATA abort handler 
    case BKPT_ARM_DATA: {
#if KVM_OPEN_DEBUG
        error("Data Abort");
#endif
        // LR is the address of instruction that generated the abort + 0x8
        set_kvm_reg(ARMv8A_PC, get_kvm_reg(ARMv8A_LR_abt) - 0x8);
        set_kvm_reg(ARMv8A_APSR, get_kvm_reg(ARMv8A_SPSR_abt));
        
        // report the ESR
        uint32_t dfsr;
        struct kvm_one_reg dfsr_reg = {
            .id = ARM64_SYSREG_ESR_EL1,
            .addr = (uint64_t)&dfsr,
        };
        ioctl(_vcpu_fd, KVM_GET_ONE_REG, &dfsr_reg);

        uint32_t dfar;
        struct kvm_one_reg dfar_reg = {
            .id = ARM64_SYSREG_FAR_EL1,
            .addr = (uint64_t)&dfar,
        };
        ioctl(_vcpu_fd, KVM_GET_ONE_REG, &dfar_reg);
        
        uint32_t fs = (((dfsr >> 10) & 0b1) << 4) | (dfsr & 0b1111);
        if (fs == 0b00001) {
            // alignment fault is translated to unaligned abort
            if (!inject_fault(USAGE_FAULT_UNALIGNED, 0)) {
                return HANDLED;
            }
        } else if (fs == 0b00100 || fs == 0b10110 || fs == 0b11000) {
            // asynchronous data abort and instruction cache maintenance abort are
            // translated to impreciserr abort
            if (!inject_fault(BUS_FAULT_IMPRECISERR, 0)) {
                return HANDLED;
            }
        } else {
            // synchronous data abort are translated to preciserr abort 
            if (!inject_fault(BUS_FAULT_PRECISERR, dfar)) {
                return HANDLED;
            }
        }

#if KVM_OPEN_DEBUG
        error("esr_val = 0x%08x", dfsr);
        dump_state();
#endif
        return EXIT_FIRMWARE_CRASH;
    }
    // DEBUG because of GIC injected irq exception
    case BKPT_ARM_IRQ: {
#if KVM_OPEN_DEBUG
        debug("handle IRQ exception");
#endif
        // LR is the address of next instruction to execute + 0x4
        bool ok = (
            set_kvm_reg(ARMv8A_PC, get_kvm_reg(ARMv8A_LR_irq) - 0x4)
            && set_kvm_reg(ARMv8A_APSR, get_kvm_reg(ARMv8A_SPSR_irq))
        );
        if (!(get_kvm_reg(ARMv8A_APSR) & COMPAT_PSR_T_BIT)) {
#if KVM_OPEN_DEBUG
            error("handling irq from arm mode, this should not happen");
#endif
            return EXIT_INTERNAL_CRASH;
        }
        if (!ok) {
            return EXIT_INTERNAL_CRASH;
        }
        return handle_irq();
    }
    // DEBUG because of runtime debug
    case BKPT_ARM_DEBUG: {
        set_kvm_reg(ARMv8A_PC, get_kvm_reg(ARMv8A_PC) + 0x4);
        char *output = _board_para->firmware_state()->debug_buffer;
        printf("%s", output);
        fflush(stdout);
        return HANDLED;
    }
    case BKPT_ARM_FIRMWARE_CRASH: {
        if (get_kvm_reg(ARMv8A_PC) == 0x0) {
#if KVM_OPEN_DEBUG
            error("potential null pointer derefernece happened");
            dump_state();
#endif
        }
        return EXIT_FIRMWARE_CRASH;
    }
    case BKPT_ARM_FIRMWARE_EXIT: {
#if KVM_OPEN_DEBUG
        info("guest exit from runtime");
#endif
        return EXIT_OK;
    }
    case BKPT_ARM_FIRMWARE_TIMEOUT: {
#if KVM_OPEN_DEBUG
        info("firmware timeout");
#endif
        return EXIT_OK;
    }
    case BKPT_ARM_FIRMWARE_SNAPSHOT: {
#if KVM_OPEN_DEBUG
        info("firmware acquire fuzzware model");
#endif
        // move the pc to next instruction
        set_kvm_reg(ARMv8A_PC, get_kvm_reg(ARMv8A_PC) + 0x4);
        return EXIT_MODEL;
    }
    case BKPT_ARM_INTERNAL_CRASH: {
        return EXIT_INTERNAL_CRASH;
    }
    default:
        return UNHANDLED;
    }
}

/// @brief emulate svc handling of ARMv7-M
/// @return always HANDLED
int CPUPara::handle_svc() {
    // svc has priority in ARMv7-M too, restore the context and wait for 
    // trigger from nvic
    _board_para->nvic()->svc_trigger(1);
    return HANDLED;
}

/// @brief handle DEBUG exit with THUMB BKPT instruction
/// @param pc pc of the firmware
/// @param thumb_asm_code opcode of the instruction
/// @return 
///  -2(UNHANDLE): not handled
///  -1(HANDLED): reenter the guest
///   0(EXIT_OK): firmware request to exit or input is used up
///   1(EXIT_FIRMWARE_CRASH): crash inside the firmware
///   2(EXIT_INTERNAL_CRASH): crash because of the engine
/// @note THUMB BKPT instructions are used in firmware code
int CPUPara::handle_thumb_bkpt_exit(uint32_t pc, uint32_t thumb_asm_code) {
    switch (thumb_asm_code) {
    // DEBUG because of incompatible instruction
    case BKPT_INCOMP_INSN: {
#if KVM_OPEN_DEBUG
        debug("handling incompatible insn at 0x%x", pc);
#endif
        return handle_incompatible_insn(pc);
    }
    // DEBUG because of breakpoint
    case BKPT_BREAKPOINT: {
#if KVM_OPEN_DEBUG
        info("handling breakpoint at 0x%x", pc);
#endif
        return handle_breakpoint();
    }
    // DEBUG because of coverage
    case BKPT_COVERAGE: {
#if KVM_OPEN_DEBUG
        debug("handling coverage at 0x%x", pc);
#endif

        if (pc == CONFIG.time_measure_begin_addr()) {
            clock_last = clock();
            _board->reset_coverage_bkpt(pc);
        }
        else if (pc == CONFIG.time_measure_end_addr()) {
            clock_current = clock();
// #if KVM_OPEN_DEBUG
            print_mutex.lock();
            printf("INFO  | khost: passing time: %f ms\n", 
                    (clock_current - clock_last) / 1000.0);
            printf("INFO  | khost: pc = %08x, reg2 = %08x, reg3 = %08x \n", pc, get_kvm_reg(ARMv8A_R2), get_kvm_reg(ARMv8A_R3));
            fflush(stdout);
            print_mutex.unlock();
// #endif
            return EXIT_OK;
        } else {
            _board->set_coverage_map_with_pc(pc);
        }
        return HANDLED;
    }
    // DEBUG because of hook point for infinitive loop
    case BKPT_TIMEOUT: {
#if KVM_OPEN_DEBUG
        info("handling hook for infinitive loop, pc=0x%08x", pc);
#endif
        return EXIT_OK;
    }
    case BKPT_THUMB_FIRMWARE_EXIT: {
#if KVM_OPEN_DEBUG
        uint32_t blocks = _board->firmware_state()->block_count;
        uint32_t max_blocks = _board->firmware_state()->block_limit;
        info("guest exit from firmware after %d/%d basic blocks", (int)blocks, (int)max_blocks);
        dump_state();
#endif
        return EXIT_OK;
    }
    // DEBUG beacuse of time measuring
    case BKPT_MEASURE_TIME: {
//#if KVM_OPEN_DEBUG
        clock_current = clock();
        print_mutex.lock();
        printf("INFO  | khost: passing time: %f ms\n", 
                (clock_current - clock_last) / 1000.0);
        fflush(stdout);
        print_mutex.unlock();
//#endif
        clock_last = clock_current;
        return (
            set_kvm_reg(ARMv8A_PC, get_kvm_reg(ARMv8A_PC) + 0x2) ?
            HANDLED : EXIT_INTERNAL_CRASH
        );
    }
    case BKPT_EMU_TIMER: {
        // skip current bkpt instruction
        set_kvm_reg(ARMv8A_PC, get_kvm_reg(ARMv8A_PC) + 0x2);
        return _board_para->systick()->handle_tick();
    }
    default:
        return UNHANDLED;
    }
}

/// @brief handle debug breakpoint in firmware
/// @return 
///  -1(HANDLED): reenter the guest
///   2(EXIT_INTERNAL_CRASH): crash because of the engine
int CPUPara::handle_breakpoint() {
    return (
        set_kvm_reg(ARMv8A_PC, get_kvm_reg(ARMv8A_PC) + 0x2) ?
        HANDLED : EXIT_INTERNAL_CRASH
    );
}

/// @brief emulate mrs/msr/cps instruction of ARMv7-M
/// @param pc instruction address
/// @return 
///  -1(HANDLED): reenter the guest
///   2(EXIT_INTERNAL_CRASH): crash because of the engine
int CPUPara::handle_incompatible_insn(uint32_t pc) {
    IncompatibleInsn *insn = CONFIG.get_incompatible_insn(pc);
    switch (insn->type)
    {
    // mrs instruction
    case ARMv7M_MRS: {
        int ret = handle_mrs(insn);
        if (ret != HANDLED) {
            return ret;
        }
        // MRS is 32bit instruction, set lr to next instruction (thumb)
        if (!set_kvm_reg(ARMv8A_PC, pc + 0x4)) {
           return EXIT_INTERNAL_CRASH;
        }
        return HANDLED;
    }
    // msr instruction
    case ARMv7M_MSR: {
        int ret = handle_msr(insn);
        if (ret != HANDLED) {
            return ret;
        }
        // MSR is 32bit instruction, set lr to next instruction (thumb)
        if (!set_kvm_reg(ARMv8A_PC, pc + 0x4)) {
            return EXIT_INTERNAL_CRASH;
        }
        return HANDLED;
    }
    // cps instruction
    case ARMv7M_CPS: {
        int ret = handle_cps(insn);
        if (ret != HANDLED) {
            return ret;
        }
        // CPS is 16bit instruction, set lr to next instruction (thumb)
        if (!set_kvm_reg(ARMv8A_PC, pc + 0x2)) {
           return EXIT_INTERNAL_CRASH;
        }
        return HANDLED;
    }
    default:
        return EXIT_INTERNAL_CRASH;
    }
}

/// @brief emulate mrs instruction of ARMv7-M
/// @param insn incompatible instruction information
/// @return 
///  -1(HANDLED): reenter the guest
///   2(EXIT_INTERNAL_CRASH): crash because of the engine
int CPUPara::handle_mrs(IncompatibleInsn *insn) {
    uint32_t r_id = insn->rd;
    uint32_t sys_id = insn->sysm;
    // MRS to R13 and R15 is UNPREDICTABLE
    if (r_id == 13 || r_id == 15) {
#if KVM_OPEN_DEBUG
        error("unpredictable mrs instruction");
#endif
        return EXIT_FIRMWARE_CRASH;
    }

    // emulate the operation of the mrs
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
            rd |= (ipsr() & 0b111111111);
        } else if ((sys_id & 0b100) == 0) {
            rd |= (apsr() & (0b11111 << 27));
            rd |= (apsr() & (0b1111 << 16));
        }
        break;
    case ARMv7M_MSP:
        rd = msp();
        break;
    case ARMv7M_PSP:
        rd = psp();
        break;
    case ARMv7M_PRIMASK:
        rd = (primask() & 0b1);
        break;
    case ARMv7M_BASERPI:
    case ARMv7M_BASERPI_MAX:
        rd = (basepri() & 0b11111111);
        break;
    case ARMv7M_FAULTMASK:
        rd = (faultmask() & 0b1);
        break;
    case ARMv7M_CONTROL:
        rd = (control() & 0b111);
        break;
    default:
        break;
    }
    return (
        set_kvm_reg(general_reg_id(r_id), rd) ? HANDLED : EXIT_INTERNAL_CRASH
    );
}

/// @brief emulate msr instruction of ARMv7-M
/// @param insn incompatible instruction information
/// @return 
///  -1(HANDLED): reenter the guest
///   2(EXIT_INTERNAL_CRASH): crash because of the engine
int CPUPara::handle_msr(IncompatibleInsn *insn) {
    uint32_t r_id = insn->rn;
    uint32_t sys_id = insn->sysm;
    // MRS to R13 and R15 is UNPREDICTABLE
    if (r_id == 13 || r_id == 15) {
#if KVM_OPEN_DEBUG
        error("unpredictable msr instruction");
#endif
        return EXIT_FIRMWARE_CRASH;
    }

    // emulate the operation of the msr
    // ref: https://developer.arm.com/documentation/ddi0403/d/System-Level-Architecture/System-Instruction-Details/ARMv7-M-system-instruction-descriptions/MSR?lang=en
    uint32_t rn = get_kvm_reg(general_reg_id(r_id));
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
            if (!set_apsr(apsr)) {
                return EXIT_INTERNAL_CRASH;
            }
        }
        break;
    case ARMv7M_MSP:
        if (!set_msp(rn)) {
            return EXIT_INTERNAL_CRASH;
        }
        break;
    case ARMv7M_PSP:
        if (!set_psp(rn)) {
            return EXIT_INTERNAL_CRASH;
        }
        break;
    case ARMv7M_PRIMASK:
        set_primask(rn & 0b1);
        break;
    case ARMv7M_BASERPI:
        set_basepri(rn & 0b11111111);
        break;
    case ARMv7M_BASERPI_MAX:
        if (((rn & 0b11111111) != 0) &&
            ((rn & 0b11111111) < basepri() || basepri() == 0)) {
            set_basepri(rn & 0b11111111);
        }
        break;
    case ARMv7M_FAULTMASK:
        set_faultmask(rn & 0b1);
        break;
    case ARMv7M_CONTROL:
        set_control(rn & 0b111);
        break;
    default:
        break;
    }
    return HANDLED;
}

/// @brief emulate cps instruction of ARMv7-M
/// @param insn incompatible instruction information
/// @return 
///  -1(HANDLED): reenter the guest
///   0(EXIT_OK): firmware request to exit or input is used up
///   1(EXIT_FIRMWARE_CRASH): crash inside the firmware
///   2(EXIT_INTERNAL_CRASH): crash because of the engine
/// @note https://developer.arm.com/documentation/ddi0403/d/System-Level-Architecture/System-Instruction-Details/ARMv7-M-system-instruction-descriptions/CPS?lang=en
int CPUPara::handle_cps(IncompatibleInsn *insn) {
    bool enable = (insn->im == 0);
    bool affect_pri = (insn->i == 1);
    bool affect_fault = (insn->f == 1);

    // change primask
    if (affect_pri) {
        set_primask(enable ? 0 : 1);
    }
    // change faultmask
    if (affect_fault) {
        set_faultmask(enable ? 0 : 1);
    }

    return HANDLED;
}

/// @brief emulate exception enter behavior of ARMv7-M
/// @param exc_number exception number
/// @return 
///  -1(HANDLED): reenter the guest
///   1(EXIT_FIRMWARE_CRASH): crash inside the firmware
///   2(EXIT_INTERNAL_CRASH): crash because of the engine
/// @note modifies special registers directly (without using helper functions)
///       and maintains the system state directly
/// @note https://developer.arm.com/documentation/ddi0403/d/System-Level-Architecture/System-Level-Programmers--Model/ARMv7-M-exception-model/Exception-entry-behavior?lang=en
int CPUPara::handle_exception_enter(uint32_t exc_number) {
#if KVM_OPEN_DEBUG
    info("handling exception enter: exc_number=%d, from=0x%08x", 
         exc_number, get_kvm_reg(ARMv8A_PC));
#endif

    // STEP0: Update current state =============================================
    uint32_t return_addr = get_kvm_reg(ARMv8A_PC);
    uint32_t pstate = get_kvm_reg(ARMv8A_APSR);
    uint32_t cur_psp = psp();   // update psp
    uint32_t cur_msp = msp();   // update msp
    uint32_t new_psp = cur_psp;
    uint32_t new_msp = cur_msp;
    uint32_t apsr_mask = (ARMv7M_PSR_N_MASK | ARMv7M_PSR_Z_MASK | 
                          ARMv7M_PSR_C_MASK | ARMv7M_PSR_V_MASK | 
                          ARMv7M_PSR_Q_MASK | ARMv7M_PSR_GE_MASK);
    uint32_t apsr = pstate & apsr_mask;

    // STEP1: PushStack ========================================================
    // calculate the frame size and align
    uint32_t frame_size = 0;
    uint32_t force_align = 0;
    uint32_t control_fpca = (_inner->control & ARMv7M_CONTROL_FPCA_MASK) 
                            >> ARMv7M_CONTROL_FPCA_OFFSET;
    if (control_fpca) {
        frame_size = 0x68;
        force_align = 1;
    } else {
        frame_size = 0x20;
        // equal to force_align = CCR.STKALIGN
        force_align = (_board_para->sysctl()->get(SC_REG_CCR) & 0b1000000000) >> 9;
    }

    // calculate the frame pointer
    uint32_t spmask = ~(force_align << 2);
    uint32_t control_spsel = (_inner->control & ARMv7M_CONTROL_SPSEL_MASK) 
                             >> ARMv7M_CONTROL_SPSEL_OFFSET;
    uint32_t frame_ptr_align = 0;
    uint32_t frame_ptr;
    if (control_spsel == 1 && _inner->current_mode == ARMv7M_THREAD) {
        frame_ptr_align = ((cur_psp & 0b100) >> 2) & force_align;
        new_psp = (cur_psp - frame_size) & spmask;
        frame_ptr = new_psp;
    } else {
        frame_ptr_align = ((cur_msp & 0b100) >> 2) & force_align;
        new_msp = (cur_msp - frame_size) & spmask;
        frame_ptr = new_msp;
    }

    // check frame ptr
    if (frame_ptr < SRAM_START || frame_ptr > SRAM_END) {
#if KVM_OPEN_DEBUG
        error("sp incorrect when exception enter");
#endif
        return EXIT_FIRMWARE_CRASH;
    }
    if (frame_ptr+frame_size < SRAM_START || frame_ptr+frame_size > SRAM_END) {
#if KVM_OPEN_DEBUG
        error("sp incorrect when exception enter");
#endif
        return EXIT_FIRMWARE_CRASH;
    }

    // store the regular context
    // note: only the stack locations, not the store order, are architected  
    _board_para->write_u32(frame_ptr, get_kvm_reg(ARMv8A_R0));
    _board_para->write_u32(frame_ptr+0x4, get_kvm_reg(ARMv8A_R1));
    _board_para->write_u32(frame_ptr+0x8, get_kvm_reg(ARMv8A_R2));
    _board_para->write_u32(frame_ptr+0xC, get_kvm_reg(ARMv8A_R3));
    _board_para->write_u32(frame_ptr+0x10, get_kvm_reg(ARMv8A_R12_usr));
    _board_para->write_u32(frame_ptr+0x14, get_kvm_reg(ARMv8A_LR_usr));
    _board_para->write_u32(frame_ptr+0x18, return_addr);
    uint32_t xpsr = (apsr | ipsr() | epsr());
    uint32_t saved_xpsr = 0;
    saved_xpsr |= (xpsr & 0b11111111111111111111110000000000);
    saved_xpsr |= (frame_ptr_align << 9);
    saved_xpsr |= (xpsr & 0b111111111);
    _board_para->write_u32(frame_ptr+0x1c, saved_xpsr);

    // store the FP context
    // note: we don't use lazy save stack
    if (control_fpca == 1) {
        // save Sx
        for (int i = 0; i < 16; i++) {
            _board_para->write_u32(frame_ptr+0x20+(4*i), get_kvm_fp_reg(i));
        }
        // save fpscr
        _board_para->write_u32(frame_ptr+0x60, get_kvm_reg(ARMv8A_FPSCR));
    }

    // calculate the lr
    uint32_t lr = 0b11111111111111111111111111100000;
    if (_inner->current_mode == ARMv7M_HANDLER) {
        lr |= (!control_fpca) << 4;
        lr |= 0b0001;
    } else {
        lr |= (!control_fpca) << 4;
        lr |= 0b1001;
        lr |= (control_spsel) << 2;
    }

    // STEP2: ExceptionTaken ===================================================
    uint32_t vector_table_addr = _board_para->sysctl()->get(SC_REG_VTOR);
    uint32_t tmp = _board_para->read_u32(vector_table_addr + exc_number * 0x4);
    uint32_t pc = tmp & 0xFFFFFFFE;

    _inner->current_mode = ARMv7M_HANDLER;
    // APSR is unknown, leave it unmodified
    _inner->ipsr = exc_number;                         // exception number set to IPSR
    // PRIMASK, FAULTMASK, BASEPRI unchanged on exception entry
    _inner->control &= (~0b110);                       // CONTROL.nPRIV unchanged
    _inner->control |= (0b100);                        // current Stack is Main, Floating-point extension active

    // STEP3: ModeChange =======================================================
    // Change ARMv8-A CPU mode to System mode, thumb
    bool ok = (
        set_kvm_reg(ARMv8A_APSR, COMPAT_PSR_MODE_SYS | COMPAT_PSR_T_BIT)
        && set_kvm_reg(ARMv8A_LR_usr, lr)
        && set_kvm_reg(ARMv8A_PC, pc)
        && set_kvm_reg(ARMv8A_SP_usr, new_msp)
    );
    if (!ok) {
        return EXIT_INTERNAL_CRASH;
    }
    _inner->msp = new_msp;
    _inner->psp = new_psp;
    
    // STEP4: ACK IRQ ==========================================================
    // nvic side acknowledge
    _board_para->nvic()->acknowledge_irq();

    _last_bb_stack.push(_board_para->firmware_state()->block_last_id);
    _board_para->firmware_state()->block_last_id = 0;
    _board_para->flush_firmware_state();

    return HANDLED;
}

/// @brief emulate exception exit behavior of ARMv7-M
/// @param exc_return EXC_RETURN value
/// @return 
///  -1(HANDLED): reenter the guest
///   1(EXIT_FIRMWARE_CRASH): crash inside the firmware
///   2(EXIT_INTERNAL_CRASH): crash because of the engine
/// @note modifies special registers directly (without using helper functions)
///       and maintains the system state directly
/// @note https://developer.arm.com/documentation/ddi0403/d/System-Level-Architecture/System-Level-Programmers--Model/ARMv7-M-exception-model/Exception-return-behavior?lang=en
int CPUPara::handle_exception_return(uint32_t exc_return) {
    // check the current mode
    if (_inner->current_mode != ARMv7M_HANDLER) {
#if KVM_OPEN_DEBUG
        error("exception return from thread mode");
#endif
        return EXIT_FIRMWARE_CRASH;
    }

    // STEP0: update current state =============================================
    uint32_t cur_msp = msp();
    uint32_t cur_psp = psp();
    uint32_t new_msp = cur_msp;
    uint32_t new_psp = cur_psp;
    uint32_t returning_exception_number = _inner->ipsr;
    uint32_t frame_ptr = 0;
    switch (exc_return & 0b1111)
    {
    // return to Handler
    case 0b0001:
        frame_ptr = cur_msp;
        _inner->current_mode = ARMv7M_HANDLER;
        _inner->control &= (~ARMv7M_CONTROL_SPSEL_MASK);   // CONTROL.SPSEL = 0
        break;
    // returning to Thread using Main stack
    case 0b1001:
        frame_ptr = cur_msp;
        _inner->current_mode = ARMv7M_THREAD;
        _inner->control &= (~ARMv7M_CONTROL_SPSEL_MASK);   // CONTROL.SPSEL = 0
        break;
    // returning to Thread using Process stack
    case 0b1101:
        frame_ptr = cur_psp;
        _inner->current_mode = ARMv7M_THREAD;
        _inner->control &= (~ARMv7M_CONTROL_SPSEL_MASK);
        _inner->control |= 0b10;                           // CONTROL.SPSEL = 1
        break;
    default:
#if KVM_OPEN_DEBUG
        error("illegal EXC_RETURN");
#endif
        return false;
    }
    if (ipsr() != 0b000000010) {
        _inner->faultmask = 0;
    }

    // STEP1: pop stack ========================================================
    uint32_t frame_size = 0;
    uint32_t force_align = 0;
    if (((exc_return & 0b10000) >> 4) == 0) {
        frame_size = 0x68;
        force_align = 1;
    } else {
        frame_size = 0x20;
        // equal to force_align = CCR.STKALIGN
        force_align = (_board_para->sysctl()->get(SC_REG_CCR) & 0b1000000000) >> 9;
    }

    // check frame ptr
    if (frame_ptr < SRAM_START || frame_ptr > SRAM_END) {
#if KVM_OPEN_DEBUG
        error("sp incorrect when exception return");
#endif
        return EXIT_FIRMWARE_CRASH;
    }
    if (frame_ptr+frame_size < SRAM_START || frame_ptr+frame_size > SRAM_END) {
#if KVM_OPEN_DEBUG
        error("sp incorrect when exception return");
#endif
        return EXIT_FIRMWARE_CRASH;
    }

    bool ok = (
        set_kvm_reg(ARMv8A_R0, _board_para->read_u32(frame_ptr))
        && set_kvm_reg(ARMv8A_R1, _board_para->read_u32(frame_ptr+0x4))
        && set_kvm_reg(ARMv8A_R2, _board_para->read_u32(frame_ptr+0x8))
        && set_kvm_reg(ARMv8A_R3, _board_para->read_u32(frame_ptr+0xc))
        && set_kvm_reg(ARMv8A_R12_usr, _board_para->read_u32(frame_ptr+0x10))
        && set_kvm_reg(ARMv8A_LR_usr, _board_para->read_u32(frame_ptr+0x14))
        && set_kvm_reg(ARMv8A_PC, _board_para->read_u32(frame_ptr+0x18))
    );
    if (!ok) {
        return EXIT_INTERNAL_CRASH;
    }
    uint32_t psr = _board_para->read_u32(frame_ptr+0x1c);
#if KVM_OPEN_DEBUG
    info("handling exception return: exc_return=0x%08x, ret=0x%08x", 
            exc_return, _board->read_u32(frame_ptr+0x18));
#endif

    // pop float point state
    // note: we don't use lazy save stack
    if (((exc_return & 0b10000) >> 4) == 0) {
        for (int i = 0; i < 16; i++) {
            if (!set_kvm_fp_reg(i, _board->read_u32(frame_ptr+0x20+(4*i)))) {
                return EXIT_INTERNAL_CRASH;
            }
        }
        if (!set_kvm_reg(ARMv8A_FPSCR, _board->read_u32(frame_ptr+0x60))) {
            return EXIT_INTERNAL_CRASH;
        }
    }
    _inner->control &= (~0b100);
    _inner->control |= (!((exc_return & 0b10000) >> 4)) << 2;

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

    // STEP2: update state =====================================================
    // uint32_t apsr_mask = (ARMv7M_PSR_N_MASK | ARMv7M_PSR_Z_MASK | 
    //                       ARMv7M_PSR_C_MASK | ARMv7M_PSR_V_MASK | 
    //                       ARMv7M_PSR_Q_MASK | ARMv7M_PSR_GE_MASK);
    uint32_t pstate = 0; //get_kvm_reg(ARMv8A_SPSR_abt) & (~apsr_mask);
    pstate |= psr & (0b1111111 << 25);
    pstate |= psr & (0b1111111111 << 10);
    pstate |= COMPAT_PSR_T_BIT;

    uint32_t control_priv = (_inner->control & ARMv7M_CONTROL_PRIV_MASK) 
                            >> ARMv7M_CONTROL_PRIV_OFFSET;
    if (_inner->current_mode == ARMv7M_THREAD) {
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
        if (!set_kvm_reg(ARMv8A_APSR, pstate)) {
            return EXIT_INTERNAL_CRASH;
        }
    } else {
        // returning to handler mode, always use Sys Mode (EL1) thumb
        if (!set_kvm_reg(ARMv8A_APSR, pstate | COMPAT_PSR_MODE_SYS)) {
            return EXIT_INTERNAL_CRASH;
        }
    }

    _inner->ipsr = psr & 0b111111111;
    _inner->psp = new_psp;
    _inner->msp = new_msp;
    if (sp_select() == ARMv7M_PSP) {
        if (!set_kvm_reg(ARMv8A_SP_usr, _inner->psp)) {
            return EXIT_INTERNAL_CRASH;
        }
    } else {
        if (!set_kvm_reg(ARMv8A_SP_usr, _inner->msp)) {
            return EXIT_INTERNAL_CRASH;
        }
    }

    // STEP3: deactive exception ===============================================
    // nvic complete irq
    _board_para->nvic()->complete_irq(returning_exception_number);

    _board->firmware_state()->block_last_id = _last_bb_stack.top();
    _board->flush_firmware_state();
    _last_bb_stack.pop();

    return HANDLED;
}

//==============================================================================
// Register Operations

/// @brief chooses the correct SP within MSP and PSP according to cpu mode and spsel
/// @return ARMv7M_PSP or ARMv7M_MSP
int CPUPara::sp_select() {
    uint32_t control_spsel = (_inner->control & ARMv7M_CONTROL_SPSEL_MASK) 
                             >> ARMv7M_CONTROL_SPSEL_OFFSET;
    if (control_spsel == 1) {
        if (_inner->current_mode == ARMv7M_HANDLER) {
            // spsel should always be 0 under handler mode 
#if KVM_OPEN_DEBUG
            error("invalid spsel under handler mode");
#endif
            return ARMv7M_MSP;
        }
        // spsel = 1 and mode = thread: use psp
        return ARMv7M_PSP;
    }
    // spsel = 0 and mode = thread: use msp
    // mode = handler: use msp
    return ARMv7M_MSP;
}

/// @brief gets current msp, updates _msp member
/// @return value of msp
uint32_t CPUPara::msp() {
    // if we are using MSP as SP now, update the MSP
    if (sp_select() == ARMv7M_MSP) {
        _inner->msp = get_kvm_reg(ARMv8A_SP_usr);
    }
    return _inner->msp;
}

/// @brief sets current msp, maintains current SP_usr if needed
/// @param value new msp
/// @return true on success, false on fail
bool CPUPara::set_msp(uint32_t value) {
    _inner->msp = value;
    // if we are using MSP as SP now, update the SP_usr
    if (sp_select() == ARMv7M_MSP) {
        return set_kvm_reg(ARMv8A_SP_usr, _inner->msp);
    }
    return true;
}

/// @brief gets current psp, updates _psp member
/// @return value of psp
uint32_t CPUPara::psp() {
    // if we are using PSP as SP now, update the PSP
    if (sp_select() == ARMv7M_PSP) {
        _inner->psp = get_kvm_reg(ARMv8A_SP_usr);
    }
    return _inner->psp;
}

/// @brief sets current psp, maintains current SP_usr if needed
/// @param value new psp
/// @return true on success, false on fail
bool CPUPara::set_psp(uint32_t value) {
    _inner->psp = value;
    // if we are using PSP as SP now, update the SP_usr
    if (sp_select() == ARMv7M_PSP) {
        return set_kvm_reg(ARMv8A_SP_usr, _inner->psp);
    }
    return true;
}

/// @brief sets current apsr, bits only used in ARMv8-A profile is unmodified
/// @param value new value of the APSR
/// @return true on success, false on fail
bool CPUPara::set_apsr(uint32_t value) {
    // ARMv7M APSR is part of ARMv8A pstate
    uint32_t pstate = get_kvm_reg(ARMv7M_APSR);
    uint32_t apsr_mask = (ARMv7M_PSR_N_MASK | ARMv7M_PSR_Z_MASK | 
                          ARMv7M_PSR_C_MASK | ARMv7M_PSR_V_MASK | 
                          ARMv7M_PSR_Q_MASK | ARMv7M_PSR_GE_MASK);
    pstate = pstate & (~apsr_mask);
    pstate = pstate | (value & apsr_mask);
    return set_kvm_reg(ARMv7M_APSR, pstate);
}

/// @brief sets current epsr
/// @param value new value of the epsr
/// @return true on success, false on fail
bool CPUPara::set_epsr(uint32_t value) {
    uint32_t pstate = get_kvm_reg(ARMv7M_APSR);
    pstate = pstate & (~ARMv7M_PSR_ICIIT_MASK);
    pstate = pstate | (value & ARMv7M_PSR_ICIIT_MASK);
    return set_kvm_reg(ARMv7M_APSR, pstate);
}

/// @brief set value of ipsr
/// @param value new value of ipsr
void CPUPara::set_ipsr(uint32_t value) { 
    _inner->ipsr = value;
}

/// @brief gets current control
/// @return value of the control
uint32_t CPUPara::control() { 
    return _inner->control; 
}

/// @brief sets current control, changes the cpu state accoring to priv and spsel bits
/// @param value new value of the CONTROL
/// @return true on success, false on fail
bool CPUPara::set_control(uint32_t value) {
    // priv defines the execution privilege
    uint32_t control_priv = (value & ARMv7M_CONTROL_PRIV_MASK) 
                            >> ARMv7M_CONTROL_PRIV_OFFSET;
    if (_inner->current_mode == ARMv7M_THREAD) {
        uint32_t apsr = get_kvm_reg(ARMv8A_APSR) & (~0b11111);
        if (control_priv == 0) {
            // thread mode has privileged access, use ARMv8A AArch32 Sys Mode (EL1)
            apsr |= COMPAT_PSR_MODE_SYS;
        } else {
            // thread mode has unprivileged access, use ARMv8A AArch32 User Mode (EL0)
            apsr |= COMPAT_PSR_MODE_USR;
        }
        if (!set_kvm_reg(ARMv8A_APSR, apsr)) {
            return false;
        }
    }

    // spsel defines the stack to be used
    uint32_t control_spsel = (value & ARMv7M_CONTROL_SPSEL_MASK) 
                             >> ARMv7M_CONTROL_SPSEL_OFFSET;
    if (_inner->current_mode == ARMv7M_THREAD) {
        // save current sp
        uint32_t current_spsel = sp_select();
        uint32_t current_sp = get_kvm_reg(ARMv8A_SP_usr);
        if (current_spsel == ARMv7M_PSP) {
            _inner->psp = current_sp;
        } else {
            _inner->msp = current_sp;
        }
        // change sp according to spsel
        if (control_spsel == 0) {
            // use sp_main as the current stack
            if (!set_kvm_reg(ARMv8A_SP_usr, _inner->msp)) {
                return false;
            }
        } else {
            // use sp_process as the current stack
            if (!set_kvm_reg(ARMv8A_SP_usr, _inner->psp)) {
                return false;
            }
        }
    }

    // fpca defines whether the fp is active 
    // note: we always enable fp in AArch32 mode, this is only used by firmware
    // note: we always set fpca when apsen is enabled
    uint32_t control_fpca = (value & ARMv7M_CONTROL_FPCA_MASK) 
                            >> ARMv7M_CONTROL_FPCA_OFFSET;
    control_fpca |= (_board_para->sysctl()->get(SC_REG_FPCCR) >> 31);

    // update the CONTROL register
    _inner->control = 0;
    _inner->control |= (control_priv) << ARMv7M_CONTROL_PRIV_OFFSET;
    _inner->control |= (control_spsel) << ARMv7M_CONTROL_SPSEL_OFFSET;
    _inner->control |= (control_fpca) << ARMv7M_CONTROL_FPCA_OFFSET;

    return true;
}

/// @brief gets current value of primask
/// @return value of the primask
uint32_t CPUPara::primask() { 
    return _inner->primask; 
}

/// @brief set value of primask and update state of nvic
/// @param value new value of primask
void CPUPara::set_primask(uint32_t value) { 
    _inner->primask = value; 
    _board_para->nvic()->irq_update();
}

/// @brief gets current faultmask
/// @return value of the faultmask
uint32_t CPUPara::faultmask() { 
    return _inner->faultmask; 
}

/// @brief set value of faultmask and update state of nvic 
/// @param value new value of faultmask
void CPUPara::set_faultmask(uint32_t value) { 
    _inner->faultmask = value; 
    _board_para->nvic()->irq_update();
}

/// @brief gets current basepri
/// @return value of the basepri
uint32_t CPUPara::basepri() { 
    return _inner->basepri; 
}

/// @brief set value of baserpi and update state of nvic 
/// @param value new value of baserpi
void CPUPara::set_basepri(uint32_t value) { 
    _inner->basepri = value; 
    _board_para->nvic()->irq_update();
}

/// @brief set a bit in the value
/// @param value base value
/// @param bit 1 or 0
/// @param id bit id
/// @return result value
uint32_t CPUPara::set_bit(uint32_t value, int bit, int id) {
    uint32_t t = value & (~(1 << id));
    return t | ((bit & 0b1) << id);
}

/// @brief check if we can take the exception
/// @param id id of the exception
/// @return 1 on succ
int CPUPara::can_take_exception(uint32_t id) {
    uint32_t vtor = _board_para->sysctl()->get(SC_REG_VTOR);

    uint32_t isr_addr = _board->read_u32(vtor + id * 4) & 0xfffffffe;
    if (isr_addr == 0) {
        // we never trigger interrupt with no isr
        return 0;
    }
    if (_board->read_u32(isr_addr) == 0xe7fe) {
        // we never trigger interrupt with infinitve loop
        return 0;
    }
    if (_board->read_u32(isr_addr) == 0xbe03) {
        // we never trigger exit isr
        return 0;
    }
    return 1;
}

/// @brief take fault exception and do the escalation when need
/// @param id exception id
/// @return 1 on succ, 0 on fail
int CPUPara::take_exception(uint32_t id) {
    uint32_t target_exception = id;

    // if we have already crashed, directly exit to the host
    uint32_t excp_id = ipsr();
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
        if (_board_para->nvic()->irq_info(id)->active == 0) {
            target_exception = ARMv7M_EXCP_HARD;
        }
        // case 3: priority unmatch
        if (_board_para->nvic()->irq_info(id)->prio >= _board_para->nvic()->exec_prio()) {
            target_exception = ARMv7M_EXCP_HARD;
        }
    }

    // do the escalation
    if (target_exception != id) {
#if KVM_OPEN_DEBUG
        info("escalate to HARD FAULT");
#endif
        uint32_t fsr = _board_para->sysctl()->get(SC_REG_HFSR);
        _board_para->sysctl()->set(SC_REG_HFSR, set_bit(fsr, 1, HFSR_FORCED));
    }
    if (!can_take_exception(target_exception)) {
        return 1;
    }
    _board_para->nvic()->set_pending(target_exception);
    return 0;
}

/// @brief inject hard | memmange | usage fault back to firmware
/// @param type fault type
/// @param fpa fault physical address
/// @return 0 for succ, 1 on fail
int CPUPara::inject_fault(int type, uint32_t fpa) {
    uint32_t fsr;
    switch (type)
    {
    case HARD_FAULT_VECTTBL:
#if KVM_OPEN_DEBUG
        info("HardFault on vector read error");
#endif
        fsr = _board_para->sysctl()->get(SC_REG_HFSR);
        _board_para->sysctl()->set(SC_REG_HFSR, set_bit(fsr, 1, HFSR_VECTTBL));
        
        return take_exception(ARMv7M_EXCP_HARD);
    case HARD_FAULT_FORCED:
#if KVM_OPEN_DEBUG
        info("HardFault on fault escalation");
#endif
        fsr = _board_para->sysctl()->get(SC_REG_HFSR);
        _board_para->sysctl()->set(SC_REG_HFSR, set_bit(fsr, 1, HFSR_FORCED));
        
        return take_exception(ARMv7M_EXCP_HARD);
    case HARD_FAULT_DEBUGEVT:
#if KVM_OPEN_DEBUG
        info("HardFault on breakpoint (BKPT) escalation");
#endif
        fsr = _board_para->sysctl()->get(SC_REG_HFSR);
        _board_para->sysctl()->set(SC_REG_HFSR, set_bit(fsr, 1, HFSR_DEBUGEVT));
        
        return take_exception(ARMv7M_EXCP_HARD);
    case BUS_FAULT_STKERR:
#if KVM_OPEN_DEBUG
        info("BusFault on exception entry stack memory operations");
#endif
        fsr = _board_para->sysctl()->get(SC_REG_CFSR);
        _board_para->sysctl()->set(SC_REG_CFSR, set_bit(fsr, 1, BFSR_STKERR));        

        return take_exception(ARMv7M_EXCP_BUS);
    case BUS_FAULT_UNSTKERR:
#if KVM_OPEN_DEBUG
        info("BusFault on exception return stack memory operations");
#endif
        fsr = _board_para->sysctl()->get(SC_REG_CFSR);
        _board_para->sysctl()->set(SC_REG_CFSR, set_bit(fsr, 1, BFSR_UNSTKERR));  

        return take_exception(ARMv7M_EXCP_BUS);
    case BUS_FAULT_IBUSERR:
#if KVM_OPEN_DEBUG
        info("BusFault on instruction fetch, precise");
#endif
        fsr = _board_para->sysctl()->get(SC_REG_CFSR);
        _board_para->sysctl()->set(SC_REG_CFSR, set_bit(fsr, 1, BFSR_IBUSERR));  

        return take_exception(ARMv7M_EXCP_BUS);
    case BUS_FAULT_PRECISERR:
#if KVM_OPEN_DEBUG
        info("BusFault on data access, precise");
#endif
        fsr = _board_para->sysctl()->get(SC_REG_CFSR);
        fsr = set_bit(fsr, 1, BFSR_PRECISERR);
        _board_para->sysctl()->set(SC_REG_CFSR, set_bit(fsr, 1, BFSR_BFARVALID));  
        _board_para->sysctl()->set(SC_REG_BFAR, fpa);

        return take_exception(ARMv7M_EXCP_BUS);
    case BUS_FAULT_IMPRECISERR:
#if KVM_OPEN_DEBUG
        info("BusFault, bus error on data bus, imprecise");
#endif
        fsr = _board_para->sysctl()->get(SC_REG_CFSR);
        _board_para->sysctl()->set(SC_REG_CFSR, set_bit(fsr, 1, BFSR_IMPRECISERR));  

        return take_exception(ARMv7M_EXCP_BUS);
    case MEM_FAULT_MSTKERR:
#if KVM_OPEN_DEBUG
        info("MemManage fault on exception entry stack memory operations");
#endif
        fsr = _board_para->sysctl()->get(SC_REG_CFSR);
        _board_para->sysctl()->set(SC_REG_CFSR, set_bit(fsr, 1, MMFSR_MSTKERR));
        
        return take_exception(ARMv7M_EXCP_MEM);
    case MEM_FAULT_MUNSTKERR:
#if KVM_OPEN_DEBUG
        info("MemManage fault on exception return stack memory operations");
#endif
        fsr = _board_para->sysctl()->get(SC_REG_CFSR);
        _board_para->sysctl()->set(SC_REG_CFSR, set_bit(fsr, 1, MMFSR_MUNSTKERR));
        
        return take_exception(ARMv7M_EXCP_MEM);
    case MEM_FAULT_DACCVIOL:
#if KVM_OPEN_DEBUG
        info("MemManage fault on data access");
#endif
        fsr = _board_para->sysctl()->get(SC_REG_CFSR);
        fsr = set_bit(fsr, 1, MMFSR_DACCVIOL);
        _board_para->sysctl()->set(SC_REG_CFSR, set_bit(fsr, 1, MMFSR_MMFARVALID));
        _board_para->sysctl()->set(SC_REG_MMFAR, fpa);

        return take_exception(ARMv7M_EXCP_MEM);
    case MEM_FAULT_IACCVIOL:
#if KVM_OPEN_DEBUG
        info("MemManage fault on instruction access");
#endif
        fsr = _board_para->sysctl()->get(SC_REG_CFSR);
        _board_para->sysctl()->set(SC_REG_CFSR, set_bit(fsr, 1, MMFSR_IACCVIOL));

        return take_exception(ARMv7M_EXCP_MEM);
    case USAGE_FAULT_NOCP:
#if KVM_OPEN_DEBUG
        info("UsageFault, No coprocessor");
#endif
        fsr = _board_para->sysctl()->get(SC_REG_CFSR);
        _board_para->sysctl()->set(SC_REG_CFSR, set_bit(fsr, 1, UFSR_NOCP));

        return take_exception(ARMv7M_EXCP_USAGE);
    case USAGE_FAULT_UNDEFINSTR:
#if KVM_OPEN_DEBUG
        info("UsageFault, Undefined Instruction");
#endif
        fsr = _board_para->sysctl()->get(SC_REG_CFSR);
        _board_para->sysctl()->set(SC_REG_CFSR, set_bit(fsr, 1, UFSR_UNDEFINSTR));

        return take_exception(ARMv7M_EXCP_USAGE);
    case USAGE_FAULT_INVSTATE:
#if KVM_OPEN_DEBUG
        info("UsageFault, attempt to execute an instruction when EPSR.T==0");
#endif
        fsr = _board_para->sysctl()->get(SC_REG_CFSR);
        _board_para->sysctl()->set(SC_REG_CFSR, set_bit(fsr, 1, UFSR_INVSTATE));
        
        return take_exception(ARMv7M_EXCP_USAGE);
    case USAGE_FAULT_INVPC:
#if KVM_OPEN_DEBUG
        info("UsageFault, exception return integrity check failures");
#endif
        fsr = _board_para->sysctl()->get(SC_REG_CFSR);
        _board_para->sysctl()->set(SC_REG_CFSR, set_bit(fsr, 1, UFSR_INVPC));
        
        return take_exception(ARMv7M_EXCP_USAGE);
    case USAGE_FAULT_UNALIGNED:
#if KVM_OPEN_DEBUG
        info("UsageFault, illegal unaligned load or store");
#endif
        fsr = _board_para->sysctl()->get(SC_REG_CFSR);
        _board_para->sysctl()->set(SC_REG_CFSR, set_bit(fsr, 1, UFSR_UNALIGNED));
        
        return take_exception(ARMv7M_EXCP_USAGE);
    case USAGE_FAULT_DIVBYZERO:
#if KVM_OPEN_DEBUG
        info("UsageFault, divide by 0");
#endif
        fsr = _board_para->sysctl()->get(SC_REG_CFSR);
        _board_para->sysctl()->set(SC_REG_CFSR, set_bit(fsr, 1, UFSR_DIVBYZERO));

        return take_exception(ARMv7M_EXCP_USAGE);
    default:
        return 1;
    }
}

}  // namespace khost
