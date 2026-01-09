#include "en_cpu_full.h"
#include "en_board_full.h"
#if KVM_OPEN_DEBUG
    #include "autogen_rt_full_debug.h"
#else
    #include "autogen_rt_full_normal.h"
#endif


namespace khost {

// #if KVM_OPEN_DEBUG
static clock_t clock_last;
static clock_t clock_current;
// #endif

//==============================================================================
// construct and destruct operations

/// @brief constructor
/// @param board belonging board
CPUFull::CPUFull(BoardFull *board) : CPU(board) {
    _board_full = board;
    _inner = (McuState *)board->gpa_to_hva(RT_FULL_mcu_state);
}

/// @brief destructor
CPUFull::~CPUFull() {

}

/// @brief reset cpu state and registers 
/// @return true on success, false on fail
bool CPUFull::reset() {
#if KVM_OPEN_DEBUG
    debug("reseting vcpu");
#endif
    if (!CPU::reset()) {
        return false;
    }
    
    // set pc value (directly write)
    uint32_t entry_pc = RT_FULL_vector_table;
    if (!set_kvm_reg(ARMv8A_PC, entry_pc)) {
        error("failed to set pc");
        return false;
    }

#if KVM_OPEN_DEBUG
    debug("reset finish");
#endif
    return true;
}

//==============================================================================
// Execution

#define ARM64_SYSREG_ESR_EL1 \
  (0x6030000000130000ULL | (3ULL << 14) | (0ULL << 11) | (5ULL << 7) | (2ULL << 3) | (0ULL << 0))

/// @brief handle DEBUG exit with ARM BKPT instruction
/// @param arm_asm_code instruction code
/// @return 
///  -2(UNHANDLE): not handled
///  -1(HANDLED): reenter the guest
///   1(EXIT_FIRMWARE_CRASH): crash inside the firmware
///   2(EXIT_INTERNAL_CRASH): crash because of the engine
/// @note ARM BKPT instructions are used in ARMv8-A style exception vector
int CPUFull::handle_arm_bkpt_exit(uint32_t arm_asm_code) {
    switch (arm_asm_code)
    {
    // DEBUG because of UND handler enter 
    case BKPT_ARM_UND: {
#if KVM_OPEN_DEBUG
        error("Undefined Instruction");
#endif
        // lr is the address of the undefined instruction + 0x2
        set_kvm_reg(ARMv8A_PC, get_kvm_reg(ARMv8A_LR_und) - 0x2);
        set_kvm_reg(ARMv8A_APSR, get_kvm_reg(ARMv8A_SPSR_und));
#if KVM_OPEN_DEBUG
        error("Instruction: 0x%08x", _board->read_u32(get_kvm_reg(ARMv8A_PC)));
        dump_state();
#endif
        return EXIT_FIRMWARE_CRASH;
    }
    // DEBUG because of PREFETCH abort handler
    case BKPT_ARM_PREFETCH: {
        // LR is the address of aborted instruction fetch + 0x4
        set_kvm_reg(ARMv8A_PC, get_kvm_reg(ARMv8A_LR_abt) - 0x4);
        set_kvm_reg(ARMv8A_APSR, get_kvm_reg(ARMv8A_SPSR_abt));
#if KVM_OPEN_DEBUG
        error("Prefetch Abort");
        dump_state();
#endif
        return EXIT_FIRMWARE_CRASH;
    }
    // DEBUG because of DATA abort handler 
    case BKPT_ARM_DATA: {
        // LR is the address of instruction that generated the abort + 0x8
        set_kvm_reg(ARMv8A_PC, get_kvm_reg(ARMv8A_LR_abt) - 0x8);
        set_kvm_reg(ARMv8A_APSR, get_kvm_reg(ARMv8A_SPSR_abt));
#if KVM_OPEN_DEBUG
        error("Data Abort");
        dump_state();
#endif
        // report the ESR
        uint32_t esr_val;
        struct kvm_one_reg reg = {
            .id = ARM64_SYSREG_ESR_EL1,
            .addr = (uint64_t)&esr_val,
        };
        ioctl(_vcpu_fd, KVM_GET_ONE_REG, &reg);
#if KVM_OPEN_DEBUG
        error("ESR = 0x%08x", esr_val);
#endif
        return EXIT_FIRMWARE_CRASH;
    }
    // DEBUG because of runtime debug
    case BKPT_ARM_DEBUG: {
        set_kvm_reg(ARMv8A_PC, get_kvm_reg(ARMv8A_PC) + 0x4);
#if KVM_OPEN_DEBUG
        char *output = _board->firmware_state()->debug_buffer;
        printf("GUEST | %s", output);
        fflush(stdout);
#endif
        return HANDLED;
    }
    case BKPT_ARM_FIRMWARE_CRASH: {
        uint32_t *regs = (uint32_t *)_board->gpa_to_hva(RT_FULL_firmware_context);
        if (get_kvm_reg(ARMv8A_PC) == 0x0 || regs[15] == 0) {
#if KVM_OPEN_DEBUG
            error("potential null pointer derefernece happened");
#endif
            // write state back to runtime
            if (get_kvm_reg(ARMv8A_PC) == 0x0) {
                regs[0] = get_kvm_reg(ARMv8A_R0); regs[1] = get_kvm_reg(ARMv8A_R1);
                regs[2] = get_kvm_reg(ARMv8A_R2); regs[3] = get_kvm_reg(ARMv8A_R3);
                regs[4] = get_kvm_reg(ARMv8A_R4); regs[5] = get_kvm_reg(ARMv8A_R5);
                regs[6] = get_kvm_reg(ARMv8A_R6); regs[7] = get_kvm_reg(ARMv8A_R7);
                regs[8] = get_kvm_reg(ARMv8A_R8_usr); regs[9] = get_kvm_reg(ARMv8A_R9_usr);
                regs[10] = get_kvm_reg(ARMv8A_R10_usr); regs[11] = get_kvm_reg(ARMv8A_R11_usr);
                regs[12] = get_kvm_reg(ARMv8A_R12_usr); regs[13] = get_kvm_reg(ARMv8A_SP_usr);
                regs[14] = get_kvm_reg(ARMv8A_LR_usr); regs[15] = get_kvm_reg(ARMv8A_PC);
            }
        }
#if KVM_OPEN_DEBUG
        dump_state();
#endif
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
int CPUFull::handle_thumb_bkpt_exit(uint32_t pc, uint32_t thumb_asm_code) {
    switch (thumb_asm_code) {
    // DEBUG because of coverage
    case BKPT_COVERAGE: {
#if KVM_OPEN_DEBUG
        debug("handling coverage at 0x%x", pc);
#endif

        if (pc == CONFIG.time_measure_begin_addr()) {
// #if KVM_OPEN_DEBUG
            clock_last = clock();
// #endif
            _board->reset_coverage_bkpt(pc);
        }
        else if (pc == CONFIG.time_measure_end_addr()) {
//#if KVM_OPEN_DEBUG
            clock_current = clock();
            print_mutex.lock();
            printf("INFO  | khost: passing time: %f ms\n", 
                    (clock_current - clock_last) / 1000.0);
            printf("INFO  | khost: pc = %08x, r0 = %08x, r1 = %08x, reg2 = %08x, reg3 = %08x \n", 
							pc, get_kvm_reg(ARMv8A_R0), get_kvm_reg(ARMv8A_R1), get_kvm_reg(ARMv8A_R2), get_kvm_reg(ARMv8A_R3));
            fflush(stdout);
            print_mutex.unlock();
//#endif
            return EXIT_OK;
        } else {
            if (pc == CONFIG.fire_after()) {
                board()->write_u32(RT_FULL_init_period, CONFIG.init_period());
#if KVM_OPEN_DEBUG
                info("reach interrupt fire point 0x%08x", pc);
#endif
            }
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
        info("guest exit from firmware after %d/%d basic blocks", pc, (int)blocks, (int)max_blocks);
        dump_state();
#endif
        return EXIT_OK;
    }
    // DEBUG beacuse of time measuring
    case BKPT_MEASURE_TIME: {
#if KVM_OPEN_DEBUG
        clock_current = clock();
        print_mutex.lock();
        printf("INFO  | khost: passing time: %f ms\n", 
                (clock_current - clock_last) / 1000.0);
        fflush(stdout);
        print_mutex.unlock();
        clock_last = clock_current;
#endif
        return (
            set_kvm_reg(ARMv8A_PC, get_kvm_reg(ARMv8A_PC) + 0x2) ?
            HANDLED : EXIT_INTERNAL_CRASH
        );
    }
    default:
        return UNHANDLED;
    }
}

void CPUFull::dump_state() {
    CPU::dump_state();
    uint32_t *regs = (uint32_t *)_board->gpa_to_hva(RT_FULL_firmware_context);
    error("Frimware Context:");
    error("   PC=0x%08x   XPSR=0x%08x", 
          regs[15], regs[16]);
    error("   R0=0x%08x   R1=0x%08x   R2=0x%08x", 
          regs[0], regs[1], regs[2]);
    error("   R3=0x%08x   R4=0x%08x   R5=0x%08x", 
          regs[3], regs[4], regs[5]);
    error("   R6=0x%08x   R7=0x%08x   R8=0x%08x", 
          regs[6], regs[7], regs[8]);
    error("   R9=0x%08x  R10=0x%08x  R11=0x%08x", 
          regs[9], regs[10], regs[11]);
    error("  R12=0x%08x   SP=0x%08x   LR=0x%08x", 
          regs[12], regs[13], regs[14]);
    error("=============================================================");
}

}  // namespace khost
