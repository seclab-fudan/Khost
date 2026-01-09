#include "stacktrace.h"
#if KVM_OPEN_DEBUG
    #include "autogen_rt_full_debug.h"
#else
    #include "autogen_rt_full_normal.h"
#endif
#include "stdio.h"
#include <string>

static void print_context(StackTrace *trace) {
    uint32_t *regs = (uint32_t *)&trace->context;
    printf("Frimware Context: task_id=%u excp_id=%u\n", trace->cur_task, trace->cur_excp);
    printf("   PC=0x%08x   XPSR=0x%08x\n", regs[15], regs[16]);
    printf("   R0=0x%08x   R1=0x%08x   R2=0x%08x\n", regs[0], regs[1], regs[2]);
    printf("   R3=0x%08x   R4=0x%08x   R5=0x%08x\n", regs[3], regs[4], regs[5]);
    printf("   R6=0x%08x   R7=0x%08x   R8=0x%08x\n", regs[6], regs[7], regs[8]);
    printf("   R9=0x%08x  R10=0x%08x  R11=0x%08x\n", regs[9], regs[10], regs[11]);
    printf("  R12=0x%08x   SP=0x%08x   LR=0x%08x\n", regs[12], regs[13], regs[14]);
}

void print_trace(StackTrace *trace) {
    if (trace->rt_crash) {
        printf("[Warning: Crash in runtime]\n");
    }
    print_context(trace);
    printf("Stack Trace:\n");
    int id = 0;
    for (uint32_t addr: trace->call_site) {
        id += 1;
        uint32_t base_addr = 0x0;
        std::string name = "unknown";
        auto sym_table = khost::Config::instance().symbol_table();
        auto iter = sym_table->upper_bound(addr);
        if (iter != sym_table->begin()) {
            iter--;
            name = iter->second;
            base_addr = iter->first;
        }
        if (base_addr) {
            printf("  [%02d] 0x%08x: %s(+0x%x)\n", id, addr, name.c_str(), addr - base_addr);
        } else {
            printf("  [%02d] 0x%08x\n", id, addr);
        }
    }
}

static bool check_sp(uint32_t addr) {
    if (addr < SRAM_START || addr > SRAM_END) {
        return false;
    }
    khost::AddressInfo info = khost::Config::instance().address_info(addr);
    if (info.end == 0x0 && info.start == 0x0) {
        return false;
    }
    return true;
}

static bool check_in_code_range(uint32_t addr) {
    uint32_t firm_start = khost::Config::instance().firmware_info()->load_base;
    uint32_t firm_end = firm_start + khost::Config::instance().firmware_info()->firmware_code_size;
    return addr >= firm_start && addr < firm_end;
}

static uint32_t addr_after_rewrite(uint32_t addr) {
    auto reloc_map = khost::Config::instance().reloc_map();
    for (auto ite: *reloc_map) {
        if (ite.second == addr) {
            return ite.first;
        }
    }
    return addr;
}

static uint32_t check_potential_lr(khost::BoardFull *board_full, uint32_t value) {
    if (!check_in_code_range(value)) {
        // not in code range, return -1
        return 0xffffffff;
    }
    // the privous instruction should be bl or blx
    uint32_t cur_insn = value & (~0b1);
    if (cur_insn < 0x2) {
        return 0xffffffff;
    }
    uint32_t last_thumb_insn = board_full->read_u16(
        addr_after_rewrite(cur_insn - 0x2)
    );
    if (
        (((last_thumb_insn >> 11) & 0b11111) != 0b11101)
        && (((last_thumb_insn >> 11) & 0b11111) != 0b11110)
        && (((last_thumb_insn >> 11) & 0b11111) != 0b11111)
        && (((last_thumb_insn >> 7) & 0b111111111) == 0b010001111)
    ) {
        // this is 16 bit blx instruction
        return cur_insn - 0x2; 
    }
    if (cur_insn < 0x4) {
        return 0xffffffff;
    }
    uint32_t remain;
    uint16_t *last_arm_bytecode = (uint16_t *)board_full->gpa_to_hva(
        addr_after_rewrite(cur_insn - 0x4), &remain
    );
    if (last_arm_bytecode == nullptr || remain < 0x4) {
        return 0xffffffff;
    }
    if (
        (((last_arm_bytecode[0] >> 11) & 0b11111) == 0b11110)
        && (((last_arm_bytecode[1] >> 12) & 0b1101) == 0b1101)
    ) {
        // this is 32 bit bl instruction
        return cur_insn - 0x4;
    }
    return 0xffffffff;
}

/// @brief get the error stack trace of the firmware
/// @param board pointer to the base board
/// @return stack trace result
StackTrace stack_trace(khost::Board *board) {
    khost::BoardFull *board_full = dynamic_cast<khost::BoardFull *>(board);
    if (board_full == nullptr) {
        printf("INFO  | stacktrace: stack trace is only avaliable under full mode\n");
        return StackTrace();
    }

    StackTrace trace_res;
    trace_res.call_site.clear();
    if (board_full->read_u32(RT_FULL_first_crash_context_valid) == 1) {
        // check the firmware state when first crash
        uint32_t tmp_excp = board_full->read_u32(RT_FULL_first_crash_excp_id);
        uint32_t tmp_task = board_full->read_u32(RT_FULL_first_crash_task_id);
        FirmwareContext *tmp_context = (FirmwareContext *)board_full->gpa_to_hva(RT_FULL_first_crash_context);

        trace_res.cur_excp = tmp_excp;
        trace_res.cur_task = tmp_task;
        memcpy(&trace_res.context, tmp_context, sizeof(FirmwareContext));
    } else {
        // check the current firmware state 
        uint32_t tmp_excp = board_full->cpu()->ipsr();
        uint32_t tmp_task = board_full->read_u32(RT_FULL_current_task);
        FirmwareContext *tmp_context = (FirmwareContext *)board_full->gpa_to_hva(RT_FULL_firmware_context);

        trace_res.cur_excp = tmp_excp;
        trace_res.cur_task = tmp_task;
        memcpy(&trace_res.context, tmp_context, sizeof(FirmwareContext));        
    }

    // relocated pc and lr from rewrite address range to firmware range
    trace_res.context.pc = (
        khost::Config::instance().reloc_pc(trace_res.context.pc & (~0b1))
        | (trace_res.context.pc & 0b1)
    );
    trace_res.context.lr = (
        khost::Config::instance().reloc_pc(trace_res.context.lr & (~0b1))
        | (trace_res.context.lr & 0b1)
    );

    // check if the error is caused by runtime
    if (trace_res.context.pc >= RUNTIME_START && trace_res.context.pc <= RUNTIME_END) {
        trace_res.rt_crash = true;
    } else {
        trace_res.rt_crash = false;
    }
 
    // check the crash site
    if (check_in_code_range(trace_res.context.pc)) {
        trace_res.call_site.push_back(trace_res.context.pc);
    } else if (check_in_code_range(trace_res.context.lr)) {
        trace_res.call_site.push_back(trace_res.context.lr);
    }
    // backtrace the stack
    if ((trace_res.context.lr & 0xffffffe0) != 0xffffffe0) {
        uint32_t stack_bottom = trace_res.context.sp;
        uint32_t stack_top = board_full->read_u32(RT_FULL_per_task_initial_stack + trace_res.cur_task * 0x4);
        uint32_t sp = stack_bottom;
        while (true) {
            if (!check_sp(sp)) {
                // stack pointer is invalid, exit loop
                break;
            }
            uint32_t value = board_full->read_u32(sp);
            if (sp == stack_top) {
                // we reach the stack top of the current task, exit loop
                break;
            }
            if (trace_res.cur_excp != 0x0 && (value & 0xffffffe0) == 0xffffffe0) {
                // we reach the entrance of the exception handler, exit loop
                break;
            }
            // check if the value is potential lr
            uint32_t func = check_potential_lr(board_full, value);
            if (func == 0xffffffff) {
                // relocate to firmware and check again
                func = check_potential_lr(
                    board_full, 
                    khost::Config::instance().reloc_pc(value & (~0b1)) | (value & 0b1)
                );
            }
            if (func != 0xffffffff) {
                // we find a potential function
                trace_res.call_site.push_back(func);
            }
            sp += 0x1;
        }
    }

    return trace_res;
}

bool same_trace(const StackTrace &a, const StackTrace &b) {
    if (a.rt_crash || b.rt_crash) {
        return false;
    }
    if (a.cur_excp != b.cur_excp) {
        return false;
    }
    if (a.cur_excp == 0x0 && a.cur_task != b.cur_task) {
        return false;
    }
    if (a.call_site.size() != b.call_site.size()) {
        return false;
    }
    for (int i = 0; i < a.call_site.size(); i++) {
        if (a.call_site[i] != b.call_site[i]) {
            return false;
        }
    }
    return true;
}
