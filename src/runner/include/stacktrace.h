#ifndef BACKTRACE_H
#define BACKTRACE_H

#include "en_config.h"
#include "en_board.h"
#include "en_board_full.h"
#include "en_board_para.h"
#include <vector>

struct StackTrace {
    bool rt_crash;
    uint32_t cur_excp;
    uint32_t cur_task;
    FirmwareContext context;
    std::vector<uint32_t> call_site;
};

void print_trace(StackTrace *trace);

bool same_trace(const StackTrace &a, const StackTrace &b);

StackTrace stack_trace(khost::Board *board);

#endif  // BACKTRACE_H