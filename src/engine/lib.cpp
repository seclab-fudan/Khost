#include "en_global.h"
#include "en_board_full.h"
#include "en_config.h"
#include <signal.h>
#include <cstdlib>

#define INITIAL_INPUT_SIZE  8192
#define INITIAL_TIMEOUT     (10 * 1000000)

static khost::BoardFull *board;

extern "C"
{

uint8_t *get_coverage_ptr() {
    return board->coverage_map();
}

uint32_t get_coverage_size() {
    return COVERAGE_MAP_SIZE;
}

static uint8_t *last_input;
static int last_input_size;

int reset_execution(uint8_t *input, int input_size) {
    last_input = input;
    last_input_size = input_size;
    return board->reset(input, input_size);
}

int update_fuzzware_model() {
    if (board->update_fuzzware_model()) {
        return 0;
    } else {
        return 1;
    }
}

int start_execution(int timeout) {
    while(true) {
        if (timeout) {
            board->cpu()->set_alarm(timeout);
        }
        int ret = board->run();
        if (timeout) {
            board->cpu()->set_alarm(0);
        }
        if (ret == 4) {
            if (update_fuzzware_model() != 0) {
                // exit normally with no coverage
                return 0;
            }
        } else {
            board->report_coverage();
            return ret;
        }
    }
}

int init_execution(const char *config_path, const char *working_dir) {
    if (!khost::Config::instance().load_from_file(FULL_MODE, config_path, working_dir)) {
        return false;
    }
    khost::Config::instance().option_set_output_level(OUTPUT_LEVEL_ERROR);
    khost::Config::instance().option_set_output_target(EN_OUTPUT_BOARD, false);
    khost::Config::instance().option_set_output_target(EN_OUTPUT_CPU, false);
    khost::Config::instance().option_set_output_target(EN_OUTPUT_FUZZWARE, false);
    khost::Config::instance().option_set_output_target(EN_OUTPUT_NVIC, false);
    khost::Config::instance().option_set_output_target(EN_OUTPUT_SYSCTL, false);
    khost::Config::instance().option_set_output_target(EN_OUTPUT_SYSTICK, false);
    khost::Config::instance().option_set_output_target(EN_OUTPUT_LOADER, false);
    khost::Config::instance().option_set_output_target(RT_OUTPUT_CPU, false);
    khost::Config::instance().option_set_output_target(RT_OUTPUT_EMU, false);
    khost::Config::instance().option_set_output_target(RT_OUTPUT_FAULT, false);
    khost::Config::instance().option_set_output_target(RT_OUTPUT_FUZZWARE, false);
    khost::Config::instance().option_set_output_target(RT_OUTPUT_HANDLER, false);
    khost::Config::instance().option_set_output_target(RT_OUTPUT_NVIC, false);
    khost::Config::instance().option_set_output_target(RT_OUTPUT_SYSCTL, false);
    khost::Config::instance().option_set_output_target(RT_OUTPUT_SYSTICK, false);
    khost::Config::instance().option_set_output_target(RT_OUTPUT_TRIGGER, false);
    khost::Config::instance().option_set_use_hugepage(false);

    board = new khost::BoardFull();
    if (board == nullptr) {
        printf("failed to alloc board\n");
        return 0;
    }
    if (!board->init()) {
        printf("failed to create board\n");
        return 0;
    }

    if (strlen(working_dir)) {
        unsigned char *buffer = (unsigned char *)malloc(INITIAL_INPUT_SIZE);
        printf("[!] running all-zero input to generate initial mmio model\n");
        memset(buffer, 0, INITIAL_INPUT_SIZE);
        reset_execution(buffer, INITIAL_INPUT_SIZE);
        start_execution(INITIAL_TIMEOUT);
        printf("[!] running all-one input to generate initial mmio model\n");
        memset(buffer, 0xff, INITIAL_INPUT_SIZE);
        reset_execution(buffer, INITIAL_INPUT_SIZE);
        start_execution(INITIAL_TIMEOUT);
        printf("[!] all done\n");
        free(buffer);
    }

    return 1;
}

void exit_execution() {
    delete board;
}

uint32_t error_pc() {
    return board->error_pc();
}

}
