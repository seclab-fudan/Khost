#ifndef EN_LIB_H
#define EN_LIB_H

#include "stdint.h"

uint8_t *get_coverage_ptr();

uint32_t get_coverage_size();

int reset_execution(uint8_t *input, int input_size);

int start_execution(int timeout);

int init_execution(char *config_path, char *working_dir);

void exit_execution();

int update_fuzzware_model();

uint32_t error_pc();

#endif