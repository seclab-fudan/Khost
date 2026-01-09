#ifndef RT_DEBUG_H
#define RT_DEBUG_H

#include "data_def.h"

void debug_clear();

void debug_append_int(uint32_t val);

void debug_append_str(const char *s);

void debug_print();

int should_output(int target);

#endif  // RT_DEBUG_H