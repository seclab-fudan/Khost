#ifndef RT_SYSTICK_H
#define RT_SYSTICK_H

#include "rt_global.h"
#include "rt_data.h"
#include "rt_debug.h"
#include "rt_nvic.h"
#include "rt_cpu.h"

void systick_reset();

int systick_handle_tick();

void systick_reload();

int systick_handle_mmio_write(uint32_t addr, uint32_t pc, uint32_t *data, uint32_t len);

int systick_handle_mmio_read(uint32_t addr, uint32_t pc, uint32_t *data, uint32_t len);

#endif  // RT_SYSTICK_H