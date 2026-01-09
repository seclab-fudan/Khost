#ifndef RT_SYSCTL_H
#define RT_SYSCTL_H

#include "rt_global.h"
#include "rt_data.h"
#include "rt_nvic.h"
#include "rt_cpu.h"
#include "rt_debug.h"

void sysctl_reset();

uint32_t sysctl_get(uint32_t reg);

void sysctl_set(uint32_t reg, uint32_t value);

int sysctl_handle_mmio_write(uint32_t addr, uint32_t pc, uint32_t *data, uint32_t len);

int sysctl_handle_mmio_read(uint32_t addr, uint32_t pc, uint32_t *data, uint32_t len);

#endif  // RT_SYSCTL_H