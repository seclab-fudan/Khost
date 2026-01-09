#ifndef RT_FUZZWARE_H
#define RT_FUZZWARE_H

#include "stdint.h"

void fuzzware_reset();

void fuzzware_handle_mmio_write(uint32_t phys_addr, uint32_t pc, uint32_t *data, uint32_t len);

void fuzzware_handle_mmio_read(uint32_t phys_addr, uint32_t pc, uint32_t *data, uint32_t len);


#endif  // RT_FUZZWARE_H