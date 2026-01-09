#ifndef RT_NVIC_H
#define RT_NVIC_H

#include "rt_global.h"
#include "rt_data.h"
#include "rt_cpu.h"
#include "rt_debug.h"
#include "rt_sysctl.h"

void nvic_reset();

// set exception to pending
void nvic_set_pending(int irq);

// clear pending state of exception
void nvic_clear_pending(int irq);

// get pending exception bumber
int nvic_get_pending_irq();

// acknowledge when exception enter 
void nvic_acknowledge_irq();

// complete when exception return
int nvic_complete_irq(int irq);

// notify pending irq to the board and cpu
void nvic_irq_update();

// get vector info
NvicVecInfo* nvic_irq_info(int irq);

// find if we need to return to base level priority
int nvic_ret_to_base();

// get if isr pending
int nvic_isr_pending();

// check if cpu can take exception
int nvic_can_take_pending_exception();

int nvic_is_irq_enabled(int irq);

int nvic_exec_prio();

void nvic_recompute_state();

void nvic_set_prio(unsigned irq, int secure, uint8_t prio);

int nvic_handle_mmio_write(uint32_t phys_addr, uint32_t pc, uint32_t *data, uint32_t len);

int nvic_handle_mmio_read(uint32_t phys_addr, uint32_t pc, uint32_t *data, uint32_t len);


#endif  // RT_NVIC_H