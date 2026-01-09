#include "rt_trigger.h"
#include "rt_nvic.h"
#include "rt_systick.h"

#define MAX_NVIC_IRQ        512
#define NVIC_IRQ_SYSTICK    15

static uint32_t last_check_time __attribute__((section(".rt_dmz_data"))) = 0;
static uint32_t next_irq_time[MAX_NVIC_IRQ] __attribute__((section(".rt_dmz_data")));

/// @brief reset the irq trigger
void trigger_reset() {
    for (int i = 0; i < RUNTIME_num_irq; i++) {
        next_irq_time[i] = 0;
    }
}

/// @brief calculate next irq trigger time
/// @param irq irq number
void calc_next_irq_time(int irq) {
    next_irq_time[irq] = RUNTIME_firmware_state.block_count + RUNTIME_IRQ_TRIGGER_PERIOD;
}

__attribute__((target("thumb")))
__attribute__((section(".rt_dmz_handler")))
/// @brief check if we should trigger irq now
/// @warning THIS FUNCTION IS CALLED IN FIRMWARE CONTEXT THUMB MODE
int RUNTIME_should_irq_trigger() {
    int pending_changed = 0;
    uint32_t block_count = RUNTIME_firmware_state.block_count;

    // only trigger after a sepecific init period
    if (RUNTIME_firmware_state.block_count < RUNTIME_init_period) {
        goto check_triggered;
    }

    // check systick first
    // most of time we should skip this, so we use unlikely here
    if (block_count > RUNTIME_firmware_state.block_next_irq) {
        pending_changed = 1;
        systick_handle_tick();
    }

    // check other interrupt
    // only check every IRQ_CHECK_PERIOD basic blocks
    if (block_count - last_check_time >= RUNTIME_IRQ_CHECK_PERIOD) {
        last_check_time = block_count;
        for (int i = NVIC_IRQ_SYSTICK + 1; i < RUNTIME_num_irq; i++) {
            // check if we reach the time
            uint32_t next_irq = next_irq_time[i];
            if (block_count <= next_irq) {
                continue;
            }
            // check if the irq is enabled
            if (!nvic_is_irq_enabled(i)) {
                continue;
            }
            // check if the irq can be triggerd
            uint32_t *vtor = (uint32_t *)RUNTIME_sysctl_state.vtor;
            uint32_t isr_addr = vtor[i] & 0xfffffffe;
            if (isr_addr == 0) {
                // we never trigger interrupt with no isr
                continue;
            }
            if (((uint16_t *)isr_addr)[0] == 0xe7fe
                || ((uint16_t *)isr_addr)[0] == 0xbe03
            ) {
                // we never trigger interrupt with infinitve loop
#if KVM_OPEN_DEBUG
                if (unlikely(should_output(RT_OUTPUT_TRIGGER))) {
                    debug_clear();
                    debug_append_str("[guest-trigger] skip infinite loop irq-0x");
                    debug_append_int(i);
                    debug_append_str("\n");
                    debug_print();
                }
#endif
                continue;
            }
            // set the irq state to pending
#if KVM_OPEN_DEBUG
            if (unlikely(should_output(RT_OUTPUT_TRIGGER))) {
                debug_clear();
                debug_append_str("[guest-trigger] set irq-0x");
                debug_append_int(i);
                debug_append_str(" pending\n");
                debug_print();
            }
#endif
#if PROTECT_RUNTIME
            asm volatile (
                "mov     r0, #0xf\n\t"
                "mcr     p15, 0, r0, c3, c0, 0\n\t"
                "isb\n\t"
                :
                :
                : "r0", "memory"
            );
#endif
            pending_changed = 1;
            nvic_set_pending(i);
            // update next trigger time
            next_irq_time[i] = block_count + RUNTIME_IRQ_TRIGGER_PERIOD;
            RUNTIME_emu_systick_state.irq_cnt += 1;
#if PROTECT_RUNTIME
            asm volatile (
                "mov     r0, #0x1\n\t"
                "mcr     p15, 0, r0, c3, c0, 0\n\t"
                "isb\n\t"
                :
                :
                : "r0", "memory"
            );
#endif
        }
    }

check_triggered:
    if (pending_changed) {
        int pending_exc_number = nvic_get_pending_irq();
        if (pending_exc_number == 0) {
            // we directly return to the guest if there's no exception
            return 0;
        }
        // context switch to runtime
        return 1;
    } else {
        // nothing changed! directly return to firmware maybe faster
        return 0;
    }
}
