#ifndef DEV_NVIC_H
#define DEV_NVIC_H

#include "en_global.h"
#include "data_def.h"
#include "dev_mmio.h"

namespace khost {

class BoardPara;

class NVIC : public MMIORegion {
public:
    NVIC(BoardPara *board, uint32_t base_addr = 0);
    ~NVIC() {}
    bool reset();

    // MMIO handlers ===========================================================
    int handle_read(uint64_t phys_addr, uint8_t *data, uint32_t len, uint32_t pc);
    int handle_write(uint64_t phys_addr, uint8_t *data, uint32_t len, uint32_t pc);

    // NVIC handlers ===========================================================
    // device handler, device irq=irqn, trigger when level change from 0 to 1
    void set_irq_level(int irqn, int level);
    // systick trigger, trigger when level is 1
    void systick_trigger(int level);
    // nmi trigger, trigger when level is 1
    void nmi_trigger(int level);
    // pendsv trigger, trigger when level is 1
    void pendsv_trigger(int level);
    // svc trigger, trigger when level is 1
    void svc_trigger(int level);
    // set exception to pending
    void set_pending(int irq);
    // clear pending state of exception
    void clear_pending(int irq);
    // get pending exception bumber
    int get_pending_irq();
    // acknowledge when exception enter 
    void acknowledge_irq();
    // complete when exception return
    int complete_irq(int irq);
    // notify pending irq to the board and cpu
    void irq_update();
    // get vector info
    inline NvicVecInfo* irq_info(int irq) { return &_inner->vectors[irq]; }
    // find if we need to return to base level priority
    bool ret_to_base();
    // get if isr pending
    bool isr_pending();
    // check if cpu can take exception
    bool can_take_pending_exception();

    bool is_irq_enabled(int irq) { return _inner->vectors[irq].enabled; }

    // execution priority
    int exec_prio();

private:
    BoardPara *_board;

    NvicState *_inner;

    // ========== NVIC Handler ==========
    // recompute cached states
    void recompute_state();
    // change exception raw priority to group priority
    int exc_group_prio(int rawprio);

    // group priority mask
    uint32_t gprio_mask();
    // set priority
    void set_prio(unsigned irq, bool secure, uint8_t prio);

    void flush_state();

    // Debug stuff =============================================================
    void error(const char *s, ...);
    void info(const char *s, ...);
    void debug(const char *s, ...);
};

}

#endif  // DEV_NVIC_H