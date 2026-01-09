#ifndef DEV_EMU_SYSTICK_H
#define DEV_EMU_SYSTICK_H

#include "en_global.h"
#include "data_def.h"
#include "dev_mmio.h"
#include "en_config.h"

namespace khost {

class BoardPara;

class EmuSysTick : public MMIORegion {
public:
    EmuSysTick(BoardPara *board);
    ~EmuSysTick();

    BoardPara *board() { return _board; }

    // ========== MMIO handlers ==========
    int handle_read(uint64_t phys_addr, uint8_t *data, uint32_t len, uint32_t pc);
    int handle_write(uint64_t phys_addr, uint8_t *data, uint32_t len, uint32_t pc);
    int handle_tick();
    void reload();

private:
    BoardPara *_board;
    EmuSystickState *_inner;

    void flush_state();
    void info(const char *s, ...);
    void error(const char *s, ...);
};

}  // namespace khost

#endif  // DEV_EMU_SYSTICK_H