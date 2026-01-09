#ifndef DEV_SYS_CTL_H
#define DEV_SYS_CTL_H

#include "en_global.h"
#include "data_def.h"
#include "dev_mmio.h"
#include <map>
#include <string>

// values [to-do]
// #define R_V7M_AIRCR_PRIS_MASK       (7 << 8)
// #define R_V7M_HFSR_FORCED_MASK      (1 << 30)
// #define R_V7M_FPCCR_HFRDY_MASK      (1 << 4)
// #define R_V7M_FPCCR_MONRDY_MASK     (1 << 8)
// #define R_V7M_FPCCR_MMRDY_MASK      (1 << 5)
// #define R_V7M_FPCCR_BFRDY_MASK      (1 << 6)

namespace khost {

class BoardPara;

// ARMv7-M System Control
// 0xe000ed00 - 0xe000ed8f      System Control Block
class SysCtl : public MMIORegion {
public:
    SysCtl(BoardPara *board, uint32_t base_addr = 0x0);
    ~SysCtl() {}

    bool reset();
    uint32_t get(uint32_t reg);
    void set(uint32_t reg, uint32_t value);

    int handle_read(uint64_t phys_addr, uint8_t *data, uint32_t len, uint32_t pc);
    int handle_write(uint64_t phys_addr, uint8_t *data, uint32_t len, uint32_t pc);

private:
    BoardPara *_board;

    SysctlState *_inner;

    void error(const char *s, ...);
    void info(const char *s, ...);

    void flush_state();
    std::map<uint32_t, std::string> _reg_names;
    void init_reg_names();
    const char *reg_name(uint32_t reg);
};

}

#endif  // DEV_SYS_CTL_H