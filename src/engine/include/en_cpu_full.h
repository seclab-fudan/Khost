#ifndef EN_CPU_FULL_H
#define EN_CPU_FULL_H

#include "en_cpu.h"

namespace khost {

class BoardFull;

class CPUFull : public CPU {
public:
    CPUFull(BoardFull *board);
    virtual ~CPUFull();

    inline BoardFull *board_full() { return _board_full; }
    
    // override base board function
    virtual McuState *inner() override { return _inner; }
    virtual bool reset() override;
    virtual int handle_arm_bkpt_exit(uint32_t arm_asm_code) override;
    virtual int handle_thumb_bkpt_exit(uint32_t pc, uint32_t thumb_asm_code) override;
    virtual void dump_state() override;

private:
    BoardFull *_board_full;
    McuState *_inner;
};

}  // namespace khost

#endif  // EN_CPU_FULL_H