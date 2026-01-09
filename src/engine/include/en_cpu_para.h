#ifndef EN_CPU_PARA_H
#define EN_CPU_PARA_H

#include "en_cpu.h"

namespace khost {

class BoardPara;

class CPUPara : public CPU {
public:
    CPUPara(BoardPara *board);
    virtual ~CPUPara();

    inline BoardPara *board_full() { return _board_para; }
    virtual McuState *inner() override { return _inner; }

    virtual bool reset() override;

    // special registers getter
    int sp_select();
    uint32_t msp();
    uint32_t psp();
    uint32_t primask(); 
    uint32_t faultmask();
    uint32_t basepri(); 
    uint32_t control(); 
    
    // special registers setter
    bool set_msp(uint32_t value);
    bool set_psp(uint32_t value);
    bool set_apsr(uint32_t value);
    bool set_epsr(uint32_t value);
    void set_ipsr(uint32_t value);
    void set_primask(uint32_t value);
    void set_faultmask(uint32_t value);
    void set_basepri(uint32_t value);
    bool set_control(uint32_t value);

    virtual int handle_mmio_exit() override;
    virtual int handle_arm_bkpt_exit(uint32_t arm_asm_code);
    virtual int handle_thumb_bkpt_exit(uint32_t pc, uint32_t thumb_asm_code);

private:
    BoardPara *_board_para;
    McuState *_inner;

    int handle_irq();
    int handle_svc();
    int handle_breakpoint();
    int handle_exception_enter(uint32_t exc_number);
    int handle_exception_return(uint32_t exc_return);
    int handle_incompatible_insn(uint32_t pc);
    int handle_mrs(IncompatibleInsn *insn);
    int handle_msr(IncompatibleInsn *insn);
    int handle_cps(IncompatibleInsn *insn);

    std::stack<uint32_t> _last_bb_stack;
    bool _mmio_ran_out;

    int inject_fault(int type, uint32_t fpa);
    uint32_t set_bit(uint32_t value, int bit, int id);
    int can_take_exception(uint32_t id);
    int take_exception(uint32_t id);
};

}  // namespace khost

#endif  // EN_CPU_PARA_H