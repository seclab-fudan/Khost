#ifndef CPU_H
#define CPU_H

#include "en_global.h"
#include "data_def.h"
#include "loader.h"

#include <pthread.h>
#include <mutex>
#include <stack>
#include <vector>

namespace khost {

class Board;

FIELD(DBGWCR, E, 0, 1)
FIELD(DBGWCR, PAC, 1, 2)
FIELD(DBGWCR, LSC, 3, 2)
FIELD(DBGWCR, BAS, 5, 8)
FIELD(DBGWCR, HMC, 13, 1)
FIELD(DBGWCR, SSC, 14, 2)
FIELD(DBGWCR, LBN, 16, 4)
FIELD(DBGWCR, WT, 20, 1)
FIELD(DBGWCR, MASK, 24, 5)
FIELD(DBGWCR, SSCE, 29, 1)

/* GDB breakpoint/watchpoint types */
#define GDB_BREAKPOINT_SW        0
#define GDB_BREAKPOINT_HW        1
#define GDB_WATCHPOINT_WRITE     2
#define GDB_WATCHPOINT_READ      3
#define GDB_WATCHPOINT_ACCESS    4

/* Breakpoint/watchpoint flags */
#define BP_MEM_READ             0x01
#define BP_MEM_WRITE            0x02
#define BP_MEM_ACCESS           (BP_MEM_READ | BP_MEM_WRITE)
#define BP_HIT_SHIFT            6
#define BP_WATCHPOINT_HIT_READ  (BP_MEM_READ << BP_HIT_SHIFT)
#define BP_WATCHPOINT_HIT_WRITE (BP_MEM_WRITE << BP_HIT_SHIFT)
#define BP_WATCHPOINT_HIT       (BP_MEM_ACCESS << BP_HIT_SHIFT)

struct CPUWatchpoint {
    uint32_t vaddr;
    uint32_t len;
    uint32_t hitaddr;
    int flags;
};

struct HWBreakpoint { 
    uint64_t bcr; 
    uint64_t bvr; 
};

struct HWWatchpoint { 
    uint64_t wcr; 
    uint64_t wvr; 
    CPUWatchpoint details; 
};

class CPU {
public:
    CPU(Board *board);
    virtual ~CPU();
    
    Board *board() { return _board; }
    virtual McuState *inner() = 0;
    inline bool init_succ() { return _init_succ; }

    // kvm internal ============================================================
    int run();
    int loop();
    virtual bool reset();
    bool set_alarm(uint64_t usec);
    bool running();
    void set_running(bool value);

    // access register from kvm
    uint32_t get_kvm_reg(uint64_t id);
    uint32_t get_kvm_fp_reg(int id);
    bool set_kvm_reg(uint64_t id, uint32_t value);
    bool set_kvm_fp_reg(int id, uint32_t value);
    uint32_t apsr();
    uint32_t epsr();
    uint32_t ipsr();
    uint32_t xpsr();
    virtual void dump_state();

    // Debug stuff =============================================================
    bool init_debug();
    bool update_debug();
    inline int max_hw_bps() { return _max_hw_bps; }
    inline int max_hw_wps() { return _max_hw_wps; }
    bool is_single_stepping();
    bool set_single_stepping(bool enable);
    bool insert_hw_breakpoints(uint32_t addr);
    bool delete_hw_breakpoints(uint32_t addr);
    bool insert_hw_watchpoints(uint32_t addr, uint32_t len, int type);
    bool delete_hw_watchpoints(uint32_t addr, uint32_t len, int type);
    bool check_watchpoint_in_range(HWWatchpoint wp, uint32_t addr, uint32_t size);
    void remove_all_hw_breakpoints();
    HWBreakpoint hit_hw_breakpoint();
    HWWatchpoint hit_hw_watchpoint();

protected:
    
    // kvm internal ============================================================
    Board *_board;
    bool _init_succ = false;
    int _vcpu_fd = -1;
    int _kvm_run_size = -1;
    struct kvm_run *_kvm_run = nullptr;
    bool _is_running = false;
    
    // access register from kvm
    uint32_t do_get_kvm_reg(uint64_t kvm_id);
    bool do_set_kvm_reg(uint64_t kvm_id, uint32_t value);

    // kvm internal ============================================================
    int enter_exit();
    int handle_bkpt_exit();
    virtual int handle_mmio_exit();
    virtual int handle_arm_bkpt_exit(uint32_t arm_asm_code) = 0;
    virtual int handle_thumb_bkpt_exit(uint32_t pc, uint32_t thumb_asm_code) = 0;

    // output functions
    void error(const char *s, ...);
    void info(const char *s, ...);
    void debug(const char *s, ...);

    // debug stuff
    bool _single_stepping = false;
    int _max_hw_bps = 0;
    int _max_hw_wps = 0;
    std::vector<HWBreakpoint> _hw_bps;
    std::vector<HWWatchpoint> _hw_wps;
    int _hit_point_id;

    int get_insn_type(uint32_t opcode);
    uint64_t decode_insn(uint64_t *access_len);
    uint64_t arm_reg_to_kvm(arm_reg reg, int *reg_len);
    int arm_reg_size(arm_reg reg);
    uint64_t decode_arm_insn(cs_insn *insn, uint64_t *access_len);
};

}  // namespace khost

#endif  // CPU_H