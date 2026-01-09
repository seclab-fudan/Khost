#ifndef DISASM_H
#define DISASM_H

#include "en_global.h"
#include "en_config.h"
#include "keystone/keystone.h"
#include "capstone/capstone.h"

#include <string>
#include <vector>

#define BRANCH_MAXDIST (16 * 1024 * 1024)
#define COND_BRANCH_MAXDIST (1024 * 1024)

namespace khost {

class DisIns {
public:
    DisIns(void *binary, csh handler, cs_insn insn);

    inline uint32_t size() { return _size; }
    inline uint32_t address() { return _address; }
    inline std::vector<uint8_t> bytes() { return _bytes; }
    inline std::string mnemonic() { return _mnemonic; }
    inline std::string op_str() { return _op_str; }
    inline bool is_it() { return _is_it; }
    inline uint32_t it_block_len() { return _it_block_len; }
    inline bool is_special_reg() { return _is_special_reg; }
    
    bool is_relocatable(uint32_t reloc_addr = -1);
    std::vector<uint8_t> relocated_bytes(ks_engine *ks, uint32_t reloc_addr);
    void print(FILE *fp);

private:
    bool _is_it;
    bool _is_special_reg;
    bool _is_relocatable;
    bool _is_cf;
    bool _is_conditional;
    uint32_t _size;
    uint32_t _address;
    uint32_t _target_addr;
    uint32_t _it_block_len;
    std::string _mnemonic;
    std::string _op_str;
    std::vector<uint8_t> _bytes;

    bool reads_pc(cs_insn insn);
    bool writes_pc(cs_insn insn);
    bool is_pop(cs_insn insn);
    bool is_control_flow_group(cs_insn insn);
    bool is_non_reloc_control_flow(cs_insn insn);
    bool is_reloc_control_flow(cs_insn insn);
    bool is_reloc_cond(cs_insn insn);
    bool is_reloc_uncond(cs_insn insn);
};

class DisBasicBlock {
public:
    DisBasicBlock(void *binary, csh handler, BasicBlock block);

    inline uint32_t start_addr() { return _start_addr; }
    inline uint32_t end_addr() { return _end_addr; }
    inline std::vector<DisIns> *insns() { return &_insns; }

    void print(FILE *fp);

private:
    void *_binary;
    uint32_t _start_addr;
    uint32_t _end_addr;
    std::vector<DisIns> _insns;

    void disasm(csh handler);
    void calc_detour_insn();
};

}  // namespace khost

#endif