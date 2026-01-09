#include "disasm.h"

namespace khost {

#define MAX_INSN_BUFFER 256

DisIns::DisIns(void *binary, csh handler, cs_insn insn) {
    _target_addr = -1;
    _size = insn.size;
    _address = insn.address;
    _is_cf = false;
    _is_conditional = false;
    _mnemonic = std::string(insn.mnemonic);
    _op_str = std::string(insn.op_str);

    // write byte code
    _bytes.clear();
    uint32_t offset = _address - CONFIG.firmware_info()->load_base;
    uint8_t *ptr = (uint8_t *)((uint64_t)binary + offset);
    for (uint32_t i = 0; i < _size; i++) {
        _bytes.push_back(ptr[i]);
    }

    // check if then
    _is_it = _mnemonic.find("it") != std::string::npos;
    if (_is_it) {
        _it_block_len = _mnemonic.length() - 1;
    } else {
        _it_block_len = 0;
    }

    // check special reg
    _is_special_reg = false;
    for (int i = 0; i < insn.detail->arm.op_count; i++) {
        if (insn.detail->arm.operands[i].type == ARM_OP_SYSREG) {
            _is_special_reg = true;
            break;
        }
    }

    // check relocatable
    if (reads_pc(insn)) {
        _is_relocatable = false;
        return;
    }
    bool wpc = writes_pc(insn);
    if (wpc && is_pop(insn)) {
        // popping an address from the stack into the pc is
        // not position dependent, can be relocated
        _is_relocatable = true;
        return;
    } else if (wpc){
        // instructions write pc are not relocatable
        _is_relocatable = false;
        return;
    }
    if (!is_control_flow_group(insn)) {
        _is_relocatable = true;
        return;
    }
    if (is_non_reloc_control_flow(insn)) {
        _is_relocatable = false;
        return;
    } else if (is_reloc_control_flow(insn)) {
        _is_cf = true;
        _target_addr = insn.detail->arm.operands[0].imm;
        _is_conditional = is_reloc_cond(insn);
    }
    _is_relocatable = true;
}

bool DisIns::is_relocatable(uint32_t reloc_addr) {
    if (reloc_addr == (uint32_t)-1 || !_is_cf) {
        return _is_relocatable;
    }
    uint32_t dist = _is_conditional ? COND_BRANCH_MAXDIST : BRANCH_MAXDIST;
    return _is_relocatable 
           && std::abs((int64_t)reloc_addr - (int64_t)_target_addr) < dist;
}

bool DisIns::reads_pc(cs_insn insn) {
    for (int i = 0; i < insn.detail->arm.op_count; i++) {
        // register PC as operand
        if (insn.detail->arm.operands[i].type == ARM_OP_REG
            && insn.detail->arm.operands[i].reg == ARM_REG_PC
            && insn.detail->arm.operands[i].access == CS_AC_READ
        ) {
            return true;
        }
        // register PC as memory index or base
        if (insn.detail->arm.operands[i].type == ARM_OP_MEM
            && (
                insn.detail->arm.operands[i].mem.base == ARM_REG_PC
                || insn.detail->arm.operands[i].mem.index == ARM_REG_PC
            )
            && insn.detail->arm.operands[i].access == CS_AC_READ
        ) {
            return true;
        }
    }
    return false;
}

bool DisIns::writes_pc(cs_insn insn) {
    for (int i = 0; i < insn.detail->arm.op_count; i++) {
        if (insn.detail->arm.operands[i].type == ARM_OP_REG
            && insn.detail->arm.operands[i].reg == ARM_REG_PC
            && insn.detail->arm.operands[i].access == CS_AC_WRITE
        ) {
            return true;
        }   
    }
    return false;
} 

bool DisIns::is_pop(cs_insn insn) {
    if (_mnemonic == "pop")
        return true;
    if (_mnemonic == "pop.w")
        return true;
    return false;
}

bool DisIns::is_control_flow_group(cs_insn insn) {
    for (int i = 0; i < insn.detail->groups_count; i++) {
        if (insn.detail->groups[i] == CS_GRP_CALL) {
            return true;
        }
        if (insn.detail->groups[i] == CS_GRP_JUMP) {
            return true;
        }
        if (insn.detail->groups[i] == CS_GRP_RET) {
            return true;
        }
    }
    return false;
}

bool DisIns::is_non_reloc_control_flow(cs_insn insn) {
    if (_mnemonic.find("cbz") != std::string::npos)
        return true;
    if (_mnemonic.find("cbnz") != std::string::npos)
        return true;
    return false;
}

bool DisIns::is_reloc_control_flow(cs_insn insn) {
    return is_reloc_cond(insn) || is_reloc_uncond(insn);
}

bool DisIns::is_reloc_cond(cs_insn insn) {
    std::string ops[24] = {
        "bne", "bne.w", "blt", "blt.w", "ble", "ble.w", "beq", "beq.w",
        "bge", "bge.w", "bgt", "bgt.w", "bhi", "bhi.w", "bls", "bls.w",
        "bmi", "bmi.w", "bpl", "bpl.w", "blo", "blo.w", "bhs", "bhs.w",
    };
    for (int i = 0; i < 24; i++) {
        if (_mnemonic == ops[i]) {
            return true;
        }
    }
    return false;
}

bool DisIns::is_reloc_uncond(cs_insn insn) {
    if (_mnemonic == "bl")
        return true;
    if (_mnemonic == "b")
        return true;
    if (_mnemonic == "b.w")
        return true;
    return false;
}

std::vector<uint8_t> DisIns::relocated_bytes(ks_engine *ks, uint32_t reloc_addr) {
    if (!_is_cf) {
        return _bytes;
    }
    
    char imm_buffer[256];
    sprintf(imm_buffer, "#%lld", 
            (int64_t)_target_addr - (int64_t)reloc_addr - (_is_conditional ? 4ll : 0ll));
    std::string asm_code = _mnemonic + " " + std::string(imm_buffer); 
    
    size_t size, count;
    unsigned char *ks_bytes;
    if (ks_asm(ks, asm_code.c_str(), 0, &ks_bytes, &size, &count) != KS_ERR_OK) {
        printf("failed to compile relocated bytes\n");
        exit(-1);
    }
    std::vector<uint8_t> buf;
    for (uint32_t i = 0; i < size; i++) {
        buf.push_back(ks_bytes[i]);
    }
    ks_free(ks_bytes);
    return buf;
}

void DisIns::print(FILE *fp) {
    if (is_relocatable()) {
        fprintf(fp, " R");
    } else {
        fprintf(fp, "  ");
    }
    if (_is_cf) {
        if (_is_conditional) {
            fprintf(fp, "C ");
        } else {
            fprintf(fp, "B ");
        }
    } else {
        fprintf(fp, "  ");
    }
    fprintf(fp, "| 0x%08x | ", _address);
    for (auto i: _bytes) {
        fprintf(fp, "%02x ", i);
    }
    for (uint32_t i = 0; i < 4 - _bytes.size(); i++) {
        fprintf(fp, "   ");
    }
    fprintf(fp, "| %s %s", _mnemonic.c_str(), _op_str.c_str());
    fprintf(fp, "\n");
}

DisBasicBlock::DisBasicBlock(void *binary, csh handler, BasicBlock block) {
    _binary = binary;
    _start_addr = block.start;
    _end_addr = block.end;
    _insns.clear();
    disasm(handler);
}

void DisBasicBlock::disasm(csh handler) {
    cs_insn *insn_buffer;
    uint32_t offset = _start_addr - CONFIG.firmware_info()->load_base;
    uint8_t *code = (uint8_t *)((uint64_t)_binary + offset);
    uint32_t code_size = _end_addr - _start_addr;
    size_t cnt = cs_disasm(handler, code, code_size, _start_addr, 0, &insn_buffer);
    for (uint32_t i = 0; i < cnt; i++) {
        _insns.push_back(DisIns(_binary, handler, insn_buffer[i]));
    }
    cs_free(insn_buffer, cnt);
}

void DisBasicBlock::calc_detour_insn() {

}

void DisBasicBlock::print(FILE *fp) {
    fprintf(fp, "basicblock @ 0x%08x (%s):\n", 
           _start_addr, CONFIG.belonging_symbol(_start_addr).c_str());
    for (auto i: _insns) {
        fprintf(fp, "  ");
        i.print(fp);
    }
}

}  // namespace khost