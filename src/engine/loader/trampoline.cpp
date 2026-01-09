#include "trampoline.h"
#include "loader.h"

namespace khost {

std::string PROLOGUE_ASM = R"delimiter(
    # save context
    push {r0, r1, r2, r3, ip, lr};
    # save application program state (e.g., flags used for comparisons)
    mrs r0, apsr;
    push {r0};
)delimiter";

void *PrologueAddition::bytecode() {
    size_t size, count;
    unsigned char *ks_bytes;
    if (ks_asm(_loader->ks_thumb(), PROLOGUE_ASM.c_str(), 0, &ks_bytes, &size, &count) != KS_ERR_OK) {
        printf("failed to compile prologue\n");
        exit(-1);
    }
    void *code_part_buffer = mmap(NULL, size, 
                                  PROT_READ | PROT_WRITE, 
                                  MAP_SHARED | MAP_ANONYMOUS, -1, 0);
    memcpy(code_part_buffer, ks_bytes, size);
    ks_free(ks_bytes);
    return code_part_buffer;
}

uint32_t PrologueAddition::calc_size() {
    size_t size, count;
    unsigned char *ks_bytes;
    if (ks_asm(_loader->ks_thumb(), PROLOGUE_ASM.c_str(), 0, &ks_bytes, &size, &count) != KS_ERR_OK) {
        printf("failed to compile prologue\n");
        exit(-1);
    }
    ks_free(ks_bytes);
    return size;
}

void PrologueAddition::set_addr(uint32_t addr) {
    Addition::set_addr(addr);
}

std::string EPILOGUE_ASM = R"delimiter(
    # restore application program state (e.g., flags used for comparisons)
    pop {r0};
    msr apsr, r0;
    # restore context
    pop {r0, r1, r2, r3, ip, lr};
)delimiter";

void *EpilogueAddition::bytecode() {
    size_t size, count;
    unsigned char *ks_bytes;
    if (ks_asm(_loader->ks_thumb(), EPILOGUE_ASM.c_str(), 0, &ks_bytes, &size, &count) != KS_ERR_OK) {
        printf("failed to compile epilogue\n");
        exit(-1);
    }
    void *code_part_buffer = mmap(NULL, size, 
                                  PROT_READ | PROT_WRITE, 
                                  MAP_SHARED | MAP_ANONYMOUS, -1, 0);
    memcpy(code_part_buffer, ks_bytes, size);
    ks_free(ks_bytes);
    return code_part_buffer;
}

uint32_t EpilogueAddition::calc_size() {
    size_t size, count;
    unsigned char *ks_bytes;
    if (ks_asm(_loader->ks_thumb(), EPILOGUE_ASM.c_str(), 0, &ks_bytes, &size, &count) != KS_ERR_OK) {
        printf("failed to compile epilogue\n");
        exit(-1);
    }
    ks_free(ks_bytes);
    return size;
}

void EpilogueAddition::set_addr(uint32_t addr) {
    Addition::set_addr(addr);
}

TrampolineAddition::TrampolineAddition(Loader *loader, void *binary,
                                       BasicBlock basic_block) 
    : Addition(loader), _binary(binary) {
    _basic_block = new DisBasicBlock(binary, loader->cs(), basic_block);
    _additions.clear();
    _additions.push_back(new PrologueAddition(loader));
    _additions.push_back(new CoverageAddition(loader));
    _additions.push_back(new EpilogueAddition(loader));
}

TrampolineAddition::~TrampolineAddition() {
    delete _basic_block;
    for (auto addition: _additions) {
        delete addition;
    }
}

void *TrampolineAddition::bytecode() {
    // current offset
    uint32_t offset = 0;
    // merge all code parts
    void *code_buffer = mmap(NULL, size(), 
                             PROT_READ | PROT_WRITE, 
                             MAP_SHARED | MAP_ANONYMOUS, -1, 0);
    for (auto addition: _additions) {
        void *ptr = (void *)((uint64_t)code_buffer + offset);
        void *part_code = addition->bytecode();
        uint32_t part_size = addition->size();
        memcpy(ptr, part_code, part_size);
        munmap(part_code, part_size);
        offset += addition->size();
    }
    // add relocated bytes
    uint32_t reloc_offset_start = offset;
    std::vector<uint8_t> reloc_bytes = calc_relocate_insns();
    for (int i = 0; i < MAX_RELOCATE_SIZE; i++) {
        uint8_t *ptr = (uint8_t *)((uint64_t)code_buffer + offset);
        ptr[0] = reloc_bytes[i];
        offset += 1;
    }

    uint32_t lr_pos = MAX_RELOCATE_SIZE;
    while (true) {
        if (lr_pos < 2 || lr_pos - 1 >= MAX_RELOCATE_SIZE) {
            break;
        }
        uint8_t a = reloc_bytes[lr_pos-2];
        uint8_t b = reloc_bytes[lr_pos-1];
        if (a != 0x00 || b != 0xbf) {
            break;
        }
        lr_pos -= 2;
    }
    // hack: relocate potential LR register
    CONFIG.set_additional_insn_map(
        _basic_block->start_addr() + _trampoline_call_off + _reloc_sz,
        addr() + reloc_offset_start + lr_pos
    );

    // add paddings
    while (offset % 4 != 0) {
        uint8_t *ptr = (uint8_t *)((uint64_t)code_buffer + offset);
        ptr[0] = 0x00;
        ptr[1] = 0xbf;
        offset += 2;
    }
    // add ret branch
    uint32_t *ptr = (uint32_t *)((uint64_t)code_buffer + offset);
    ptr[0] = 0xf000f8df;  // ldr.w pc, =IMM
    ptr[1] = (_basic_block->start_addr() + _trampoline_call_off + _reloc_sz) | 0b1;
    
    if (_trampoline_canfit) {
        for (uint32_t i = 0; i < _reloc_sz; i++) {
            uint32_t origin_addr = _basic_block->start_addr() + _trampoline_call_off + i;
            uint32_t reloc_addr = addr() + reloc_offset_start + i;
            CONFIG.set_reloc_insn(origin_addr, reloc_addr);
            for (auto model_map: *CONFIG.fuzzware_model()) {
                for (auto model: model_map.second) {
                    FuzzwareModelInfo *info = model.second;
                    if (info->pc == origin_addr) {
                        if (CONFIG.option_save_rewrite_result()) {
                            std::string res_path = CONFIG.option_save_rewrite_result_path();
                            FILE *fp = fopen(res_path.c_str(), "a");
                            if (fp) {
                                fprintf(fp, "patch model: pc: 0x%08x -> 0x%08x, addr: 0x%08x\n", origin_addr, reloc_addr, info->addr);
                                fclose(fp);
                            } else {
                                printf("ERROR | khost: failed to write file: %s\n", res_path.c_str());
                                exit(-1);
                            }
                        }
                        info->pc = reloc_addr;
                    }
                }
            }
            auto insn_map = CONFIG.get_incompatible_insn_map();
            if (insn_map->find(origin_addr) != insn_map->end()) {
                auto insn_info = (*insn_map)[origin_addr];
                insn_map->erase(origin_addr);
                insn_info->addr = reloc_addr;
                (*insn_map)[reloc_addr] = insn_info;
            }
            for (unsigned int i = 0; i < CONFIG.hook_exit_addrs()->size(); i++) {
                if ((*CONFIG.hook_exit_addrs())[i] == origin_addr) {
                    (*CONFIG.hook_exit_addrs())[i] = reloc_addr;
                }
            }
        }
        patch_detour();
    }

    if (CONFIG.option_save_rewrite_result()) {
        std::string res_path = CONFIG.option_save_rewrite_result_path();
        FILE *fp = fopen(res_path.c_str(), "a");
        if (fp) {
            fprintf(fp, "basicblock @ 0x%08x (%s):\n", 
                _basic_block->start_addr(), 
                CONFIG.belonging_symbol(_basic_block->start_addr()).c_str()
            );
            uint32_t bb_offset = 0;
            for (auto i: *_basic_block->insns()) {
                if (bb_offset >= _trampoline_call_off && bb_offset < _trampoline_call_off + _reloc_sz) {
                    fprintf(fp, " ==reloc==> ");
                } else {
                    fprintf(fp, "            ");
                }
                i.print(fp);
                bb_offset += i.size();
            }
            if (_trampoline_canfit) {
                fprintf(fp, "trampoline set to 0x%08x\n", addr());
                cs_insn *insn_buffer;
                uint32_t tram_offset = 0;
                size_t cnt = cs_disasm(_loader->cs(), (uint8_t *)code_buffer, size()-4, addr(), 0, &insn_buffer);
                for (size_t i = 0; i < cnt; i++) {
                    fprintf(fp, "                | 0x%08x | ", tram_offset + addr());
                    for (int j = 0; j < insn_buffer[i].size; j++) {
                        fprintf(fp, "%02x ", ((uint8_t *)code_buffer)[j + tram_offset]);
                    }
                    for (int j = 0; j < 4-insn_buffer[i].size; j++) {
                        fprintf(fp, "   ");
                    }
                    fprintf(fp, "| %s %s\n", insn_buffer[i].mnemonic, insn_buffer[i].op_str);
                    tram_offset += insn_buffer[i].size;
                }
                fprintf(fp, "                | 0x%08x | %02x %02x %02x %02x | ----> 0x%08x\n", 
                    tram_offset + addr(),
                    ((uint8_t *)code_buffer)[tram_offset + 0],
                    ((uint8_t *)code_buffer)[tram_offset + 1],
                    ((uint8_t *)code_buffer)[tram_offset + 2],
                    ((uint8_t *)code_buffer)[tram_offset + 3],
                    ((uint32_t *)((uint64_t)code_buffer + tram_offset))[0]
                );
                cs_free(insn_buffer, cnt);

                fprintf(fp, "patched basic block:\n");
                DisBasicBlock pb(_binary, _loader->cs(), 
                    (BasicBlock) { _basic_block->start_addr(), _basic_block->end_addr() });
                for (auto i: *pb.insns()) {
                    fprintf(fp, "            ");
                    i.print(fp);
                }
            } else {
                fprintf(fp, "trampoline cannot fit\n");
            }
            fprintf(fp, "\n");
            fclose(fp);
        } else {
            printf("ERROR | khost: failed to write file: %s\n", res_path.c_str());
            exit(-1);
        }
    }
    return code_buffer;
}

void TrampolineAddition::set_addr(uint32_t addr) {
    _additions[1]->set_addr(addr);
    Addition::set_addr(addr);
}

uint32_t TrampolineAddition::calc_size() {
    _trampoline_bytes = 0;
    for (auto addition: _additions) {
        _trampoline_bytes += addition->size();
    }
    uint32_t size = _trampoline_bytes + MAX_RELOCATE_SIZE + RET_BRANCH_SIZE;
    while (size % 4 != 0) {
        size += 2;
    }
    return size;
}

std::vector<uint8_t> TrampolineAddition::calc_relocate_insns() {
    _trampoline_call_off = -1;
    uint32_t off = 0;
    uint32_t sz = 0;
    uint32_t skip_ninsns = 0;
    std::vector<uint8_t> reloc_code, insn_code;
    for (auto insn: *_basic_block->insns()) {
        off += insn.size();
        if (skip_ninsns > 0) {
            skip_ninsns -= 1;
            continue;
        }
        if (insn.is_it()) {
            sz = 0;
            reloc_code.clear();
            skip_ninsns = insn.it_block_len();
            continue;
        }
        // check whether the instruction can be relocated into the current 
        // code location
        if (!insn.is_relocatable(
            addr() == (uint32_t)-1 ? (uint32_t)-1 : addr() + _trampoline_bytes + reloc_code.size()
        )) {
            // if `insn` is position dependent and cannot be moved into the 
            // trampoline, we reset `sz`
            sz = 0;
            reloc_code.clear();
        } else {
            // otherwise we increase `sz` by the size of `insn`
            sz += insn.size();
            insn_code = insn.relocated_bytes(
                _loader->ks_thumb(),
                addr() + _trampoline_bytes + reloc_code.size()
            );
            reloc_code.insert(reloc_code.end(), insn_code.begin(), insn_code.end());
        }
        if (sz >= CALL_SZ) {
            _trampoline_call_off = off - sz;
            break;
        }
    }
    _trampoline_canfit = _trampoline_call_off != (uint32_t)-1;
    while (reloc_code.size() != MAX_RELOCATE_SIZE) {
        // nop is 00 bf in thumb
        reloc_code.push_back(0x00);
        reloc_code.push_back(0xbf);
    }
    _reloc_sz = sz;
    return reloc_code;
}

void TrampolineAddition::patch_detour() {
    char asm_code[256];
    sprintf(asm_code, "b.w #%ld", (int64_t)addr() - ((int64_t)_basic_block->start_addr() + (int64_t)_trampoline_call_off));
    size_t size, count;
    unsigned char *ks_bytes;
    if (ks_asm(_loader->ks_thumb(), asm_code, 0, &ks_bytes, &size, &count) != KS_ERR_OK) {
        printf("failed to compile detour\n");
        exit(-1);
    }
    
    uint32_t block_offset = _basic_block->start_addr() - CONFIG.firmware_info()->load_base;
    uint8_t *binary_buffer = (uint8_t *)((uint64_t)_binary + block_offset);
    for (uint32_t i = 0; i < _reloc_sz; i++) {
        if (i < size) {
            binary_buffer[_trampoline_call_off + i] = ks_bytes[i];
        } else {
            binary_buffer[_trampoline_call_off + i++] = 0x00;
            binary_buffer[_trampoline_call_off + i] = 0xbf;
        }
    }

    ks_free(ks_bytes);
}

}  // namespace khost