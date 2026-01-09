#ifndef TRAMPOLINE_H
#define TRAMPOLINE_H

#include "en_global.h"
#include "en_config.h"
#include "addition.h"
#include "disasm.h"
#include "coverage.h"

#include <vector>

#define CALL_SZ             4
#define MAX_RELOCATE_SIZE   8
#define RET_BRANCH_SIZE     8

namespace khost {

class loader;

class PrologueAddition : public Addition {
public:
    PrologueAddition(Loader *loader) : Addition(loader) {}
    virtual ~PrologueAddition() {}
    virtual void *bytecode() override final;
    virtual void set_addr(uint32_t addr) override final;
private:
    virtual uint32_t calc_size() override final;
};

class EpilogueAddition : public Addition {
    public:
    EpilogueAddition(Loader *loader) : Addition(loader) {}
    virtual ~EpilogueAddition() {}
    virtual void *bytecode() override final;
    virtual void set_addr(uint32_t addr) override final;
private:
    virtual uint32_t calc_size() override final;
};

class TrampolineAddition : public Addition {
public:
    TrampolineAddition(Loader *loader, void *binary, BasicBlock basic_block);
    virtual ~TrampolineAddition();
    virtual void *bytecode() override final;
    virtual void set_addr(uint32_t addr) override final;
private:
    virtual uint32_t calc_size() override final; 

    void *_binary;
    uint32_t _detour_offset;
    DisBasicBlock *_basic_block;

    bool _trampoline_canfit;
    uint32_t _reloc_sz;
    uint32_t _trampoline_bytes;
    uint32_t _trampoline_call_off;

    std::vector<Addition *> _additions;

    std::vector<uint8_t> calc_relocate_insns();
    void patch_detour();
};

}  // namespace khost

#endif  // TRAMPOLINE_H