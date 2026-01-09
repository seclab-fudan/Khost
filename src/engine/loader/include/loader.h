#ifndef LOADER_H
#define LOADER_H

#include <string.h>
#include <string>
#include <vector>
#include "en_global.h"
#include "addition.h"
#include "trampoline.h"
#include "keystone/keystone.h"
#include "capstone/capstone.h"

namespace khost {

class Addition;

class Loader {
public:
    Loader();
    ~Loader();

    void load();

    inline ks_engine *ks_arm() { return _ks_arm; }
    inline ks_engine *ks_thumb() { return _ks_thumb; }
    inline csh cs() { return _cs; }

private:
    ks_engine *_ks_arm;
    ks_engine *_ks_thumb;
    csh _cs;

    void open_engine();
    void close_engine();

    uint32_t calc_load_base(void *buffer, ssize_t size);
    uint32_t calc_addition_base(uint32_t code_base, uint32_t code_size, 
                                uint32_t addition_size);
    
    void find_and_patch_incompatible_insns(void *buffer, ssize_t size);

    std::vector<Addition *> _additions;
};

}  // namespace khost

#endif  // LOADER_H