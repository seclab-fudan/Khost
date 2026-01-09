#ifndef ADDITION_H
#define ADDITION_H

#include "en_global.h"
#include "en_config.h"

namespace khost {

class Loader;

// Addition: base class for inserted code parts
class Addition {
public:
    Addition(Loader *loader) : _loader(loader) {}
    virtual ~Addition() {}

    // return the size need for current pass
    inline uint32_t size() {
        return _cached_size != (uint32_t)-1 ? 
               _cached_size : (_cached_size = calc_size());
    }
    // return address of the addition
    inline uint32_t addr() {
        return _addr;
    }
    // return the code for current pass
    virtual void *bytecode() = 0;
    // set the addr of the addition
    virtual void set_addr(uint32_t addr) { _addr = addr; }

protected:
    inline uint32_t word_low(uint32_t num) { return num & 0xffff; }
    inline uint32_t word_high(uint32_t num) { return (num >> 16) & 0xffff; }
    Loader *_loader;

private:
    virtual uint32_t calc_size() = 0;
    uint32_t _cached_size = -1;
    uint32_t _addr = -1;
};

}  // namespace khost

#endif  // ADDITION_H