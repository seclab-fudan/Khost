#ifndef DEV_MMIO_H
#define DEV_MMIO_H

#include "en_global.h"
#include "en_config.h"

namespace khost {

class MMIORegion {
public:
    MMIORegion(uint32_t base_addr = 0) : _base_addr(base_addr) {}
    virtual ~MMIORegion() {};

    virtual int handle_read(uint64_t phys_addr, uint8_t *data, uint32_t len, uint32_t pc) = 0;
    virtual int handle_write(uint64_t phys_addr, uint8_t *data, uint32_t len, uint32_t pc) = 0;

    uint32_t base_addr() const { return _base_addr; }
private:
    uint32_t _base_addr;
};

struct MMIORegionDes {
    uint32_t l;
    MMIORegion *h;

    MMIORegionDes(uint32_t _len = 0, MMIORegion *_handler = nullptr) 
        : l(_len), h(_handler) {}
};

class MMIOModel : public MMIORegion {
public:
    MMIOModel(uint32_t base_addr = 0, const char *name = "0") 
        : MMIORegion(base_addr), _name(name) {}
    ~MMIOModel() {}

    int handle_read(uint64_t phys_addr, uint8_t *data, uint32_t len, uint32_t pc);
    int handle_write(uint64_t phys_addr, uint8_t *data, uint32_t len, uint32_t pc);

private:
    const char *_name;

    void info(const char *s, ...);
};

}  // namespace khost

#endif  // DEV_MMIO_H