#ifndef DEV_FUZZWARE_MODEL_H
#define DEV_FUZZWARE_MODEL_H

#include <vector>
#include <map>
#include "dev_mmio.h"
#include "en_config.h"

namespace khost {

class FuzzwareModel : public MMIORegion {
public:
    FuzzwareModel();
    ~FuzzwareModel();

    int handle_read(uint64_t phys_addr, uint8_t *data, uint32_t len, uint32_t pc);
    int handle_write(uint64_t phys_addr, uint8_t *data, uint32_t len, uint32_t pc);

    bool reset(uint8_t *buffer, uint32_t size) {
        _buffer = buffer;
        _size = size;
        _cur = 0;
        _passthrough.clear();
        return true;
    }

    uint32_t read_from_file(uint32_t size, uint32_t left_shift, bool *ok);
    uint32_t read_set_pos_from_file(uint32_t total, bool *ok);

private:
    std::map<uint32_t, uint32_t> _passthrough;
    uint8_t *_buffer;
    uint32_t _size;
    uint32_t _cur;

    void error(const char *s, ...);
    void info(const char *s, ...);
};

}  // namespace khost

#endif  // DEV_FUZZWARE_MODEL