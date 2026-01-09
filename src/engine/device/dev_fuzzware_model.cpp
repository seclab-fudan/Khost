#include "dev_fuzzware_model.h"

#include <yaml-cpp/yaml.h>

namespace khost {

static uint32_t read_from_buffer(uint8_t *buffer, uint32_t len) {
    if (len == 1)
        return ((uint8_t*)buffer)[0];
    if (len == 2)
        return ((uint16_t*)buffer)[0];
    if (len == 4)
        return ((uint32_t*)buffer)[0];
    return 0x0;
}

static void write_to_buffer(uint8_t *buffer, uint32_t len, uint32_t val) {
    if (len == 1)
        ((uint8_t*)buffer)[0] = (val & 0xff);
    if (len == 2)
        ((uint16_t*)buffer)[0] = (val & 0xffff);
    if (len == 4)
        ((uint32_t*)buffer)[0] = (val & 0xffffffff);
}

uint32_t FuzzwareModel::read_from_file(uint32_t size, uint32_t left_shift, bool *ok = nullptr) {
    bool ran_out_of_input = false;
    uint64_t result = 0;

    if (size && _cur + size < _size) {
        // valid read
        memcpy(&result, &_buffer[_cur], size);
#if KVM_OPEN_DEBUG
        info("read %d bytes from offset %d", size, (int)_cur);
#endif
        _cur += size;
    } else {
        // invalid read
        ran_out_of_input = true;
#if KVM_OPEN_DEBUG
        info("used all input up, exiting...");
#endif
        result = rand();
    }

    if (ok) {
        *ok = !ran_out_of_input;
    }
    return result << left_shift;
}

uint32_t FuzzwareModel::read_set_pos_from_file(uint32_t total, bool *ok = nullptr) {
    if (total == 1) {
        if (ok) {
            *ok = true;
        }
        return 0;
    }
    return read_from_file(1, 0, ok) % total;
}

FuzzwareModel::FuzzwareModel() {
    _passthrough.clear();
}

FuzzwareModel::~FuzzwareModel() {

}

int FuzzwareModel::handle_read(uint64_t phys_addr, uint8_t *data, uint32_t len, 
                                uint32_t pc) {
    uint32_t type;
    uint32_t result, pos;
    std::map<uint32_t, uint32_t>::iterator memory_iter;
    FuzzwareModelInfo default_model;
    default_model.init_val = 0;
    default_model.type = FUZZWARE_IDENTITY;
    bool ok = false;
    
    // find fuzzware model
    FuzzwareModelInfo *model = CONFIG.get_model(phys_addr, pc);
    if (model == nullptr) {
#if KVM_OPEN_DEBUG
        error("addr hasn't been modeled: read 0x%x with length 0x%x [pc=0x%x]", phys_addr, len, pc);
#endif
        type = FUZZWARE_IDENTITY;    
        model = &default_model;
    } else {
        type = model->type; 
    }

    switch (type) {
    case FUZZWARE_BITEXTRACT:
        result = read_from_file(model->size, model->left_shift, &ok);
#if KVM_OPEN_DEBUG
        info("[BITEXTRACT] read 0x%x with length 0x%x, mask=0x%08x, size=%d, left_shift=%d, ret 0x%x [pc=0x%x]", 
                phys_addr, len, model->mask, model->size, model->left_shift, result, pc);
#endif
        write_to_buffer(data, len, result);
        _passthrough[phys_addr] = result;
        return ok ? HANDLED : EXIT_OK;
    case FUZZWARE_CONSTANT:
        result = model->val;
#if KVM_OPEN_DEBUG
        info("[CONSTANT] read 0x%x with length 0x%x, ret 0x%x [pc=0x%x]", phys_addr, len, result, pc);
#endif
        write_to_buffer(data, len, result);
        _passthrough[phys_addr] = result;
        return HANDLED;
    case FUZZWARE_SET:
        pos = read_set_pos_from_file(model->val_cnt, &ok);
        result = model->vals[pos];
        write_to_buffer(data, len, result);
        _passthrough[phys_addr] = result;
#if KVM_OPEN_DEBUG
        info("[SET] read 0x%x with length 0x%x, id 0x%x, ret 0x%x [pc=0x%x]", phys_addr, len, pos, result, pc);
#endif
        return ok ? HANDLED : EXIT_OK;
    case FUZZWARE_PASSTHROUGH:
        memory_iter = _passthrough.find(phys_addr);
        if (memory_iter == _passthrough.end()) {
            result = model->init_val;
            _passthrough[phys_addr] = model->init_val;
        } else {
            result = memory_iter->second;
        }
        write_to_buffer(data, len, result);
#if KVM_OPEN_DEBUG
        info("[PASSTHROUGH] read 0x%x with length 0x%x, ret 0x%x [pc=0x%x]", phys_addr, len, result, pc);
#endif
        return HANDLED;
    default: 
        result = read_from_file(len, 0, &ok);
        write_to_buffer(data, len, result);
        _passthrough[phys_addr] = result;
#if KVM_OPEN_DEBUG
        info("[IDENTITY] read 0x%x with length 0x%x, ret 0x%x [pc=0x%x]", phys_addr, len, result, pc);
#endif
        return ok ? HANDLED : EXIT_OK;
    }
}

int FuzzwareModel::handle_write(uint64_t phys_addr, uint8_t *data, 
                                 uint32_t len, uint32_t pc) {
#if KVM_OPEN_DEBUG
    info("[PASSTHROUGH] write 0x%x with length 0x%x [pc=0x%x]", phys_addr, len, pc);
#endif
    _passthrough[phys_addr] = read_from_buffer(data, len);
    return HANDLED;
}

/// @brief log error message
/// @param s error message
void FuzzwareModel::error(const char *s, ...) { 
    if (unlikely(CONFIG.option_should_target_output(EN_OUTPUT_FUZZWARE))) {
        if (CONFIG.option_output_level() <= OUTPUT_LEVEL_ERROR) {
            char buffer[256];
            va_list args;
            va_start(args, s);
            vsprintf(buffer, s, args);
            va_end(args); 
            print_mutex.lock();
            printf("ERROR | host-fuzzware: %s\n", buffer);
            print_mutex.unlock();
        }
    }
}

/// @brief log info message
/// @param s info message
void FuzzwareModel::info(const char *s, ...) { 
    if (unlikely(CONFIG.option_should_target_output(EN_OUTPUT_FUZZWARE))) {
        if (CONFIG.option_output_level() <= OUTPUT_LEVEL_INFO) {
            char buffer[256];
            va_list args;
            va_start(args, s);
            vsprintf(buffer, s, args);
            va_end(args); 
            print_mutex.lock();
            printf("INFO  | host-fuzzware: %s\n", buffer);
            print_mutex.unlock();
        }
    }
}

}  // namespace khost
