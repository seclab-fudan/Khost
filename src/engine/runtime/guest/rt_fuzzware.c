#include "rt_fuzzware.h"
#include "data_def.h"
#include "rt_data.h"
#include "rt_debug.h"
#include "rt_cpu.h"
#include <string.h>

/// @brief look up table for the fuzzware model
typedef struct FuzzwareModelLUT {
    uint32_t passthrough_value;
    uint32_t init_value;
    uint16_t model_cnt;
    int16_t model_list[MAX_FUZZWARE_LUT_PRE_ADDR_SIZE];
} FuzzwareModelLUT;

/// @brief count of the item in fuzzware model look up table
static uint32_t fuzzware_model_lut_cnt;

/// @brief fuzzware model look up table
static FuzzwareModelLUT fuzzware_model_lut_list[RUNTIME_FUZZWARE_MODEL_CNT];

/// @brief read value from a buffer
/// @param buffer address of the read
/// @param offset offset to read
/// @param len length of the read
/// @return read result
static inline uint32_t read_from_buffer(uint8_t *buffer, uint32_t offset, uint32_t len) {
    switch (len) {
    case 1:  return ((uint8_t*)(buffer + offset))[0];
    case 2:  return ((uint16_t*)(buffer + offset))[0];
    case 4:  return ((uint32_t*)(buffer + offset))[0];
    default: return 0x0;
    }
}
static inline uint32_t read_from_buffer(uint8_t *, uint32_t, uint32_t) __attribute__((always_inline));

/// @brief write value to buffer
/// @param buffer address of the write
/// @param val write value
/// @param offset offset to write
/// @param len length of the write
static inline void write_to_buffer(uint8_t *buffer, uint32_t val, uint32_t offset, uint32_t len) {
    switch (len) {
    case 1:  ((uint8_t*)(buffer + offset))[0] = (val & 0xff); return;
    case 2:  ((uint16_t*)(buffer + offset))[0] = (val & 0xffff); return;
    case 4:  ((uint32_t*)(buffer + offset))[0] = (val & 0xffffffff); return;
    default: return;
    }
}
static inline void write_to_buffer(uint8_t *, uint32_t, uint32_t, uint32_t) __attribute__((always_inline));

/// @brief init the lookup table for sepecific model
/// @param idx index into the fuzzware model list
void fuzzware_add_model(int idx) {
    FuzzwareModelInfo *model_info = &RUNTIME_fuzzware_model_list[idx];
    // check if the address is already in look up table
    uint32_t base_addr = model_info->addr & 0xfffffffc;
    uint32_t offset = model_info->addr - base_addr;
    // check if the model is valid
    // we skip all invalid mmio models
    if (model_info->access_size > 0x4 - offset) {
#if KVM_OPEN_DEBUG
        if (unlikely(should_output(RT_OUTPUT_FUZZWARE))) {
            debug_clear();
            debug_append_str("[guest-fuzzware] access accross the align, invalid model ");
            debug_append_int(model_info->addr);
            debug_append_str("\n");
            debug_print();
        }
#endif
        return;
    }
    FuzzwareModelLUT *model_lut = (FuzzwareModelLUT *)(*(uint32_t *)base_addr);
    if (model_lut == 0) {
        if (fuzzware_model_lut_cnt >= RUNTIME_FUZZWARE_MODEL_CNT) {
#if KVM_OPEN_DEBUG
            debug_clear();
            debug_append_str("[guest-fuzzware] failed to alloc new fuzzware lut\n");
            debug_print();
#endif
            runtime_abort();
        }
        model_lut = &fuzzware_model_lut_list[fuzzware_model_lut_cnt++];
        model_lut->model_cnt = 0;
        *(uint32_t *)(base_addr) = (uint32_t)model_lut;
    }
    // check conflict fuzzware model
    for (int i = 0; i < model_lut->model_cnt; i++) {
        int16_t id = model_lut->model_list[i];
        FuzzwareModelInfo *other_model = &RUNTIME_fuzzware_model_list[id];
        if (other_model->pc == model_info->pc
            && other_model->addr == model_info->addr
            && other_model->access_size == model_info->access_size
        ) {
            // always use the newer one
            model_lut->model_list[i] = idx;
            goto ResetPassthrough;
        }
    }
    // set up look up table item
    if (model_lut->model_cnt >= MAX_FUZZWARE_LUT_PRE_ADDR_SIZE) {
#if KVM_OPEN_DEBUG
        debug_clear();
        debug_append_str("[guest-fuzzware] failed to alloc lut model item for 0x");
        debug_append_int(base_addr);
        debug_append_str("\n");
        debug_print();
#endif
		runtime_abort();
    }
    model_lut->model_list[model_lut->model_cnt++] = idx;
ResetPassthrough:
    // reset the passthrough value
    if (model_info->type == FUZZWARE_PASSTHROUGH) {
        write_to_buffer((uint8_t *)&model_lut->init_value, model_info->init_val, offset, model_info->access_size);
        model_lut->passthrough_value = model_lut->init_value;
    }
}

/// @brief init the look up table of the fuzzware
void fuzzware_reset() {
    if (!RUNTIME_use_fuzzware) {
        return;
    }
    if (RUNTIME_first_run) {
        // .bss is not zeroed at start
        fuzzware_model_lut_cnt = 0;
        for (int i = 0; i < RUNTIME_fuzzware_model_cnt; i++) {
            fuzzware_add_model(i);
        }
    } else {
        // only reset the passthrough value
        for (int i = 0; i < fuzzware_model_lut_cnt; i++) {
            fuzzware_model_lut_list[i].passthrough_value = fuzzware_model_lut_list[i].init_value;
        }
        for (int i = RUNTIME_fuzzware_model_new_start; i < RUNTIME_fuzzware_model_cnt; i++) {
            fuzzware_add_model(i);
        }
    }
    RUNTIME_fuzzware_model_new_start = RUNTIME_fuzzware_model_cnt;
}

/// @brief get fuzzware model info by access address and pc
/// @param addr access address
/// @param pc access pc
/// @return pointer to fuzzware model info
static inline FuzzwareModelInfo *fuzzware_get_model(uint32_t addr, uint32_t pc) {
    FuzzwareModelLUT *model_lut = (FuzzwareModelLUT *)(*(uint32_t *)(addr & 0xfffffffc));
    if (model_lut == 0) {
        return 0;
    }
    for (int i = 0; i < model_lut->model_cnt; i++) {
        int16_t model_id = model_lut->model_list[i];
        FuzzwareModelInfo *model = &RUNTIME_fuzzware_model_list[model_id];
        if (model->addr == addr && model->pc == pc) {
            return model;
        }
    }
    return 0;
}
static inline FuzzwareModelInfo *fuzzware_get_model(uint32_t, uint32_t) __attribute__((always_inline));

/// @brief read passthrough value of the address 
/// @param addr access address
/// @param offset offset to read
/// @param len length to read
/// @return value of the passthrough 
static inline uint32_t fuzzware_read_passthrough(uint32_t addr, uint32_t offset, uint32_t len) {
    FuzzwareModelLUT *model_lut = (FuzzwareModelLUT *)(*(uint32_t *)(addr));
    if (model_lut == 0) {
        return 0;
    }
    return read_from_buffer((uint8_t *)&model_lut->passthrough_value, offset, len);
}
static inline uint32_t fuzzware_read_passthrough(uint32_t, uint32_t, uint32_t) __attribute__((always_inline));

/// @brief write passthrough value of the address
/// @param addr access address
/// @param value value to write
/// @param offset offset to write
/// @param len length to write
static inline void fuzzware_write_passthrough(uint32_t addr, uint32_t value, uint32_t offset, uint32_t len) {
    FuzzwareModelLUT *model_lut = (FuzzwareModelLUT *)(*(uint32_t *)(addr));
    if (model_lut == 0) {
        return;
    }
    write_to_buffer((uint8_t *)&model_lut->passthrough_value, value, offset, len);
}
static inline void fuzzware_write_passthrough(uint32_t, uint32_t, uint32_t, uint32_t) __attribute__((always_inline));

/// @brief read value from input file
/// @param size size to read
/// @param left_shift left shift bits for the value
/// @return result of the read
static uint32_t read_from_file(uint32_t size, uint32_t left_shift) {
    uint32_t result = 0;

    // valid read
    if (size && RUNTIME_input_cur + size < RUNTIME_input_len) {
        memcpy(&result, &RUNTIME_input_buffer[RUNTIME_input_cur], size);
        RUNTIME_input_cur += size;
        return result << left_shift;
    } 

    // invalid read
#if KVM_OPEN_DEBUG
    if (unlikely(should_output(RT_OUTPUT_FUZZWARE))) {
        debug_clear();
        debug_append_str("[guest-fuzzware] ran out of input\n");
        debug_print();
    }
#endif
    guest_exit();
    return 0;  // never reach here
}

/// @brief read set index from file
/// @param total size of the set
/// @return value of the set index
static inline uint32_t read_set_idx_from_file(uint32_t total) {
    if (total == 1) {
        return 0;
    }
    return read_from_file(1, 0) % total;
}
static inline uint32_t read_set_idx_from_file(uint32_t) __attribute__((always_inline));

/// @brief handle mmio request in fuzzware memory region
/// @param phys_addr access addr
/// @param pc access pc
/// @param data access data ptr
/// @param len access length
void fuzzware_handle_mmio_write(uint32_t phys_addr, uint32_t pc, uint32_t *data, uint32_t len) {
    uint32_t align_addr = phys_addr & 0xfffffffc;
    uint32_t offset = phys_addr - align_addr;
    
    if (!RUNTIME_use_fuzzware) {
        return;
    }

    fuzzware_write_passthrough(
        align_addr, 
        read_from_buffer((uint8_t *)data, 0, len), 
        offset, 
        len
    );

#if KVM_OPEN_DEBUG
    if (unlikely(should_output(RT_OUTPUT_FUZZWARE))) {
        debug_clear();
        debug_append_str("[guest-fuzzware] ");
        debug_append_int(pc);
        debug_append_str(" ");
        debug_append_int(RUNTIME_firmware_context.lr);
        debug_append_str(" w ");
        debug_append_int(len);
        debug_append_str(" ");
        debug_append_int(RUNTIME_input_cur);
        debug_append_str(" ");
        debug_append_int(phys_addr);
        debug_append_str(":");
        debug_append_int(read_from_buffer((uint8_t *)data, 0, len));
        debug_append_str(" (passthrough)\n");
        debug_print();
    }
#endif
}

/// @brief handle mmio read request in fuzzware memory region
/// @param phys_addr access addr
/// @param pc access pc
/// @param data access data ptr
/// @param len access length
void fuzzware_handle_mmio_read(uint32_t phys_addr, uint32_t pc, uint32_t *data, uint32_t len) {
    uint32_t result, pos;
    FuzzwareModelInfo default_model;
    default_model.type = FUZZWARE_IDENTITY;

    if (!RUNTIME_use_fuzzware) {
        *data = 0x0;
        return;
    }

    // find fuzzware model
    FuzzwareModelInfo *model = fuzzware_get_model(phys_addr, pc);
    if (model == 0) {
		// unlike mmio address, just exit
		uint32_t off = (phys_addr >> 24) & 0x0f;
		// Due to the limitation of fuzzware, 
		// address 8000922 always be mis-identified as a mmio access point due to the overflow, 
		// so we just skip it to reduce false positive
        if ( off > 0x4 || pc == 0x8000922) {
#if KVM_OPEN_DEBUG
        	if (unlikely(should_output(RT_OUTPUT_FUZZWARE))) {
        	    debug_clear();
        	    debug_append_str("[guest-fuzzware] unlike a valid mmio address = 0x");
        	    debug_append_int(phys_addr);
        	    debug_append_str(", pc = 0x");
        	    debug_append_int(pc);
        	    debug_append_str(", more likely an overflow happens !");
        	    debug_append_str("\n");
        	    debug_print();
        	}
			guest_exit();
#endif
		}
#if KVM_OPEN_DEBUG
        if (unlikely(should_output(RT_OUTPUT_FUZZWARE))) {
            debug_clear();
            debug_append_str("[guest-fuzzware] model not found: addr = 0x");
            debug_append_int(phys_addr);
            debug_append_str(", pc = 0x");
            debug_append_int(pc);
            debug_append_str("\n");
            debug_print();
        }
#endif
        // WAY1: treat as default
        // WAY2: exit the guest and use fuzzware to generate mmio model
        asm("bkpt #0xc"); 
        for (int i = RUNTIME_fuzzware_model_new_start; i < RUNTIME_fuzzware_model_cnt; i++) {
            fuzzware_add_model(i);
        }
        model = fuzzware_get_model(phys_addr, pc);
        if (model == 0) {
            if (RUNTIME_fuzzware_model_cnt >= RUNTIME_FUZZWARE_MODEL_CNT) {
#if KVM_OPEN_DEBUG
                debug_clear();
                debug_append_str("[guest-fuzzware] failed to alloc new fuzzware model\n");
                debug_print();
#endif
                runtime_abort();
            }
            RUNTIME_fuzzware_model_list[RUNTIME_fuzzware_model_cnt].addr = phys_addr;
            RUNTIME_fuzzware_model_list[RUNTIME_fuzzware_model_cnt].pc = pc;
            RUNTIME_fuzzware_model_list[RUNTIME_fuzzware_model_cnt].access_size = len;
            RUNTIME_fuzzware_model_list[RUNTIME_fuzzware_model_cnt].type = FUZZWARE_IDENTITY;
            RUNTIME_fuzzware_model_cnt += 1;
            fuzzware_add_model(RUNTIME_fuzzware_model_cnt - 1);
            model = &default_model;
        }
        RUNTIME_fuzzware_model_new_start = RUNTIME_fuzzware_model_cnt;
    }

    switch (model->type) {
    case FUZZWARE_BITEXTRACT:
        result = read_from_file(model->size, model->left_shift);
#if KVM_OPEN_DEBUG
        if (unlikely(should_output(RT_OUTPUT_FUZZWARE))) {
            debug_clear();
            debug_append_str("[guest-fuzzware] ");
            debug_append_int(pc);
            debug_append_str(" ");
            debug_append_int(RUNTIME_firmware_context.lr);
            debug_append_str(" r ");
            debug_append_int(len);
            debug_append_str(" ");
            debug_append_int(RUNTIME_input_cur);
            debug_append_str(" ");
            debug_append_int(phys_addr);
            debug_append_str(":");
            debug_append_int(result);
            debug_append_str(" (bitextract)\n");
            debug_print();
        }
#endif
        write_to_buffer((uint8_t *)data, result, 0, len);
        fuzzware_write_passthrough(phys_addr & 0xfffffffc, result, phys_addr % 0x4, len);
        return;
    case FUZZWARE_CONSTANT:
        result = model->val;
#if KVM_OPEN_DEBUG
        if (unlikely(should_output(RT_OUTPUT_FUZZWARE))) {
            debug_clear();
            debug_append_str("[guest-fuzzware] ");
            debug_append_int(pc);
            debug_append_str(" ");
            debug_append_int(RUNTIME_firmware_context.lr);
            debug_append_str(" r ");
            debug_append_int(len);
            debug_append_str(" ");
            debug_append_int(RUNTIME_input_cur);
            debug_append_str(" ");
            debug_append_int(phys_addr);
            debug_append_str(":");
            debug_append_int(result);
            debug_append_str(" (constant)\n");
            debug_print();
        }
#endif
        write_to_buffer((uint8_t *)data, result, 0, len);
        fuzzware_write_passthrough(phys_addr & 0xfffffffc, result, phys_addr % 0x4, len);
        return;
    case FUZZWARE_SET:
        pos = read_set_idx_from_file(model->val_cnt);
        result = model->vals[pos];
        write_to_buffer((uint8_t *)data, result, 0, len);
        fuzzware_write_passthrough(phys_addr & 0xfffffffc, result, phys_addr % 0x4, len);
#if KVM_OPEN_DEBUG
        if (unlikely(should_output(RT_OUTPUT_FUZZWARE))) {
            debug_clear();
            debug_append_str("[guest-fuzzware] ");
            debug_append_int(pc);
            debug_append_str(" ");
            debug_append_int(RUNTIME_firmware_context.lr);
            debug_append_str(" r ");
            debug_append_int(len);
            debug_append_str(" ");
            debug_append_int(RUNTIME_input_cur);
            debug_append_str(" ");
            debug_append_int(phys_addr);
            debug_append_str(":");
            debug_append_int(result);
            debug_append_str(" (set)\n");
            debug_print();
        }
#endif
        return;
    case FUZZWARE_PASSTHROUGH:
        result = fuzzware_read_passthrough(phys_addr & 0xfffffffc, phys_addr % 0x4, len);
        write_to_buffer((uint8_t *)data, result, 0, len);
#if KVM_OPEN_DEBUG
        if (unlikely(should_output(RT_OUTPUT_FUZZWARE))) {
            debug_clear();
            debug_append_str("[guest-fuzzware] ");
            debug_append_int(pc);
            debug_append_str(" ");
            debug_append_int(RUNTIME_firmware_context.lr);
            debug_append_str(" r ");
            debug_append_int(len);
            debug_append_str(" ");
            debug_append_int(RUNTIME_input_cur);
            debug_append_str(" ");
            debug_append_int(phys_addr);
            debug_append_str(":");
            debug_append_int(result);
            debug_append_str(" (passthrough)\n");
            debug_print();
        }
#endif
        return;
    default: 
        result = read_from_file(len, 0);
        write_to_buffer((uint8_t *)data, result, 0, len);
        fuzzware_write_passthrough(phys_addr & 0xfffffffc, result, phys_addr % 0x4, len);
#if KVM_OPEN_DEBUG
        if (unlikely(should_output(RT_OUTPUT_FUZZWARE))) {
            debug_clear();
            debug_append_str("[guest-fuzzware] ");
            debug_append_int(pc);
            debug_append_str(" ");
            debug_append_int(RUNTIME_firmware_context.lr);
            debug_append_str(" r ");
            debug_append_int(len);
            debug_append_str(" ");
            debug_append_int(RUNTIME_input_cur);
            debug_append_str(" ");
            debug_append_int(phys_addr);
            debug_append_str(":");
            debug_append_int(result);
            debug_append_str(" (identity)\n");
            debug_print();
        }
#endif
        return;
    }
    return;
}
