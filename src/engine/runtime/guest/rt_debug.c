#include "rt_debug.h"
#include "rt_data.h"

__attribute__((section(".rt_dmz_handler")))
/// @brief convert interger to hex string
/// @param val interger value
/// @return hex string
static inline char to_hex(uint32_t val) {
    if (val < 10) {
        return '0' + val;
    } else {
        return 'A' + val - 10;
    }
}
static inline char to_hex(uint32_t) __attribute__((always_inline));

__attribute__((section(".rt_dmz_handler")))
/// @brief clear debug information buffer
inline void debug_clear() {
    RUNTIME_firmware_state.debug_buffer[0] = 0;
}
inline void debug_clear() __attribute__((always_inline));

__attribute__((section(".rt_dmz_handler")))
/// @brief write uint32_t value to debug buffer
/// @param val value of the uint32_t
void debug_append_int(uint32_t val) {
    // find possible place to print
    int cnt = 0;
    for (; cnt < 0x100; cnt++) {
        if (RUNTIME_firmware_state.debug_buffer[cnt] == 0) {
            break;
        }
    }
    if (cnt == 0xff) {
        return;
    }

    // add value to the buffer
    for (int i = 7; i >= 0; i --) {
        RUNTIME_firmware_state.debug_buffer[cnt++] = to_hex((val >> (i*4)) & 0xf);
        if (cnt == 0x100) {
            cnt = 0xff;
            break;
        }
    }
    RUNTIME_firmware_state.debug_buffer[cnt] = 0;    
}

__attribute__((section(".rt_dmz_handler")))
/// @brief write debug output to debug buffer 
/// @param s string of the information
void debug_append_str(const char *s) { 
    // find possible place to print
    int cnt = 0;
    for (; cnt < 0x100; cnt++) {
        if (RUNTIME_firmware_state.debug_buffer[cnt] == 0) {
            break;
        }
    }
    if (cnt == 0xff) {
        return;
    }

    // add string to the buffer
    for (const char *c = s; *c && cnt <= 0xff; c++) {
        RUNTIME_firmware_state.debug_buffer[cnt++] = *c;
        if (cnt == 0x100) {
            cnt = 0xff;
            break;
        }
    }
    RUNTIME_firmware_state.debug_buffer[cnt] = 0;
}

__attribute__((section(".rt_dmz_handler")))
/// @brief use bkpt instruction to notify host to print the information out
inline void debug_print() {
    asm("isb");
    asm("dsb");
    asm("bkpt #6");
}
inline void debug_print() __attribute__((always_inline));

__attribute__((section(".rt_dmz_handler")))
/// @brief return if the component should print message
/// @return 1 for true and 0 for false
inline int should_output(int target) {
    return RUNTIME_output_option & target;
}
inline int should_output(int) __attribute__((always_inline));
