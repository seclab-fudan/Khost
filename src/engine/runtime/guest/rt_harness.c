#include "rt_global.h"
#include "rt_data.h"
#include "rt_debug.h"
#include "string.h"

__attribute__((target("thumb")))
__attribute__((section(".rt_dmz_handler")))
uint32_t HAL_return_zero() {
#if KVM_OPEN_DEBUG
	if (unlikely(should_output(RT_OUTPUT_HANDLER))) {
    	debug_clear();
    	debug_append_str("enter HAL return zero\n");
    	debug_print();
	}
#endif
    return 0;
}

__attribute__((target("thumb")))
__attribute__((section(".rt_dmz_handler")))
uint32_t HAL_return_one() {
#if KVM_OPEN_DEBUG
	if (unlikely(should_output(RT_OUTPUT_HANDLER))) {
    	debug_clear();
    	debug_append_str("enter HAL return one\n");
    	debug_print();
	}
#endif
    return 1;
}

__attribute__((target("thumb")))
__attribute__((section(".rt_dmz_handler")))
void HAL_return_void() {
#if KVM_OPEN_DEBUG
	if (unlikely(should_output(RT_OUTPUT_HANDLER))) {
	    debug_clear();
	    debug_append_str("enter HAL return void\n");
	    debug_print();
	}
#endif
    return;
}

__attribute__((target("thumb")))
__attribute__((section(".rt_dmz_handler")))
uint32_t HAL_stm32_get_tick() {
#if KVM_OPEN_DEBUG
	if (unlikely(should_output(RT_OUTPUT_HANDLER))) {
	    debug_clear();
	    debug_append_str("enter HAL get tick\n");
	    debug_print();
	}
#endif
    RUNTIME_firmware_state.block_count += 10000;
    return RUNTIME_firmware_state.block_count;
}

__attribute__((target("thumb")))
__attribute__((section(".rt_dmz_handler")))
void HAL_stm32_trigger_tick(uint32_t delay) {
#if KVM_OPEN_DEBUG
	if (unlikely(should_output(RT_OUTPUT_HANDLER))) {
    	debug_clear();
    	debug_append_str("enter HAL trigger tick\n");
    	debug_print();
	}
#endif
    asm("svc #0xff");
    return;
}

__attribute__((target("thumb")))
__attribute__((section(".rt_dmz_handler")))
/// @brief hal hook for read file
/// @param file_ptr pointer to the file
/// @param buf_ptr pointer to the buffer
/// @param sizeofbuf size of the buffer
/// @return bytes read
uint32_t HAL_stm32_read_file(uint32_t file_ptr, uint32_t buf_ptr, uint32_t sizeofbuf) {
#if PROTECT_RUNTIME
    asm volatile (
        "mov     r0, #0xf\n\t"
        "mcr     p15, 0, r0, c3, c0, 0\n\t"
        "isb\n\t"
        : : : "r0", "memory"
    );
#endif
    uint32_t ret = 0;
    
    uint32_t remain = RUNTIME_input_len - RUNTIME_input_cur;
    uint32_t n = sizeofbuf < remain ? sizeofbuf: remain;
    if (RUNTIME_input_cur + n <= RUNTIME_input_len) {
        memcpy((void *)buf_ptr, &RUNTIME_input_buffer[RUNTIME_input_cur], n);
        RUNTIME_input_cur += n;
        ret = n;
    }

#if PROTECT_RUNTIME
    asm volatile (
        "mov     r0, #0x1\n\t"
        "mcr     p15, 0, r0, c3, c0, 0\n\t"
        "isb\n\t"
        : : : "r0", "memory"
    );
#endif

#if KVM_OPEN_DEBUG
	if (unlikely(should_output(RT_OUTPUT_HANDLER))) {
    	debug_clear();
    	debug_append_str("enter HAL read_file: file: 0x");
    	debug_append_int(file_ptr);
    	debug_append_str(", ptr:0x");
    	debug_append_int(buf_ptr);
    	debug_append_str(", size=0x");
    	debug_append_int(sizeofbuf);
    	debug_append_str(", ret=0x");
    	debug_append_int(ret);
    	debug_append_str(", remain=0x");
    	debug_append_int(RUNTIME_input_len - RUNTIME_input_cur);
    	debug_append_str("\n");
    	debug_print();
	}
#endif
    return ret;
}
