#include "dev_mmio.h"

namespace khost {

int MMIOModel::handle_read(uint64_t phys_addr, uint8_t *data, uint32_t len, uint32_t pc) {
#if KVM_OPEN_DEBUG
    info("read 0x%x with length 0x%x [pc=0x%x]", phys_addr, len, pc);
#endif
    return HANDLED;
}

int MMIOModel::handle_write(uint64_t phys_addr, uint8_t *data, uint32_t len, uint32_t pc) {    
#if KVM_OPEN_DEBUG
    info("write 0x%x with length 0x%x [pc=0x%x]", phys_addr, len, pc);
#endif
    return HANDLED;
}

/// @brief log info message
/// @param s info message
void MMIOModel::info(const char *s, ...) { 
    if (unlikely(CONFIG.option_should_target_output(EN_OUTPUT_BOARD))) {
        if (CONFIG.option_output_level() <= OUTPUT_LEVEL_INFO) {
            char buffer[256];
            va_list args;
            va_start(args, s);
            vsprintf(buffer, s, args);
            va_end(args); 
            print_mutex.lock();
            printf("INFO  | host-mmio: %s\n", buffer);
            print_mutex.unlock();
        }
    }
}

}  // namespace khost
