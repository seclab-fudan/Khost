#ifndef EN_BOARD_FULL_H
#define EN_BOARD_FULL_H

#if KVM_OPEN_DEBUG
	#include "autogen_rt_full_debug.h"
#else
	#include "autogen_rt_full_normal.h"
#endif
#include "en_board.h"
#include "en_cpu_full.h"

#include <sys/ipc.h>
#include <sys/shm.h>

namespace khost {

/// @brief virtual machine for fully virtualized firmware
class BoardFull : public Board {
public:
    BoardFull();
    virtual ~BoardFull();

    inline CPUFull *cpu() { return _cpu; }
    virtual FirmwareState *firmware_state() override;
    virtual void reset_firmware_state() override;

    // statefile and fuzzware model
    bool create_statefile(std::string file_path);
    bool update_fuzzware_model();
    
    // interact with fuzzer
    uint32_t error_pc() { return 0; }
    void report_coverage();

    // reset and run
    virtual bool reset(uint8_t *buffer, uint32_t size) override;
	bool reset_hooks();
    virtual int run() override;
	uint8_t *rt_buffer = 0;
	uint32_t *rt_size = 0;
    uint32_t *rt_cur = 0;

protected:
    // init operations
    virtual bool init_memory_backends() override;
    virtual bool init_runtime_state() override;
    virtual bool init_vcpu() override;
    virtual bool init_devices() override;

private:
    // memory backends
    void *_on_chip_device_backend = nullptr;
    void *_off_chip_device_backend = nullptr;
    uint8_t *_afl_area_ptr = nullptr;

    CPUFull *_cpu;

    void init_page_table();
    void load_input(uint8_t *buffer, uint32_t size);
    bool init_signal();
};

}  // namespace khost

#endif  // EN_BOARD_FULL_H
