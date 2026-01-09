#ifndef EN_BOARD_H
#define EN_BOARD_H

#include "en_global.h"
#include "en_config.h"
#include "en_cpu.h"

extern "C" {
    #include "data_def.h"
}

#include <linux/mman.h>
#include <vector>

namespace khost {

/// @brief base virtual machine for khost
class Board {
public:
    Board();
    virtual ~Board();
    bool init();

    // states and run
    inline bool first_run() { return _first_run; }
    virtual FirmwareState *firmware_state() = 0;
    virtual void reset_firmware_state() = 0;
    void flush_firmware_state();

    virtual bool reset(uint8_t *buffer, uint32_t size) = 0;
    virtual int run() = 0;

    // descriptors
    inline int sys_fd() { return _sys_fd; }
    inline int vm_fd() { return _vm_fd; }
    int check_extension(unsigned int extension);

    // cpu and interrupt
    bool irq_trigger(int irq);
    bool irq_line(int irq, int level);
    bool gic_write_dist_reg(uint64_t offset, uint64_t value);
    bool gic_write_cpu_reg(uint64_t offset, uint64_t value);
    uint32_t gic_read_cpu_reg(uint64_t offset);

    // memory operations
    void* gpa_to_hva(uint32_t gpa, uint32_t *remain = nullptr);
    uint8_t read_u8(uint32_t gpa);
    uint16_t read_u16(uint32_t gpa);
    uint32_t read_u32(uint32_t gpa);
    void write_u8(uint32_t gpa, uint8_t val, bool flush = true);
    void write_u16(uint32_t gpa, uint16_t val, bool flush = true);
    void write_u32(uint32_t gpa, uint32_t val, bool flush = true);
    bool gpa_to_hva_memcpy(void *hva, uint32_t gpa, uint32_t buf_size);
    bool hva_to_gpa_memcpy(uint32_t gpa, void *hva, uint32_t buf_size);
    void flush_memory(uint32_t gpa, uint32_t size);

    // coverage collection
    uint8_t *coverage_map();
    uint8_t *total_coverage_map() { return _total_coverage_buffer; }
    std::map<uint32_t, std::string> *hit_coverage_map();
    std::map<uint32_t, std::string> *new_hit_coverage_map();
    void set_coverage_ground_truth(std::vector<uint32_t> ground_truth);
    void set_coverage_map_with_pc(uint32_t pc); 
    void reset_coverage_bkpt(uint32_t pc);

protected:
    // states
    bool _first_run = true;
    
    // descriptors
    int  _sys_fd = -1;          // kvm system descriptor
    int   _vm_fd = -1;          // kvm virtual machine descriptor
    int  _gic_fd = -1;          // gic controller fd
    
    bool init_descriptors();
    
    // memory backends
    void *_rom_or_flash_backend = nullptr;
    void *_sram_backend         = nullptr;
    void *_wbwa_ram_backend     = nullptr;
    void *_wt_ram_backend       = nullptr;
    void *_runtime_backend      = nullptr;
    std::vector<int> _kvm_slots;

    virtual bool init_memory_backends();
    bool submit_memory_region(int slot, int flags, uint32_t gpa, void *backend, 
                              uint32_t size);
    bool cancel_memory_region(int slot);
    bool load_memory_regions();

    // cpus and interrupts
    virtual bool init_vcpu();
    bool init_gic();
    bool gic_create_irqchip();

    // devices
    virtual bool init_devices();
 
    // runtime state
    virtual bool init_runtime_state();

    // coverage collection
    uint8_t _coverage_buffer[COVERAGE_MAP_SIZE];
    uint8_t _total_coverage_buffer[COVERAGE_MAP_SIZE];
    std::map<uint32_t, std::string> _hit_coverage_map;
    std::map<uint32_t, std::string> _new_hit_coverage_map;
    std::map<uint32_t, std::pair<int, uint32_t>> _coverage_bpkts;
    bool init_coverage_buffer();
    void set_coverage_bkpts();
    
    // output functions
    void error(const char *s, ...);
    void info(const char *s, ...);
    void debug(const char *s, ...);
};

}  // namespace khost

#endif  // EN_BOARD_H