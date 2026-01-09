#include "en_board.h"

#include <iostream>

namespace khost {

inline static void force_clear_cache(uintptr_t start, uintptr_t end) {
    uint32_t ctr;
    asm("mrs %[ctr], ctr_el0\n\t" : [ctr] "=r" (ctr));

    uintptr_t const dsize = 4 << ((ctr >> 16) & 0xf);
    uintptr_t const isize = 4 << ((ctr >>  0) & 0xf);

    for (uintptr_t dline = start & ~(dsize - 1); dline < end; dline += dsize) {
        asm("dc cvac, %[dline]\n\t" : : [dline] "r" (dline) : "memory");
    }

    asm("dsb ish\n\t" : : : "memory");

    for (uintptr_t iline = start & ~(isize - 1); iline < end; iline += isize) {
        asm("ic ivau, %[iline]\n\t" : : [iline] "r" (iline) : "memory");
    }

    asm("dsb ish\n\t"
        "isb\n\t"
        : : : "memory");
}

// =============================================================================
// construct and destruct operations

/// @brief init system descriptors needed for managing kvm
/// @return true on success, false on fail
bool Board::init_descriptors() {
    _sys_fd = open("/dev/kvm", O_RDWR);
    if (_sys_fd < 0) {
#if KVM_OPEN_DEBUG
        error("failed to create system fd");
#endif
        _sys_fd = -1;
        return false;
    }
    _vm_fd = ioctl(_sys_fd, KVM_CREATE_VM, KVM_VM_TYPE_ARM_IPA_SIZE(32));
    if (_vm_fd < 0) {
#if KVM_OPEN_DEBUG
        error("failed to create virtual machine fd");
#endif
        _vm_fd = -1;
        return false;
    } 
    return true;
}

/// @brief submit specific memory region to kvm
/// @param slot kvm slot region
/// @param flags memory flag
/// @param gpa guest physical address
/// @param backend host virtual memory region
/// @param size size of the memory region
/// @return true on success, false on fail
bool Board::submit_memory_region(int slot, int flags, uint32_t gpa, 
                                 void *backend, uint32_t size) {
    struct kvm_userspace_memory_region mem;
    mem.slot = slot;
    mem.flags = flags;
    mem.guest_phys_addr = (uint64_t)gpa;
    mem.userspace_addr = (uint64_t)backend;
    mem.memory_size = (uint64_t)size;
    int ret = ioctl(_vm_fd, KVM_SET_USER_MEMORY_REGION, &mem);
    if (ret == 0) {
        _kvm_slots.push_back(slot);
    }
    return ret == 0;
}

/// @brief cancel memory region which has id slot
/// @param slot kvm slot region
/// @return true on success, false on fail
bool Board::cancel_memory_region(int slot) {
    struct kvm_userspace_memory_region mem;
    mem.slot = slot;
    mem.memory_size = 0;
    int ret = ioctl(_vm_fd, KVM_SET_USER_MEMORY_REGION, &mem);
    return ret == 0;    
}

/// @brief init memory backend for the virtual machine
/// @return true on success, false on fail
bool Board::init_memory_backends() {
    uint32_t simple_attr = MAP_SHARED | MAP_ANONYMOUS | MAP_POPULATE;
    uint32_t hugepage_attr = simple_attr | MAP_HUGETLB | MAP_HUGE_1GB;

    _rom_or_flash_backend = mmap(
        NULL, ROM_OR_FLASH_SIZE, PROT_READ | PROT_WRITE, 
        CONFIG.option_use_hugepage() ? hugepage_attr : simple_attr, -1, 0
    );
    if (_rom_or_flash_backend == MAP_FAILED) {
#if KVM_OPEN_DEBUG
        error("failed to create memory for rom or flash");
#endif
        _rom_or_flash_backend = nullptr;
        return false;
    }

    _sram_backend = mmap(
        NULL, SRAM_SIZE, PROT_READ | PROT_WRITE,
        CONFIG.option_use_hugepage() ? hugepage_attr : simple_attr, -1, 0
    );
	if (_sram_backend == MAP_FAILED) {
#if KVM_OPEN_DEBUG
        error("failed to create memory for sram");
#endif
        _sram_backend = nullptr;
        return false;
    }

    _wbwa_ram_backend = mmap(
        NULL, WBWA_RAM_SIZE, PROT_READ | PROT_WRITE, simple_attr, -1, 0
    );
    if (_wbwa_ram_backend == MAP_FAILED) {
#if KVM_OPEN_DEBUG
        error("failed to create memory for ram(WBWA)");
#endif
        _wbwa_ram_backend = nullptr;
        return false;
    }

    _wt_ram_backend = mmap(
        NULL, WT_RAM_SIZE, PROT_READ | PROT_WRITE, simple_attr, -1, 0
    );
    if (_wt_ram_backend == MAP_FAILED) {
#if KVM_OPEN_DEBUG
        error("failed to create memory for ram(WT)");
#endif
        _wt_ram_backend = nullptr;
        return false;
    }

    _runtime_backend = mmap(
        NULL, RUNTIME_SIZE, PROT_READ | PROT_WRITE,
        CONFIG.option_use_hugepage() ? hugepage_attr : simple_attr, -1, 0
    );
    if (_runtime_backend == MAP_FAILED) {
#if KVM_OPEN_DEBUG
        error("failed to create memory for runtime");
#endif
        _runtime_backend = nullptr;
        return false;
    }

    if (!submit_memory_region(
        ROM_OR_FLASH_SLOT, ROM_OR_FLASH_FLAGS, 
        ROM_OR_FLASH_START, _rom_or_flash_backend, ROM_OR_FLASH_SIZE
    )) {
#if KVM_OPEN_DEBUG
        error("failed to submit rom memory slot");
#endif
        return false;
    }

    if (!submit_memory_region(
        SRAM_SLOT, SRAM_FLAGS, 
        SRAM_START, _sram_backend, SRAM_SIZE
    )) {
#if KVM_OPEN_DEBUG
        error("failed to submit sram memory slot");
#endif
        return false;
    }

    if (!submit_memory_region(
        WBWA_RAM_SLOT, WBWA_RAM_FLAGS, 
        WBWA_RAM_START, _wbwa_ram_backend, WBWA_RAM_SIZE
    )) {
#if KVM_OPEN_DEBUG
        error("failed to submit ram(WBWA) memory slot");
#endif
        return false;
    }

    if (!submit_memory_region(
        WT_RAM_SLOT, WT_RAM_FLAGS, 
        WT_RAM_START, _wt_ram_backend, WT_RAM_SIZE
    )) {
#if KVM_OPEN_DEBUG
        error("failed to submit ram(WT) memory slot");
#endif
        return false;
    }

    if (!submit_memory_region(
        RUNTIME_SLOT, RUNTIME_FLAGS, 
        RUNTIME_START, _runtime_backend, RUNTIME_SIZE
    )) {
#if KVM_OPEN_DEBUG
        error("failed to submit runtime memory slot");
#endif
        return false;
    }

    return true;
}

/// @brief init devices for mmio handle
/// @return true on success, false on fail
bool Board::init_devices() {
    return false;
}

/// @brief load memory regions to the virtual machine
/// @return true on success, false on fail
bool Board::load_memory_regions() {
    // get load information from global config
    std::vector<LoadInfo *> *load_info = CONFIG.load_info();
    
    // load every memory region
    for (auto info: *load_info) {
        if (!hva_to_gpa_memcpy(
            info->phys_addr, info->source_buffer, info->source_size
        )) {
            return false;
        }
    }
    return true;
}

/// @brief init the coverage buffer
/// @return true on success, false on fail
bool Board::init_coverage_buffer() {
    memset(_coverage_buffer, 0, COVERAGE_MAP_SIZE);
    memset(_total_coverage_buffer, 0, COVERAGE_MAP_SIZE);
    return true;
}

bool Board::init_vcpu() {
    return false;
}

bool Board::init_runtime_state() { 
    return false; 
}

/// @brief constructor for base virtual machine
Board::Board() {}

/// @brief init the state of the board
/// @return true on success, false on fail
bool Board::init() {
#if KVM_OPEN_DEBUG
    info("crafting virtual board...");
#endif
    _kvm_slots.clear();

    if (!init_descriptors()) {
#if KVM_OPEN_DEBUG
        error("failed to initialize kvm descriptors");
#endif
        return false;
    }
    if (!init_memory_backends()) {
#if KVM_OPEN_DEBUG
        error("failed to initialize memory backends");
#endif
        return false;
    }
    if (!init_devices()) {
#if KVM_OPEN_DEBUG
        error("failed to initialize virtual devices");
#endif
        return false;
    }
    if (!this->init_vcpu()) {
#if KVM_OPEN_DEBUG
        error("failed to initialize virtual cpu");
#endif
        return false;
    }
    if (!init_coverage_buffer()) {
#if KVM_OPEN_DEBUG
        error("failed to initialize coverage buffer");
#endif
        return false;
    }
    if (!load_memory_regions()) {
#if KVM_OPEN_DEBUG
        error("failed to load memory regions");
#endif
        return false;
    }
    if (!init_runtime_state()) {
#if KVM_OPEN_DEBUG
        error("failed to initialize runtime state");
#endif
        return false;
    }

#if KVM_OPEN_DEBUG
    info("virtual board successfully initialized");
#endif
    return true;
}

/// @brief destructor
Board::~Board() {
    // cancel memory slots
    for (auto slot: _kvm_slots) {
        cancel_memory_region(slot);
    }

    // clear memory
    if (_rom_or_flash_backend) {
        munmap(_rom_or_flash_backend, ROM_OR_FLASH_SIZE);
    }
    if (_sram_backend) {
        munmap(_sram_backend, SRAM_SIZE);
    }
    if (_wbwa_ram_backend) {
        munmap(_wbwa_ram_backend, WBWA_RAM_SIZE);
    }
    if (_wt_ram_backend) {
        munmap(_wt_ram_backend, WT_RAM_SIZE);
    }
    if (_runtime_backend) {
        munmap(_runtime_backend, RUNTIME_SIZE);
    }

    // close descriptor
    if (_vm_fd != -1) {
        close(_vm_fd);
    }
    if (_sys_fd != -1) {
        close(_sys_fd);
    }
    if (_gic_fd != -1) {
        close(_gic_fd);
    }
}

/// @brief check kvm extension
/// @param extension number of kvm extension
/// @return ioctl result
int Board::check_extension(unsigned int extension) {
    int ret = ioctl(_sys_fd, KVM_CHECK_EXTENSION, extension);
    if (ret < 0) {
        ret = 0;
    }
    return ret;
}

//==============================================================================
// CPU and Interrupt

/// @brief rigger irq with exception number irq
/// @param irq exception to be triggered
/// @return true on success, false on fail
bool Board::irq_trigger(int irq) {
#if KVM_OPEN_DEBUG
    debug("trigger irq-%d", irq);
#endif
    if (!irq_line(irq, 1)) {
        return false;
    }
    if (!irq_line(irq, 0)) {
        return false;
    }
    return true;
}

#define KVM_IRQCHIP_IRQ(x) (KVM_ARM_IRQ_TYPE_SPI << KVM_ARM_IRQ_TYPE_SHIFT) |\
			                ((x) & KVM_ARM_IRQ_NUM_MASK)

/// @brief set arm gic input irq line value
/// @param irq exception number
/// @param level exception level
/// @return true on success, false on fail
bool Board::irq_line(int irq, int level) {
    struct kvm_irq_level irq_level = {
		.irq	= KVM_IRQCHIP_IRQ(32),
		.level	= !!level,
	};

    if (ioctl(_vm_fd, KVM_IRQ_LINE, &irq_level) < 0) {
#if KVM_OPEN_DEBUG
        error("Could not KVM_IRQ_LINE for irq %d", irq);
#endif
        return false;
    }
    return true;
}

/// @brief init gic interrupt controller (only when needed)
/// @return true on success, false on fail
bool Board::init_gic() {
    if (!gic_create_irqchip()) {
#if KVM_OPEN_DEBUG
        error("failed to create irqchip");
#endif
        return false;
    }

	uint32_t nr_irqs = 64;
	struct kvm_device_attr nr_irqs_attr = {
		.group	= KVM_DEV_ARM_VGIC_GRP_NR_IRQS,
		.addr	= (uint64_t)(unsigned long)&nr_irqs,
	};
	struct kvm_device_attr vgic_init_attr = {
		.group	= KVM_DEV_ARM_VGIC_GRP_CTRL,
		.attr	= KVM_DEV_ARM_VGIC_CTRL_INIT,
	};

	if (!ioctl(_gic_fd, KVM_HAS_DEVICE_ATTR, &nr_irqs_attr)) {
		int ret = ioctl(_gic_fd, KVM_SET_DEVICE_ATTR, &nr_irqs_attr);
		if (ret) {
#if KVM_OPEN_DEBUG
            error("failed to set gic nr_irq");
#endif
            return false;
        }
	}

	if (!ioctl(_gic_fd, KVM_HAS_DEVICE_ATTR, &vgic_init_attr)) {
		int ret = ioctl(_gic_fd, KVM_SET_DEVICE_ATTR, &vgic_init_attr);
		if (ret) {
#if KVM_OPEN_DEBUG
            error("failed to set vgic init");
#endif
            return false;
        }
	}

    // init gic controler
    gic_write_dist_reg(0, 0);
    for (int i = 32; i < 64; i += 16)
        gic_write_dist_reg(4*(i>>4) + 0xc00, 0);
    for (int i = 0; i < 64; i += 4)
        gic_write_dist_reg(4*(i>>2) + 0x400, 0xa0a0a0a0U);
    for (int i = 32; i < 64; i += 4) 
        gic_write_dist_reg(4*(i>>2) + 0x800, 0);
    for (int i = 0; i < 64; i += 32)
        gic_write_dist_reg(4*(i >> 5) + 0x280, 0xffffffffU); 
    for (int i = 0; i < 64; i += 32)
        gic_write_dist_reg(4*(i >> 5) + 0x180, 0xffffffffU);
    gic_write_dist_reg(0, 1);
    
    gic_write_cpu_reg(4, 0xf0);
    gic_write_cpu_reg(0, 0x3);
    
    // enable our irq-32, we use irq32 to notify interrupt
    gic_write_dist_reg(4*(32>>2)+0x400, (0 & 0xF8) << (8 * (32 & 3)));
    gic_write_dist_reg(4*(32>>4)+0xc00, 3 << (2 * (32 & 0xF)));
    gic_write_dist_reg(4*(32>>5)+0x100, 1<<(32&0x1f));
	
    return true;
}

/// @brief write gic distribute register
/// @param offset offset of the register
/// @param value value to write 
/// @return true on success, false on fail
bool Board::gic_write_dist_reg(uint64_t offset, uint64_t value) {
    struct kvm_device_attr dist_regs_attr = {
		.group	= KVM_DEV_ARM_VGIC_GRP_DIST_REGS,
        .attr   = offset,
		.addr	= (uint64_t)(unsigned long)&value,
	};
    int ret = ioctl(_gic_fd, KVM_SET_DEVICE_ATTR, &dist_regs_attr);
    if (ret) {
#if KVM_OPEN_DEBUG
        error("failed to write gic dist reg 0x%x", offset);
#endif
        return false;
    }
    return true;
}

/// @brief write gic cpu register
/// @param offset offset of the register
/// @param value value to write
/// @return true on success, false on fail
bool Board::gic_write_cpu_reg(uint64_t offset, uint64_t value) {
    struct kvm_device_attr cpu_regs_attr = {
		.group	= KVM_DEV_ARM_VGIC_GRP_CPU_REGS,
        .attr   = offset,
		.addr	= (uint64_t)(unsigned long)&value,
	};
    int ret = ioctl(_gic_fd, KVM_SET_DEVICE_ATTR, &cpu_regs_attr);
    if (ret) {
#if KVM_OPEN_DEBUG
        error("failed to write gic vcpu reg 0x%x", offset);
#endif
        return false;
    }
    return true;
}

/// @brief read gic cpu register
/// @param offset offset of the register
/// @return value of the register, 0 on fail 
uint32_t Board::gic_read_cpu_reg(uint64_t offset) {
    uint64_t value;
    struct kvm_device_attr cpu_regs_attr = {
		.group	= KVM_DEV_ARM_VGIC_GRP_CPU_REGS,
        .attr   = offset,
		.addr	= (uint64_t)(unsigned long)&value,
	};
    int ret = ioctl(_gic_fd, KVM_GET_DEVICE_ATTR, &cpu_regs_attr);
    if (ret) {
#if KVM_OPEN_DEBUG
        error("failed to read gic vcpu reg 0x%x", offset);
#endif
        return 0;
    }
    return value;
}

/// @brief create gic in kernel irqchip
/// @return true on succ, false on fail
bool Board::gic_create_irqchip() {
	int err;
	uint64_t cpu_if_addr = ARM_GIC_CPUI_BASE;
	uint64_t dist_addr   = ARM_GIC_DIST_BASE;
	struct kvm_create_device gic_device = {
        .type = KVM_DEV_TYPE_ARM_VGIC_V2,
		.flags	= 0,
	};
	struct kvm_device_attr cpu_if_attr = {
		.group	= KVM_DEV_ARM_VGIC_GRP_ADDR,
		.attr	= KVM_VGIC_V2_ADDR_TYPE_CPU,
		.addr	= (uint64_t)(unsigned long)&cpu_if_addr,
	};
	struct kvm_device_attr dist_attr = {
		.group	= KVM_DEV_ARM_VGIC_GRP_ADDR,
        .attr   = KVM_VGIC_V2_ADDR_TYPE_DIST,
		.addr	= (uint64_t)(unsigned long)&dist_addr,
	};

	err = ioctl(_vm_fd, KVM_CREATE_DEVICE, &gic_device);
	if (err)
		return false;

	_gic_fd = gic_device.fd;

	err = ioctl(_gic_fd, KVM_SET_DEVICE_ATTR, &cpu_if_attr);
	if (err)
		goto out_err;

	err = ioctl(_gic_fd, KVM_SET_DEVICE_ATTR, &dist_attr);
	if (err)
		goto out_err;

	return true;

out_err:
	close(_gic_fd);
	return false;
}

//==============================================================================
// Memory Operations

/// @brief Transform guest physical address to memory backend buffer
/// @param gpa physical address of the guest
/// @param remain remain space after gpa to the end of backend
/// @return void * pointer to memory backend, nullptr when fail
void *Board::gpa_to_hva(uint32_t gpa, uint32_t *remain) {
    // ROM or Flash backend
    if (ROM_OR_FLASH_START <= gpa && gpa <= ROM_OR_FLASH_END) {
        if (remain) {
            *remain = ROM_OR_FLASH_SIZE - gpa + ROM_OR_FLASH_START;
        }
        return (void *)((uint64_t)_rom_or_flash_backend + gpa - ROM_OR_FLASH_START);
    }
    // SRAM backend
    if (SRAM_START <= gpa && gpa <= SRAM_END) {
        if (remain) {
            *remain = SRAM_SIZE - gpa + SRAM_START;
        }
        return (void *)((uint64_t)_sram_backend + gpa - SRAM_START);
    }
    // WT_RAM backend
    if (WT_RAM_START <= gpa && gpa <= WT_RAM_END) {
        if (remain) {
            *remain = WT_RAM_SIZE - gpa + WT_RAM_START;
        }
        return (void *)((uint64_t)_wt_ram_backend + gpa - WT_RAM_START);
    }
    // WBWA_RAM backend
    if (WBWA_RAM_START <= gpa && gpa <= WBWA_RAM_END) {
        if (remain) {
            *remain = WBWA_RAM_SIZE - gpa + WBWA_RAM_START;
        }
        return (void *)((uint64_t)_wbwa_ram_backend + gpa - WBWA_RAM_START);
    }
    // Runtime backend
    if (RUNTIME_START <= gpa && gpa <= RUNTIME_END) {
        if (remain) {
            *remain = RUNTIME_SIZE - gpa + RUNTIME_START;
        }
        return (void *)((uint64_t)_runtime_backend + gpa - RUNTIME_START);
    }
    // unknown region
    return nullptr;
}

/// @brief memcpy from guest physical address to host virtual address
/// @param hva host virtual address
/// @param gpa guest physical address
/// @param buf_size size of the buffer
/// @return true when success, false when fail
bool Board::gpa_to_hva_memcpy(void *hva, uint32_t gpa, uint32_t buf_size) {
    uint32_t remain;
    void *gpa_backend = gpa_to_hva(gpa, &remain);

    if (gpa_backend == nullptr || buf_size > remain) {
        return false;
    }
    memcpy(hva, gpa_backend, buf_size);

    return true;
}

/// @brief memcpy from host virtual address to guest physical address 
/// @param gpa guest physical address
/// @param hva host virtual address
/// @param buf_size size of the buffer
/// @return true when success, false when fail
bool Board::hva_to_gpa_memcpy(uint32_t gpa, void *hva, uint32_t buf_size) {
    uint32_t remain;
    void *gpa_backend = gpa_to_hva(gpa, &remain);

    if (gpa_backend == nullptr || buf_size > remain) {
        return false;
    }
    memcpy(gpa_backend, hva, buf_size);
    force_clear_cache(
        (uintptr_t)gpa_backend, 
        (uintptr_t)((uint64_t)gpa_backend + buf_size)
    );

    return true;
}

/// @brief read uint8_t from guest physical address
/// @param gpa guest physical address
/// @return value of the uint8_t, 0 when fail
uint8_t Board::read_u8(uint32_t gpa) {
    uint32_t remain;
    uint8_t *ptr = (uint8_t *)gpa_to_hva(gpa, &remain);
    if (ptr == nullptr || remain < 8) {
        return 0;
    }
    return *ptr;
}

/// @brief read uint16_t from guest physical address
/// @param gpa guest physical address
/// @return value of the uint16_t, 0 when fail
uint16_t Board::read_u16(uint32_t gpa) {
    uint32_t remain;
    uint16_t *ptr = (uint16_t *)gpa_to_hva(gpa, &remain);
    if (ptr == nullptr || remain < 16) {
        return 0;
    }
    return *ptr;
}

/// @brief read uint32_t from guest physical address
/// @param gpa guest physical address
/// @return value of the uint32_t, 0 when fail
uint32_t Board::read_u32(uint32_t gpa) {
    uint32_t remain;
    uint32_t *ptr = (uint32_t *)gpa_to_hva(gpa, &remain);
    if (ptr == nullptr || remain < 32) {
        return 0;
    }
    return *ptr;
}

/// @brief write uint8_t to guest physical address
/// @param gpa guest physical address
/// @param val value to br written
/// @param flush true to flush the memory
void Board::write_u8(uint32_t gpa, uint8_t val, bool flush) {
    uint32_t remain;
    uint8_t *ptr = (uint8_t *)gpa_to_hva(gpa, &remain);
    if (ptr == nullptr || remain < 8) {
        return;
    }
    *ptr = val;
    if (flush) {
        flush_memory(gpa, 0x1);
    }
}

/// @brief write uint16_t to guest physical address
/// @param gpa guest physical address
/// @param val value to br written
/// @param flush true to flush the memory
void Board::write_u16(uint32_t gpa, uint16_t val, bool flush) {
    uint32_t remain;
    uint16_t *ptr = (uint16_t *)gpa_to_hva(gpa, &remain);
    if (ptr == nullptr || remain < 16) {
        return;
    }
    *ptr = val;
    if (flush) {
        flush_memory(gpa, 0x2);
    }
}

/// @brief write uint32_t to guest physical address
/// @param gpa guest physical address
/// @param val value to br written
/// @param flush true to flush the memory
void Board::write_u32(uint32_t gpa, uint32_t val, bool flush) {
    uint32_t remain;
    uint32_t *ptr = (uint32_t *)gpa_to_hva(gpa, &remain);
    if (ptr == nullptr || remain < 32) {
        return;
    }
    *ptr = val;
    if (flush) {
        flush_memory(gpa, 0x4);
    }
}

/// @brief flush the data and instruction cache of the target region
/// @param gpa guest pgysical address
/// @param size size of the memory area
void Board::flush_memory(uint32_t gpa, uint32_t size) {
    void *backend = gpa_to_hva(gpa);
    force_clear_cache((uintptr_t)backend, (uintptr_t)((uint64_t)backend + size));
}

/// @brief invalid guest cache for FirmwareState structure
void Board::flush_firmware_state() {
    FirmwareState *fs = firmware_state();
    
    force_clear_cache(
        (uintptr_t)fs, 
        (uintptr_t)((uint64_t)fs + sizeof(FirmwareState))
    );
}

//==============================================================================
// Coverage Collection

/// @brief get the pointer to the coverage map located in runtime
/// @return pointer to the coverage map in FirmwareState
uint8_t *Board::coverage_map() {
    FirmwareState *fs = firmware_state();
    return fs->coverage_map;
}

/// @brief set ground truth of the coverage
/// @param ground_truth basic block list
void Board::set_coverage_ground_truth(std::vector<uint32_t> ground_truth) {
    _coverage_bpkts.clear();
    for (uint32_t addr: ground_truth) {
        uint32_t code = read_u32(addr);
        // hack: we skip potential infinative loop
        if ((code & 0xffff) == 0xe7fe) {
            continue;
        }
        _coverage_bpkts[addr] = std::make_pair(_coverage_bpkts.size(), code & 0xffff);
    }
    _hit_coverage_map.clear();
}

/// @brief hit coverage map
/// @return pointer to the hit coverage map
std::map<uint32_t, std::string> *Board::hit_coverage_map() {
    return &_hit_coverage_map;
}

/// @brief new hit coverage map
/// @return pointer to the new hit coverage map
std::map<uint32_t, std::string> *Board::new_hit_coverage_map() {
    return &_new_hit_coverage_map;
}

/// @brief set coverage map with pc
/// @param pc coverage pc
void Board::set_coverage_map_with_pc(uint32_t pc) {
    std::pair<int, uint32_t> info = _coverage_bpkts[pc];

    if (_total_coverage_buffer[info.first] == 0) {
        _hit_coverage_map[pc] = CONFIG.belonging_symbol(pc);
        _new_hit_coverage_map[pc] = CONFIG.belonging_symbol(pc);
    }
    _coverage_buffer[info.first] = 1;
    _total_coverage_buffer[info.first] = 1;

    uint32_t code = read_u32(pc);
    code &= 0xffff0000;
    code |= info.second;
    write_u32(pc, code, true);
}

/// @brief set coverage breakpoints
void Board::set_coverage_bkpts() {
    for (auto i = _coverage_bpkts.begin(); i != _coverage_bpkts.end(); ++i) {
        uint32_t code = read_u32(i->first);
        code &= 0xffff0000;
        code |= BKPT_COVERAGE;
        write_u32(i->first, code, true);
    }
    _new_hit_coverage_map.clear();
}

/// @brief reset coverage break points
/// @param pc position of the break point
void Board::reset_coverage_bkpt(uint32_t pc) {
    std::pair<int, uint32_t> info = _coverage_bpkts[pc];
    uint32_t code = read_u32(pc);
    code &= 0xffff0000;
    code |= info.second;
    write_u32(pc, code, true);
}

//==============================================================================
// Output Functions

/// @brief log error message
/// @param s error message
void Board::error(const char *s, ...) { 
    if (unlikely(CONFIG.option_should_target_output(EN_OUTPUT_BOARD))) {
        if (CONFIG.option_output_level() <= OUTPUT_LEVEL_ERROR) {
            char buffer[256];
            va_list args;
            va_start(args, s);
            vsprintf(buffer, s, args);
            va_end(args); 
            print_mutex.lock();
            printf("ERROR | host-board: %s\n", buffer);
            print_mutex.unlock();
        }
    }
}

/// @brief log info message
/// @param s info message
void Board::info(const char *s, ...) { 
    if (unlikely(CONFIG.option_should_target_output(EN_OUTPUT_BOARD))) {
        if (CONFIG.option_output_level() <= OUTPUT_LEVEL_INFO) {
            char buffer[256];
            va_list args;
            va_start(args, s);
            vsprintf(buffer, s, args);
            va_end(args); 
            print_mutex.lock();
            printf("INFO  | host-board: %s\n", buffer);
            print_mutex.unlock();
        }
    }
}

/// @brief log debug message
/// @param s debug message
void Board::debug(const char *s, ...) { 
    if (unlikely(CONFIG.option_should_target_output(EN_OUTPUT_BOARD))) {
        if (CONFIG.option_output_level() <= OUTPUT_LEVEL_DEBUG) {
            char buffer[256];
            va_list args;
            va_start(args, s);
            vsprintf(buffer, s, args);
            va_end(args); 
            print_mutex.lock();
            printf("DEBUG | host-board: %s\n", buffer);
            print_mutex.unlock();
        }
    }
}

}  // namespace khost
