#include "en_board_para.h"

namespace khost {

// =============================================================================
// construct and destruct operations

BoardPara::BoardPara() : Board() {}

/// @brief init devices and mmio address mapping
/// @return true on success, false on fail
bool BoardPara::init_devices() {
    _fuzzware_model = new FuzzwareModel();
    _nvic = new NVIC(this, NVIC_START);
    _mpu = new MMIOModel(MPU_START, "mpu");
    _sysctl = new SysCtl(this, SC_START);

    _mmio_map[NVIC_START] = MMIORegionDes(NVIC_SIZE, _nvic);
    _mmio_map[ON_CHIP_DEV_START] = MMIORegionDes(ON_CHIP_DEV_SIZE, _fuzzware_model);
    _mmio_map[S_DEV_START] = MMIORegionDes(S_DEV_SIZE, _fuzzware_model);
    _mmio_map[NS_DEV_START] = MMIORegionDes(NS_DEV_SIZE, _fuzzware_model);
    _mmio_map[MPU_START] = MMIORegionDes(MPU_SIZE, _mpu);
    _mmio_map[0xe000e000] = MMIORegionDes(0xe000e00f-0xe000e000+1, _sysctl);
    _mmio_map[SCB_START] = MMIORegionDes(SCB_SIZE, _sysctl);
    _mmio_map[0xe000edf0] = MMIORegionDes(0xe000eeff-0xe000edf0+1, _sysctl);
    _mmio_map[0xe000ef00] = MMIORegionDes(0xe000ef4f-0xe000ef00+1, _sysctl);
    _mmio_map[0xe000ef50] = MMIORegionDes(0xe000ef8f-0xe000ef50+1, _sysctl);
    _mmio_map[0xe000ef90] = MMIORegionDes(0xe000efcf-0xe000ef90+1, _sysctl);
    _mmio_map[0xe000efd0] = MMIORegionDes(0xe000efff-0xe000efd0+1, _sysctl);

    return true;
}

/* AP[2] = 0b0, AP[1:0] = 0b11 */
#define AP_RW               ((0b0 << 15) + (0b11 << 10))
/* AP[2] = 0b1, AP[1:0] = 0b11 */
#define AP_R                ((0b1 << 15) + (0b11 << 10))
/* XN = 0b1 */
#define PER_NX              (0b1 << 4)
/* DOMAIN = 1 */
#define DOMAIN_1            (0b1 << 5)

/// @brief refine the page table, set NX to invalid memory
void BoardPara::init_page_table() {
    auto in_region = [](uint32_t a, uint32_t b, uint32_t c, uint32_t d) -> bool { 
        return a < d && b >= c; 
    };

    uint32_t *page_table = (uint32_t *)gpa_to_hva(RT_PARA_page_table);
    uint32_t firm_start = CONFIG.firmware_info()->load_base;
    uint32_t firm_end = firm_start + CONFIG.firmware_info()->firmware_code_size;
    uint32_t addition_start = CONFIG.firmware_info()->addition_start;
    uint32_t addition_end = addition_start + CONFIG.firmware_info()->addition_size;
    uint32_t runtime_start = RUNTIME_START;
    uint32_t runtime_end = RUNTIME_START + RT_PARA_binary_size;

    // FLASH area
    uint32_t sect = 0;
    for (int i = 0; i < ROM_OR_FLASH_SIZE / PAGE_SIZE; i++) {
        // mark the part in flash but doesn't contain any code as non-execution
        if (!(
            in_region(sect, sect + PAGE_SIZE, firm_start, firm_end) ||
            in_region(sect, sect + PAGE_SIZE, addition_start, addition_end)
        )) {
            page_table[i] |= PER_NX;
        }
        sect = sect + PAGE_SIZE;
    }

    // check if the area is set valid
    sect = SRAM_START; 
    for (int i = 0; sect < RUNTIME_START; i++) {
        AddressInfo info = CONFIG.address_info(sect);
        if (info.start == 0 && info.end == 0) {
            // invalid memory area, set to DOMAIN_1
            page_table[SRAM_START / PAGE_SIZE + i] |= DOMAIN_1;
        } else {
            // valid memory area, set to DOMAIN_0
            if (!info.x) {
                // None Execution
                page_table[SRAM_START / PAGE_SIZE + i] |= PER_NX;
            }
            if (info.w) {
                // set AP_RW bits
                page_table[SRAM_START / PAGE_SIZE + i] |= AP_RW;
            } else {
                // set AP_R bits
                page_table[SRAM_START / PAGE_SIZE + i] |= AP_R;
            }
        }
        sect = sect + PAGE_SIZE;
    }

    // RUNTIME area
    sect = RUNTIME_START;
    for (int i = 0; i < (int)(RUNTIME_SIZE / PAGE_SIZE); i++) {
        if (!in_region(sect, sect + PAGE_SIZE, runtime_start, runtime_end)) {
            page_table[RUNTIME_START/PAGE_SIZE + i] |= PER_NX;
        }
        sect = sect + PAGE_SIZE;
    }
}

/// @brief init state in the runtime
/// @return true on success, false on fail
bool BoardPara::init_runtime_state() {
    // real firmware entry
    write_u32(RT_PARA_firmware_entry, CONFIG.firmware_info()->initial_pc | 0b1);

    // inital sp
    write_u32(RT_PARA_initial_sp, CONFIG.firmware_info()->initial_sp & 0xfffffffc);

    // load base
    write_u32(RT_PARA_load_base, CONFIG.firmware_info()->load_base);

    // number of irq
    write_u32(RT_PARA_num_irq, CONFIG.firmware_info()->num_irq);

    // init the interrupt configure
    write_u32(RT_PARA_init_period, CONFIG.init_period());

    // enable fuzzware
    write_u32(RT_PARA_use_fuzzware, CONFIG.enable_fuzzware());

    // init firmware info
    uint32_t firm_start = CONFIG.firmware_info()->load_base;
    uint32_t firm_end = firm_start + CONFIG.firmware_info()->firmware_code_size;
    uint32_t addition_start = CONFIG.firmware_info()->addition_start;
    uint32_t addition_end = addition_start + CONFIG.firmware_info()->addition_size;
    CodeRange *code_range = (CodeRange *)gpa_to_hva(RT_PARA_code_range);
    code_range->bin_start = firm_start;
    code_range->bin_end = firm_end;
    code_range->rewrite_start = addition_start;
    code_range->rewrite_end = addition_end;
    flush_memory(RT_PARA_code_range, sizeof(CodeRange));

    // init incompatible instruction list
    uint32_t cnt = 0;
    IncompatibleInsn *insn_list = (IncompatibleInsn *)gpa_to_hva(
        RT_PARA_incompatible_insn_list
    );
    for (auto insn : *CONFIG.get_incompatible_insn_map()) {
        if (cnt == RUNTIME_INCOMPATIBLE_INSN_CNT) {
#if KVM_OPEN_DEBUG
            error(
                "insn buffer too small: need %ld\n", 
                CONFIG.get_incompatible_insn_map()->size()
            );
#endif
            return false;
        }
        memcpy(&insn_list[cnt], insn.second, sizeof(IncompatibleInsn));
        cnt = cnt + 1;
    }
    flush_memory(RT_PARA_incompatible_insn_list, sizeof(IncompatibleInsn)*cnt);
    write_u32(RT_PARA_incompatible_insn_cnt, cnt);

    // init fuzzware model
    cnt = 0;
    FuzzwareModelInfo *model_list = (FuzzwareModelInfo *)gpa_to_hva(
        RT_PARA_fuzzware_model_list
    );
    for (auto model_map: *CONFIG.fuzzware_model()) {
        for (auto model: model_map.second) {
            if (cnt == RUNTIME_FUZZWARE_MODEL_CNT) {
#if KVM_OPEN_DEBUG
                error("fuzzware model buffer too small");
                return false;
#endif
            } 
            memcpy(&model_list[cnt], model.second, sizeof(FuzzwareModelInfo));
            cnt = cnt + 1;
        }
    }
    flush_memory(RT_PARA_fuzzware_model_list, sizeof(FuzzwareModelInfo)*cnt);
    write_u32(RT_PARA_fuzzware_model_cnt, cnt);

    // init the valid memory range
    auto address_vect = *CONFIG.address_info_list();
    AddressRange *info_list = (AddressRange *)gpa_to_hva(RT_PARA_address_range);
    if (address_vect.size() + 1 >= RUNTIME_ADDRESS_RANGE_SIZE) {
#if KVM_OPEN_DEBUG
        error("address range buffer too small");
#endif
        return false;
    }
    for (size_t i = 0; i < address_vect.size(); i++) {
        info_list[i].start = address_vect[i].start;
        info_list[i].end = address_vect[i].end;
    }
    info_list[address_vect.size()].start = 0xffffffff;
    info_list[address_vect.size()].end = 0xffffffff;
    flush_memory(RT_PARA_address_range, sizeof(AddressRange)*RUNTIME_ADDRESS_RANGE_SIZE);

    // refine the page table
    init_page_table();
    
    return true;
}

/// @brief init virtual cpu of the firmware
/// @return true on success, false on fail
bool BoardPara::init_vcpu() {
#if KVM_OPEN_DEBUG
    debug("creating vcpu...");    
#endif
    _cpu = new CPUPara(this);
    if (!_cpu->init_succ()) {
        delete _cpu;
        _cpu = nullptr;
        return false;
    }
    if (!init_gic()) {
        return false;
    }
    return true;
}

/// @brief destructor
BoardPara::~BoardPara() {
    if (_fuzzware_model) {
        delete _fuzzware_model;
    }
    if (_nvic) {
        delete _nvic;
    }
    if (_mpu) {
        delete _mpu;
    }
    if (_sysctl) {
        delete _sysctl;
    }
    if (_cpu) {
        delete _cpu;
    }
}

// =============================================================================
// Firmware State Operations

/// @brief get pointer to FirmwareState structure in runtime memory region
/// @return pointer of FirmwareState 
FirmwareState *BoardPara::firmware_state() {
    return (FirmwareState *)(gpa_to_hva(RT_PARA_firmware_state));
}

/// @brief reset FirmwareState structure in runtime memory region
void BoardPara::reset_firmware_state() {
    write_u32(RT_PARA_first_run, first_run());
}

//==============================================================================
// MMIO handle

MMIORegion *BoardPara::find_mmio_handler(uint64_t phy_addr) {
    // short cut here
    if (ON_CHIP_DEV_START <= phy_addr && phy_addr <= ON_CHIP_DEV_END) {
        return _fuzzware_model;
    }
    if (S_DEV_START <= phy_addr && phy_addr <= S_DEV_END) {
        return _fuzzware_model;
    }
    if (NS_DEV_START <= phy_addr && phy_addr <= NS_DEV_END) {
        return _fuzzware_model;
    }
    auto iter = _mmio_map.upper_bound(phy_addr);
    if (iter == _mmio_map.begin()
        || (--iter)->second.l + iter->first <= phy_addr) {
        return nullptr;
    }
    return iter->second.h;
}

//==============================================================================
// Reset and Execute Operations

// reset all hooks
bool BoardPara::reset_hooks() {

   // set report for coverage
   if (_coverage_bpkts.size()) {
       set_coverage_bkpts();
   }

   // set hal hooks
   for (auto hook_item: *CONFIG.hal_hooks()) {
       uint32_t addr = hook_item.first;
       std::string name = hook_item.second;
       bool found = false;
       for (int i = 0; RT_PARA_hal_table[i].addr != 0x0; i++) {
           uint32_t hook_addr = RT_PARA_hal_table[i].addr;
           std::string hook_name = RT_PARA_hal_table[i].name;
           if (hook_name == name) {
#if KVM_OPEN_DEBUG
                info("set hook: 0x%08x->%s", addr, name.c_str());
#endif
               found = true;
               // write to ldr.w pc, =0x????????
               write_u32(addr, 0xf000f8df);
               write_u32(addr+0x4, hook_addr | 0b1);
               break;
           }
       }
       if (!found) {
#if KVM_OPEN_DEBUG
            error("failed to find hal hook: %s", name.c_str());
#endif
            return false;
        }
    }

    // set hook for exit address
    for (auto addr: *CONFIG.hook_exit_addrs()) {
        uint16_t *code = (uint16_t *)gpa_to_hva(addr);
        *code = BKPT_TIMEOUT;
    }

    // set hook for nop address
    // hack: just skip origin operation
    for (auto addr: *CONFIG.hook_nop_addrs()) {
        write_u16(addr, 0xbf00);
    }

    // set hook for return zero address
    // hack: store 0 to r0, bx lr
    for (auto addr: *CONFIG.return_zero_addrs()) {
        //write_u32(addr, 0x20004770);
        write_u32(addr, 0x47702000);
    }
 
    // set hook for null pointer dereference
    uint32_t *code = (uint32_t *)gpa_to_hva(0x0);
    *code = BKPT_ARM_FIRMWARE_CRASH;
    
    rt_buffer = (uint8_t *)gpa_to_hva(RT_PARA_input_buffer);
    rt_size = (uint32_t *)gpa_to_hva(RT_PARA_input_len);
    rt_cur = (uint32_t *)gpa_to_hva(RT_PARA_input_cur);
	
	return true;
}

/// @brief reset the execution state of the board
/// @param buffer buffer of the input
/// @param size size of the input
/// @return true on success, false on fail
bool BoardPara::reset(uint8_t *buffer, uint32_t size) {
    // reload state vars
    reset_firmware_state();

    // reset systick
    _systick = new EmuSysTick(this);
    _mmio_map[SYS_TICK_START] = MMIORegionDes(SYS_TICK_SIZE, _systick);

    // set coverage bkpt after we load the memory
    if (first_run()) {
        // hack here: reuse coverage bkpt
        uint32_t tm_begin_addr = CONFIG.time_measure_begin_addr();
        uint32_t tm_end_addr = CONFIG.time_measure_end_addr();
        if (tm_begin_addr != (uint32_t)-1) {
            printf("INFO  | khost: set begin hook @ 0x%08x\n", tm_begin_addr);
            uint32_t code = read_u32(tm_begin_addr);
            _coverage_bpkts[tm_begin_addr] = std::make_pair(_coverage_bpkts.size(), code & 0xffff);
        }
        if (tm_end_addr != (uint32_t)-1) {
            printf("INFO  | khost: set end hook @ 0x%08x\n", tm_end_addr);
            uint32_t code = read_u32(tm_end_addr);
            _coverage_bpkts[tm_end_addr] = std::make_pair(_coverage_bpkts.size(), code & 0xffff);
        }
		if (!reset_hooks()) {
			return false;
		} 
    } else {
#if KVM_OPEN_DEBUG
		if (!reset_hooks()) {
			return false;
		} 

#endif
	}

	// load input
    // load_input(buffer, size);
    uint32_t load_size = size < RUNTIME_INPUT_BUFFER_SIZE ? size : RUNTIME_INPUT_BUFFER_SIZE;
    if (load_size != size) {
    	error("input buffer too small");
    }
    memcpy(rt_buffer, buffer, load_size);
    *rt_size = load_size;
    *rt_cur = 0;

	// clear coverage
    _new_hit_coverage_map.clear();

    // reset instance
    return _cpu->reset() && _nvic->reset() && _sysctl->reset() &&
           _fuzzware_model->reset(buffer, size);
}

/// @brief run the firmware
/// @return 
///   0(EXIT_OK): firmware request to exit or input is used up
///   1(EXIT_FIRMWARE_CRASH): crash inside the firmware
///   2(EXIT_INTERNAL_CRASH): crash because of the engine
///   3(EXIT_TIMEOUT): thread killed by user or fuzzer
int BoardPara::run() {
#if KVM_OPEN_DEBUG
    info("running start");
#endif    
    int ret = _cpu->loop();
    delete _systick;
    _systick   = nullptr;
    _first_run = false;

    switch (ret)
    {
    case EXIT_OK:
#if KVM_OPEN_DEBUG
        info("running exit: EXIT_OK");
#endif
        break;
    case EXIT_TIMEOUT:
#if KVM_OPEN_DEBUG
        info("running exit: EXIT_TIMEOUT");
#endif
        break;
    case EXIT_FIRMWARE_CRASH:
#if KVM_OPEN_DEBUG
        error("running exit: EXIT_FIRMWARE_CRASH");
#endif
        break;
    case EXIT_INTERNAL_CRASH:
#if KVM_OPEN_DEBUG
        error("running exit: EXIT_INTERNAL_CRASH");
#endif
        break;
    case EXIT_MODEL:
#if KVM_OPEN_DEBUG
        error("running exit: EXIT_MODEL");
#endif
        break;
    default:
#if KVM_OPEN_DEBUG
        error("running exit: unknown reason %d", ret);
#endif
        break;
    }
    return ret;
}

/// @brief load input into the firmware
/// @param buffer input buffer
/// @param size size of the input
void BoardPara::load_input(uint8_t *buffer, uint32_t size) {
    uint32_t load_size = size < RUNTIME_INPUT_BUFFER_SIZE ? size : RUNTIME_INPUT_BUFFER_SIZE;
    if (load_size != size) {
        error("input buffer too small");
    }

    uint8_t *rt_buffer = (uint8_t *)gpa_to_hva(RT_PARA_input_buffer);
    memcpy(rt_buffer, buffer, load_size);

    uint32_t *rt_size = (uint32_t *)gpa_to_hva(RT_PARA_input_len);
    *rt_size = load_size;

    uint32_t *rt_cur = (uint32_t *)gpa_to_hva(RT_PARA_input_cur);
    *rt_cur = 0;
}

}  // namespace khost
