#include "en_board_full.h"
#if KVM_OPEN_DEBUG
    #include "autogen_rt_full_debug.h"
#else
    #include "autogen_rt_full_normal.h"
#endif

#include <linux/mman.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <errno.h>
#include <sys/prctl.h>
#include <sys/time.h>
#include <signal.h>
#include <iostream>
#include <random>
#include <sstream>
#include <map>
#include <vector>
#include <string>
#include <fstream>
#include <filesystem>
#include <sstream>
#include <functional>

namespace khost {

// =============================================================================
// construct and destruct operations

BoardFull::BoardFull() : Board() {}

/// @brief init memory backend for the virtual machine
/// @return true on success, false on fail
bool BoardFull::init_memory_backends() {
    if (!Board::init_memory_backends()) {
        return false;
    }
    uint32_t simple_attr = MAP_SHARED | MAP_ANONYMOUS | MAP_POPULATE;

#ifndef DISABLE_APT
    _on_chip_device_backend = mmap(
        NULL, ON_CHIP_DEV_SIZE, PROT_READ | PROT_WRITE, simple_attr, -1, 0
    );
    if (_on_chip_device_backend == MAP_FAILED) {
        error("failed to create memory for on chip device");
        _on_chip_device_backend = nullptr;
        return false;
    }
    memset(_on_chip_device_backend, 0, ON_CHIP_DEV_SIZE);

    _off_chip_device_backend = mmap(
        NULL, VEN_SYS_DEV_END - S_DEV_START + 1, PROT_READ | PROT_WRITE, 
        simple_attr, -1, 0
    );
    if (_off_chip_device_backend == MAP_FAILED) {
        error("failed to create memory for off chip device");
        _off_chip_device_backend = nullptr;
        return false;
    }
    memset(_off_chip_device_backend, 0, VEN_SYS_DEV_END - S_DEV_START + 1);
    
    if (!submit_memory_region(
        RUNTIME_SLOT+1, RUNTIME_FLAGS, ON_CHIP_DEV_START, 
        _on_chip_device_backend, ON_CHIP_DEV_SIZE
    )) {
        error("failed to submit on-device memory slot");
        return false;
    }

    if (!submit_memory_region(
        RUNTIME_SLOT+2, RUNTIME_FLAGS, S_DEV_START, 
        _off_chip_device_backend, VEN_SYS_DEV_END - S_DEV_START + 1
    )) {
        error("failed to submit off-device memory slot");
        return false;
    }
#endif
    
    char *id_str = getenv("__AFL_SHM_ID");
    if (id_str) {
        uint32_t shm_id = atoi(id_str);
        _afl_area_ptr = (uint8_t *)shmat(shm_id, 0, 0);
        if (!_afl_area_ptr || _afl_area_ptr == (void *)-1) {
            error("failed to map coverage area");
            return false;
        }
    }

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
void BoardFull::init_page_table() {
    auto in_region = [](uint32_t a, uint32_t b, uint32_t c, uint32_t d) -> bool { 
        return a < d && b >= c; 
    };

    uint32_t *page_table = (uint32_t *)gpa_to_hva(RT_FULL_page_table);
    uint32_t firm_start = CONFIG.firmware_info()->load_base;
    uint32_t firm_end = firm_start + CONFIG.firmware_info()->firmware_code_size;
    uint32_t addition_start = CONFIG.firmware_info()->addition_start;
    uint32_t addition_end = addition_start + CONFIG.firmware_info()->addition_size;
    uint32_t runtime_start = RUNTIME_START;
    uint32_t runtime_end = RUNTIME_START + RT_FULL_binary_size;
    uint32_t sect = 0;

    // FLASH area
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
    sect = 0; 
    for (int i = 0; sect < RUNTIME_START; i++) {
        AddressInfo info = CONFIG.address_info(sect);
        if (info.start == 0 && info.end == 0) {
            // invalid memory area, set to DOMAIN_1
            page_table[i] |= DOMAIN_1;
        } else {
            // valid memory area, set to DOMAIN_0
            if (!info.x) {
                // None Execution
                page_table[i] |= PER_NX;
            }
            if (info.w) {
                // set AP_RW bits
                page_table[i] |= AP_RW;
            } else {
                // set AP_R bits
                page_table[i] |= AP_R;
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
bool BoardFull::init_runtime_state() {
    // real firmware entry
    write_u32(RT_FULL_firmware_entry, CONFIG.firmware_info()->initial_pc | 0b1);
#if KVM_OPEN_DEBUG
    info("set firmware entry: 0x%08x", CONFIG.firmware_info()->initial_pc | 0b1);
#endif

    // inital sp
    write_u32(RT_FULL_initial_sp, CONFIG.firmware_info()->initial_sp & 0xfffffffc);
#if KVM_OPEN_DEBUG
    info("set initial sp: 0x%08x", CONFIG.firmware_info()->initial_sp & 0xfffffffc);
#endif

    // load base
    write_u32(RT_FULL_load_base, CONFIG.firmware_info()->load_base);
#if KVM_OPEN_DEBUG
    info("set load base: 0x%08x", CONFIG.firmware_info()->load_base);
#endif
    // number of irq
    write_u32(RT_FULL_num_irq, CONFIG.firmware_info()->num_irq);
#if KVM_OPEN_DEBUG
    info("set number of total irq: %d", CONFIG.firmware_info()->num_irq);
#endif
    // enable fuzzware
    write_u32(RT_FULL_use_fuzzware, CONFIG.enable_fuzzware());

    // init firmware info
    uint32_t firm_start = CONFIG.firmware_info()->load_base;
    uint32_t firm_end = firm_start + CONFIG.firmware_info()->firmware_code_size;
    uint32_t addition_start = CONFIG.firmware_info()->addition_start;
    uint32_t addition_end = addition_start + CONFIG.firmware_info()->addition_size;
    CodeRange *code_range = (CodeRange *)gpa_to_hva(RT_FULL_code_range);
    code_range->bin_start = firm_start;
    code_range->bin_end = firm_end;
    code_range->rewrite_start = addition_start;
    code_range->rewrite_end = addition_end;
    flush_memory(RT_FULL_code_range, sizeof(CodeRange));

    // init incompatible instruction list
    uint32_t cnt = 0;
    IncompatibleInsn *insn_list = (IncompatibleInsn *)gpa_to_hva(
        RT_FULL_incompatible_insn_list
    );
    for (auto insn : *CONFIG.get_incompatible_insn_map()) {
        if (cnt == RUNTIME_INCOMPATIBLE_INSN_CNT) {
            error(
                "insn buffer too small: need %ld\n", 
                CONFIG.get_incompatible_insn_map()->size()
            );
            return false;
        }
        memcpy(&insn_list[cnt], insn.second, sizeof(IncompatibleInsn));
        cnt = cnt + 1;
    }
    flush_memory(RT_FULL_incompatible_insn_list, sizeof(IncompatibleInsn)*cnt);
    write_u32(RT_FULL_incompatible_insn_cnt, cnt);
#if KVM_OPEN_DEBUG
    info("%d incompatible instructions loaded to the runtime", cnt);
#endif

    // init fuzzware model
    cnt = 0;
    FuzzwareModelInfo *model_list = (FuzzwareModelInfo *)gpa_to_hva(
        RT_FULL_fuzzware_model_list
    );
    for (auto model_map: *CONFIG.fuzzware_model()) {
        for (auto model: model_map.second) {
            if (cnt == RUNTIME_FUZZWARE_MODEL_CNT) {
                error("fuzzware model buffer too small");
                return false;
            } 
            memcpy(&model_list[cnt], model.second, sizeof(FuzzwareModelInfo));
            cnt = cnt + 1;
        }
    }
    flush_memory(RT_FULL_fuzzware_model_list, sizeof(FuzzwareModelInfo)*cnt);
    write_u32(RT_FULL_fuzzware_model_cnt, cnt);
#if KVM_OPEN_DEBUG
    info("%d fuzzware model loaded to the runtime", cnt);
#endif

    // init the valid memory range
    auto address_vect = *CONFIG.address_info_list();
    AddressRange *info_list = (AddressRange *)gpa_to_hva(RT_FULL_address_range);
    if (address_vect.size() + 1 >= RUNTIME_ADDRESS_RANGE_SIZE) {
        error("address range buffer too small");
        return false;
    }
    for (size_t i = 0; i < address_vect.size(); i++) {
        info_list[i].start = address_vect[i].start;
        info_list[i].end = address_vect[i].end;
#if KVM_OPEN_DEBUG
        info("set valid memory range: [0x%08x, 0x%08x)", info_list[i].start, info_list[i].end);
#endif
    }
    info_list[address_vect.size()].start = 0xffffffff;
    info_list[address_vect.size()].end = 0xffffffff;
    flush_memory(RT_FULL_address_range, sizeof(AddressRange)*RUNTIME_ADDRESS_RANGE_SIZE);

    // refine the page table
    init_page_table();

    // set output config
    write_u32(RT_FULL_output_option, CONFIG.option_output_target());
    
    return true;
}

/// @brief init virtual cpu of the firmware
/// @return true on success, false on fail
bool BoardFull::init_vcpu() {
#if KVM_OPEN_DEBUG
    debug("creating vcpu...");    
#endif
    _cpu = new CPUFull(this);
    if (!_cpu->init_succ()) {
        delete _cpu;
        _cpu = nullptr;
        return false;
    }
    return true;
}

/// @brief init virtual devices
/// @return true and always success
bool BoardFull::init_devices() {
    return true;
}

/// @brief destructor
BoardFull::~BoardFull() {
    // clear share memory coverage area
    if (_afl_area_ptr) {
        shmdt(_afl_area_ptr);
    }

    // clear memory
    if (_on_chip_device_backend) {
        munmap(_on_chip_device_backend, ON_CHIP_DEV_SIZE);
    }
    if (_off_chip_device_backend) {
        munmap(_off_chip_device_backend, VEN_SYS_DEV_END - S_DEV_START + 1);
    }

    if (_cpu) {
        delete _cpu;
    }
}

// =============================================================================
// Firmware State Operations

/// @brief get pointer to FirmwareState structure in runtime memory region
/// @return pointer of FirmwareState 
FirmwareState *BoardFull::firmware_state() {
    return (FirmwareState *)(gpa_to_hva(RT_FULL_firmware_state));
}

/// @brief reset FirmwareState structure in runtime memory region
void BoardFull::reset_firmware_state() {
    write_u32(RT_FULL_first_run, first_run());

    // reset the interrupt configure
    uint32_t init_period = CONFIG.fire_after() == (uint32_t)-1 ? CONFIG.init_period() : 0x3f3f3f3f;
    write_u32(RT_FULL_init_period, init_period);
#if KVM_OPEN_DEBUG
    info("set interrupt initial period: %d", init_period);
#endif
}

/// @brief load input into the firmware
/// @param buffer input buffer
/// @param size size of the input
void BoardFull::load_input(uint8_t *buffer, uint32_t size) {
    uint32_t load_size = size < RUNTIME_INPUT_BUFFER_SIZE ? size : RUNTIME_INPUT_BUFFER_SIZE;
    if (load_size != size) {
        error("input buffer too small");
    }

    uint8_t *rt_buffer = (uint8_t *)gpa_to_hva(RT_FULL_input_buffer);
    memcpy(rt_buffer, buffer, load_size);

    uint32_t *rt_size = (uint32_t *)gpa_to_hva(RT_FULL_input_len);
    *rt_size = load_size;

    uint32_t *rt_cur = (uint32_t *)gpa_to_hva(RT_FULL_input_cur);
    *rt_cur = 0;
}


//==============================================================================
// Reset and Execute Operations
//
// reset all hooks
bool BoardFull::reset_hooks() {

   // set report for coverage
   if (_coverage_bpkts.size()) {
       set_coverage_bkpts();
   }

   // set hal hooks
   for (auto hook_item: *CONFIG.hal_hooks()) {
       uint32_t addr = hook_item.first;
       std::string name = hook_item.second;
       bool found = false;
       for (int i = 0; RT_FULL_hal_table[i].addr != 0x0; i++) {
           uint32_t hook_addr = RT_FULL_hal_table[i].addr;
           std::string hook_name = RT_FULL_hal_table[i].name;
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
    
	rt_buffer = (uint8_t *)gpa_to_hva(RT_FULL_input_buffer);
    rt_size = (uint32_t *)gpa_to_hva(RT_FULL_input_len);
    rt_cur = (uint32_t *)gpa_to_hva(RT_FULL_input_cur);
	
	return true;
}

/// @brief reset the execution state of the board
/// @param buffer buffer of the input
/// @param size size of the input
/// @return true on success, false on fail
bool BoardFull::reset(uint8_t *buffer, uint32_t size) {
    // reload state vars
    reset_firmware_state();

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
        uint32_t fire_after_addr = CONFIG.fire_after();
        if (fire_after_addr != (uint32_t)-1) {
#if KVM_OPEN_DEBUG
            info("fire after address 0x%08x", fire_after_addr);
#endif
            if (_coverage_bpkts.find(fire_after_addr) == _coverage_bpkts.end()) {
                uint32_t code = read_u32(fire_after_addr);
                _coverage_bpkts[fire_after_addr] = std::make_pair(_coverage_bpkts.size(), code & 0xffff);
            }
        }

		if(!reset_hooks()) {
			return false;
		}
    } else {
#if KVM_OPEN_DEBUG
		// because set_coverage_map_with_pc with restore original, so we must all hooks when using multi replay
		if(!reset_hooks()) {
			return false;
		}
#endif
	}
	// load input
    uint32_t load_size = size < RUNTIME_INPUT_BUFFER_SIZE ? size : RUNTIME_INPUT_BUFFER_SIZE;
    if (load_size != size) {
		error("input buffer too small");
	}
	memcpy(rt_buffer, buffer, load_size);
	*rt_size = load_size;
	*rt_cur = 0;

    // clear coverage
    _new_hit_coverage_map.clear();

    // reset cpu
    return _cpu->reset();
}

static BoardFull *current_board = nullptr;

/// @brief handle SIGALRM and SIGINT 
/// @param sig signal id
static void signal_handler(int sig) {
    if (sig == SIGALRM) {
        if (current_board) {
            current_board->cpu()->set_running(false);
        }
    }
    if (sig == SIGINT) {
        if (current_board) {
            current_board->cpu()->dump_state();
        }
        printf("ERROR | khost: terminated by user\n");
        exit(-1);
    }
}

/// @brief init signal handler
/// @return true on success, false on fail
bool BoardFull::init_signal() {
    // set SIGALRM handler
    if (signal(SIGALRM, signal_handler) == SIG_ERR) {
        error("failed to set signal SIGALRM handler");
        return false;
    }
    // set SIGINT handler
    if (signal(SIGINT, signal_handler) == SIG_ERR) {
        error("failed to set signal SIGINT handler");
        return false;
    }
    // make sure SIGALARM and SIGINT is unblocked
    sigset_t set;
    sigemptyset(&set);
    sigaddset(&set, SIGALRM);
    sigaddset(&set, SIGINT);
    if (pthread_sigmask(SIG_UNBLOCK, &set, NULL) != 0) {
        return false;
    }
    return true;
}

/// @brief run the firmware
/// @return 
///   0(EXIT_OK): firmware request to exit or input is used up
///   1(EXIT_FIRMWARE_CRASH): crash inside the firmware
///   2(EXIT_INTERNAL_CRASH): crash because of the engine
///   3(EXIT_TIMEOUT): thread killed by user or fuzzer
int BoardFull::run() {
#if KVM_OPEN_DEBUG
    info("running start");
#endif
    current_board = this;
    init_signal();

    // run the vcpu
    int ret = _cpu->loop();
    _first_run = false;

    // report coverage to share memory
    report_coverage();
    
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

    current_board = nullptr;
    return ret;
}

// =============================================================================
// coverage colletion

/// @brief report coverage to afl
void BoardFull::report_coverage() {
    if (_afl_area_ptr != 0) {
        memcpy(_afl_area_ptr, coverage_map(), COVERAGE_MAP_SIZE);
    }
}

// =============================================================================
// Fuzzware Model

struct HexLine {
    std::vector<uint8_t> line_buffer;

    HexLine() { line_buffer.clear(); }
    void append_u8(uint8_t v) { line_buffer.push_back(v); }
    void append_u16_be(uint16_t v) {
        line_buffer.push_back((v >> 8) & 0xff);
        line_buffer.push_back((v >> 0) & 0xff);
    }
    void append_u16_le(uint16_t v) {
        line_buffer.push_back((v >> 0) & 0xff);
        line_buffer.push_back((v >> 8) & 0xff);
    }
    std::string to_string() {
        std::string ret = ":";
        char buffer[64];
        uint32_t checksum = 0;
        for (size_t i = 0; i < line_buffer.size(); i++) {
            checksum += line_buffer[i];
            sprintf(buffer, "%02x", line_buffer[i]);
            ret.append(buffer);
        }
        checksum = (0x100 - checksum) & 0xff;
        sprintf(buffer, "%02x", checksum);
        ret.append(buffer);
        ret.append("\r\n");
        return ret;
    }
};

static std::vector<HexLine> buffer_to_hex(uint32_t addr, uint8_t *buf, uint32_t sz) {
    std::vector<HexLine> ret;
    // header line
    uint32_t i = 0;
    for (uint32_t base_addr = addr & 0xffff0000; base_addr < addr + sz; base_addr += 0x10000) {
        HexLine head_line;
        head_line.append_u8(0x02); 
        head_line.append_u8(0x00);
        head_line.append_u8(0x00);
        head_line.append_u8(0x04);
        head_line.append_u16_be(base_addr >> 16);
        ret.push_back(head_line);
        // data lines
        uint32_t top = addr + sz < base_addr + 0x10000 ? addr + sz : base_addr + 0x10000;
        for (; addr + i < top; i += 0x10) {
            HexLine data_line;
            data_line.append_u8(0x10);
            data_line.append_u16_be((addr + i) & 0xffff);
            data_line.append_u8(0x00);
            bool need_push = false;
            for (uint32_t j = 0; j < 0x10; j++) {
                uint8_t value = (i + j >= sz) ? 0x0 : buf[i + j];
                if (value) {
                    need_push = true;
                }
                data_line.append_u8(value); 
            }
            if (need_push) {
                ret.push_back(data_line);
            }
        }
    }
    return ret;
} 

/// @brief create fuzzware statefile
/// @param file_path statefile path
/// @return true on succ, false on fail
bool BoardFull::create_statefile(std::string file_path) {
    // create snapshot file at file_path
    std::filesystem::create_directory(
        std::filesystem::path(file_path).parent_path()
    );
    std::ofstream fstream(file_path);
    if (!fstream) {
        printf("failed to write snapshot file to %s", file_path.c_str());
        return false;
    }
    
    // dump registers
    struct { char name[8]; int id; } reg_list[] = {
        { "r0", 0 }, { "r1", 1 }, { "r2", 2 }, { "r3", 3},
        { "r4", 4 }, { "r5", 5 }, { "r6", 6 }, { "r7", 7},
        { "r8", 8 }, { "r9", 9 }, { "r10", 10 }, { "r11", 11},
        { "r12", 12 }, { "lr", 14 }, { "pc", 15 }, { "sp", 13},
    };
    char line_buffer[256];
    uint32_t *context_buffer = (uint32_t *)gpa_to_hva(RT_FULL_firmware_context);
    for (int i = 0; i < 16; i++) {
        uint32_t val = context_buffer[reg_list[i].id];
        if (reg_list[i].id == 14 || reg_list[i].id == 15) {
            val = CONFIG.reloc_pc(context_buffer[reg_list[i].id] & (~0b1)) | 1;
        }
        if (reg_list[i].id == 15) {
#if KVM_OPEN_DEBUG
            info("   working on model for PC=0x%08x\n", val);
#endif
        }
        sprintf(line_buffer, "%s=0x%08x\n", reg_list[i].name, val);
        fstream << line_buffer;
    }
    sprintf(line_buffer, "xpsr=0x%08x\n", _cpu->xpsr());
    fstream << line_buffer;

    // dump memory of original firmware (before patch)
    int code_bin_fd = open(CONFIG.frimware_path().c_str(), O_RDONLY);
    if (code_bin_fd < 0) {
        printf("failed to open firmware binary\n");
        return false;
    }
    void *code_bin_buffer = mmap(NULL, ROM_OR_FLASH_SIZE, 
                                 PROT_READ | PROT_WRITE, 
                                 MAP_SHARED | MAP_ANONYMOUS, -1, 0);
    if (code_bin_buffer == MAP_FAILED) {
        printf("failed to alloc code buffer");
        munmap(code_bin_buffer, ROM_OR_FLASH_SIZE);
        return false;
    }
    uint32_t code_bin_size = read(code_bin_fd, code_bin_buffer, ROM_OR_FLASH_SIZE);
    if (code_bin_size == (ssize_t)-1) {
        printf("failed to read binary code");
        close(code_bin_fd);
        munmap(code_bin_buffer, ROM_OR_FLASH_SIZE);
        return false;
    }
    for (auto lines: buffer_to_hex(
        CONFIG.firmware_info()->load_base, 
        (uint8_t *)code_bin_buffer, code_bin_size
    )) {
        fstream << lines.to_string().c_str();
    }
    close(code_bin_fd);
    munmap(code_bin_buffer, ROM_OR_FLASH_SIZE);
    // dump simple memory
    for (auto lines: buffer_to_hex(
        SRAM_START, (uint8_t *)gpa_to_hva(SRAM_START),
        CONFIG.firmware_info()->initial_sp - SRAM_START + 1
    )) {
        fstream << lines.to_string().c_str();
    }
    // [TODO] dump SCB memory
    uint8_t *scb_buffer = (uint8_t *)malloc(SCB_END - SC_START + 1);
    memset(scb_buffer, 0, SCB_END - SC_START + 1);
    for (auto lines: buffer_to_hex(SC_START, scb_buffer, SCB_END - SC_START + 1)) {
        fstream << lines.to_string().c_str();
    }
    free(scb_buffer);

    fstream << ":00000001FF\r\n";
    fstream.close();
    return true;
}

/// @brief merge two yaml nodes
/// @param a yaml node a
/// @param b yaml node b
/// @return merged yaml node
static YAML::Node merge_yaml(const YAML::Node& a, const YAML::Node& b) {
    YAML::Node result = a;
    for (const auto& it : b) {
        const std::string key = it.first.as<std::string>();
        if (result[key] && result[key].IsMap() && it.second.IsMap()) {
            result[key] = merge_yaml(result[key], it.second);
        } else {
            result[key] = it.second;
        }
    }
    return result;
}

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

/// @brief update the fuzzware model
/// @return true on succ, false on fail
bool BoardFull::update_fuzzware_model() {
    // create uuid for statefile name 
    std::random_device rand_dev;
    std::mt19937_64 rand_gen(rand_dev());
    std::uniform_int_distribution<uint64_t> rand_dis;
    uint64_t part1 = rand_dis(rand_gen);
    uint64_t part2 = rand_dis(rand_gen);
    std::ostringstream oss;
    oss << std::hex << part1 << part2;
    std::string uuid_str = oss.str().substr(0, 32);
    std::string statefile_path = "/tmp/" + uuid_str + "_statefile.hex";
    std::string output_yml_path = "/tmp/" + uuid_str + "_output.yml";

    // create snapshot file at /tmp/{uuid}_statefile.hex
    if (!create_statefile(statefile_path)) {
        return false;
    }

    // use fuzzware to generate the model
    // cmd: fuzzware_model -c /tmp/{uuid}_output.yml -C config.yml /tmp/{uuid}_statefile.hex
    std::string cmd = (
        "fuzzware_model -c " + output_yml_path
        + " -C " + CONFIG.config_path() + " " + statefile_path
    );
    if (!CONFIG.option_should_target_output(EN_OUTPUT_FUZZWARE)) {
        cmd = cmd + " > /dev/null 2>&1";
    }

    int status = std::system(cmd.c_str());
    if (status != 0) {
        error("failed to run fuzzware modeling command: %s\n", cmd.c_str());
        return false;
    }

    // update model
    YAML::Node config;
    try {
        config = YAML::LoadFile(output_yml_path);
    } catch (...) {
        error("failed to load config from file: %s", output_yml_path.c_str());
        return false;
    }
    YAML::Node fuzzware_model = config["mmio_models"];
    if (fuzzware_model.IsDefined() && !fuzzware_model.IsNull()) {
        for (auto model_by_type: fuzzware_model) {
            std::string name_region = model_by_type.first.as<std::string>();
            for (auto region: fuzzware_model[name_region]) {
                // set basic info of model
                struct FuzzwareModelInfo *model = new FuzzwareModelInfo;
                model->pc          = region.second["pc"].as<unsigned>();
                for (const auto& [key, val] : *CONFIG.reloc_map()) {
                    if (val == model->pc) {
                        model->pc = key;
                    }
                }
                model->addr        = region.second["addr"].as<unsigned>();
                model->access_size = region.second["access_size"].as<int>();
                // bitextract
                if ("bitextract" == name_region) {
                    model->type       = FUZZWARE_BITEXTRACT;
                    model->mask       = region.second["mask"].as<unsigned>();
                    model->size       = region.second["size"].as<int>();
                    model->left_shift = region.second["left_shift"].as<unsigned>();
                }
                // constant
                else if ("constant" == name_region) {
                    model->type = FUZZWARE_CONSTANT;
                    model->val  = region.second["val"].as<unsigned>();
                }
                // passthrough
                else if ("passthrough" == name_region) {
                    model->type     = FUZZWARE_PASSTHROUGH;
                    model->init_val = region.second["init_val"].as<unsigned>();
                }
                // set
                else if ("set" == name_region) {
                    model->type = FUZZWARE_SET;
                    auto value_vec = region.second["vals"].as<std::vector<unsigned>>();
                    for (uint32_t i = 0; i < value_vec.size(); i++) {
                        if (i == RUNTIME_FUZZWARE_SET_SIZE) {
                            printf("fuzzware set size too small\n");
                            return false;
                        }
                        model->vals[i] = value_vec[i];
                    }
                    model->val_cnt = value_vec.size();
                }
                // unmodeled region is fall back to identity
                else {
                    model->type = FUZZWARE_IDENTITY;
                }

                uint32_t *cnt = (uint32_t *)gpa_to_hva(RT_FULL_fuzzware_model_cnt);
                FuzzwareModelInfo *model_list = (FuzzwareModelInfo *)gpa_to_hva(RT_FULL_fuzzware_model_list);
                if (*cnt == RUNTIME_FUZZWARE_MODEL_CNT) {
                    error("fuzzware model buffer too small");
                    exit(-1);
                }
                memcpy(&model_list[*cnt], model, sizeof(FuzzwareModelInfo));
                *cnt = *cnt + 1;

                force_clear_cache(
                    (uintptr_t)&model_list[*cnt], 
                    (uintptr_t)&model_list[*cnt + 1]
                );
                force_clear_cache(
                    (uintptr_t)cnt, 
                    (uintptr_t)((uint64_t)cnt + 0x4)
                );
                delete model;
            }
        }
    }

    // merge config yml
    YAML::Node old_config;
    try {
        old_config = YAML::LoadFile(CONFIG.config_path());
    } catch (...) {
        error("failed to load config from file: %s", CONFIG.config_path().c_str());
        return false;
    }
    YAML::Node merged = merge_yaml(config, old_config);
    std::ofstream fout(CONFIG.config_path());
    fout << merged;
    fout.close();

    // clean the folder
    status = std::system((
        "rm " + statefile_path + " " + output_yml_path
    ).c_str());
    if (status != 0) {
#if KVM_OPEN_DEBUG
        error("failed to clean host tmp folder");
#endif
        return false;
    }
    return true;
}

}  // namespace khost
