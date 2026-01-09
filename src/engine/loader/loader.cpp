#include "en_global.h"
#include "loader.h"
#include "en_board.h"
#include "en_config.h"
#if KVM_OPEN_DEBUG
    #include "autogen_rt_full_debug.h"
    #include "autogen_rt_para_debug.h"
#else
    #include "autogen_rt_full_normal.h"
    #include "autogen_rt_para_normal.h"
#endif


#include <vector>
#include <map>

namespace khost {

Loader::Loader() {
    // start keystone and capstone engine 
    open_engine();
    _additions.clear();
}

Loader::~Loader() {
    for (auto addition: _additions) {
        delete addition;
    }
    close_engine();
}

void Loader::load() {
    // read code binary, get firmware_code_size
    int code_bin_fd = open(CONFIG.frimware_path().c_str(), O_RDONLY);
    if (code_bin_fd < 0) {
        printf("failed to open firmware binary: %s\n", CONFIG.frimware_path().c_str());
        abort();
    }
    void *code_bin_buffer = mmap(NULL, ROM_OR_FLASH_SIZE, 
                                 PROT_READ | PROT_WRITE, 
                                 MAP_SHARED | MAP_ANONYMOUS, -1, 0);
    if (code_bin_buffer == MAP_FAILED) {
        printf("failed to alloc code buffer");
        munmap(code_bin_buffer, ROM_OR_FLASH_SIZE);
        abort();
    }
    uint32_t code_bin_size = read(code_bin_fd, code_bin_buffer, ROM_OR_FLASH_SIZE);
    close(code_bin_fd);
    if (code_bin_size == (ssize_t)-1) {
        printf("failed to read binary code");
        munmap(code_bin_buffer, ROM_OR_FLASH_SIZE);
        abort();
    }
    CONFIG.firmware_info()->firmware_code_size = code_bin_size;

    // get basic information of the firmware
    // get load_base and num_irq
    uint32_t load_base = calc_load_base(code_bin_buffer, code_bin_size);
    if (load_base == (uint32_t)-1) {
        printf("failed to find load base\n");
        munmap(code_bin_buffer, ROM_OR_FLASH_SIZE);
        abort();
    }
    CONFIG.firmware_info()->load_base = load_base;
    // get initial_sp
    uint32_t initial_sp = ((uint32_t *)code_bin_buffer)[0];
    if (initial_sp < SRAM_START || initial_sp > SRAM_END) {
        printf("initial sp 0x%x is not correct\n", initial_sp);
        munmap(code_bin_buffer, ROM_OR_FLASH_SIZE);
        abort();
    }
    CONFIG.firmware_info()->initial_sp = initial_sp;
    // get initial_pc
    uint32_t initial_pc = ((uint32_t *)code_bin_buffer)[1];
    CONFIG.firmware_info()->initial_pc = initial_pc;

    // find instructions that only exist in ARMv7-M profile
    find_and_patch_incompatible_insns(code_bin_buffer, code_bin_size);

    // copy the code to the memory backend
    if (load_base + code_bin_size > ROM_OR_FLASH_SIZE) {
        printf("ERROR | khost: load base 0x%x is not correct\n", load_base);
        munmap(code_bin_buffer, ROM_OR_FLASH_SIZE);
        abort();
    }

#ifndef RUN_PROFILE
    for (auto block: *CONFIG.basic_blocks()) {
        _additions.push_back(
            new TrampolineAddition(this, code_bin_buffer, block)
        );
    }
#endif

    // get and calculate the addition size
    uint32_t addition_size = 0;
    for (auto addition: _additions) {
        addition_size += addition->size();
    }
    CONFIG.firmware_info()->addition_size = addition_size;

    // get the addition load base
    uint32_t addition_base = calc_addition_base(load_base, code_bin_size, addition_size);
    CONFIG.firmware_info()->addition_start = addition_base;
    uint32_t addition_addr = addition_base;
    for (auto addition: _additions) {
        addition->set_addr(addition_addr);
        addition_addr += addition->size();
    }

    // start to craft additions
    for (auto addition: _additions) {
        void *byte_code = addition->bytecode();
        if (byte_code == nullptr) {
            continue;
        }
        CONFIG.add_load_info(
            new LoadInfo(byte_code, addition->size(), addition->addr())
        );
    }
    void *binary_buffer = mmap(NULL, code_bin_size, 
                               PROT_READ | PROT_WRITE, 
                               MAP_SHARED | MAP_ANONYMOUS, -1, 0);
    memcpy(binary_buffer, code_bin_buffer, code_bin_size);
    CONFIG.add_load_info(new LoadInfo(binary_buffer, code_bin_size, load_base));

    // add runtime to load info
    uint8_t *rt_binary = CONFIG.mode() == FULL_MODE ? RT_FULL_binary : RT_PARA_binary;
    uint32_t rt_binary_size = CONFIG.mode() == FULL_MODE ? RT_FULL_binary_size : RT_PARA_binary_size;
    CONFIG.add_load_info(new LoadInfo(rt_binary, rt_binary_size, RUNTIME_START));

    // exit
    munmap(code_bin_buffer, ROM_OR_FLASH_SIZE);
}

void Loader::open_engine() {
    ks_err ks_e;
    cs_err cs_e;

    // open ARM keystone engine
    ks_e = ks_open(KS_ARCH_ARM, 
                   KS_MODE_ARM + KS_MODE_LITTLE_ENDIAN + KS_MODE_V8, 
                   &_ks_arm);
    if (ks_e != KS_ERR_OK) {
        printf("failed to load arm keystone engine");
        exit(-1);
    }
    // open THUMB keystone engine
    ks_e = ks_open(KS_ARCH_ARM, KS_MODE_THUMB, &_ks_thumb);
    if (ks_e != KS_ERR_OK) {
        printf("failed to load thumb keystone engine");
        exit(-1);
    }
    // open capstone engine
    cs_e = cs_open(CS_ARCH_ARM, CS_MODE_THUMB, &_cs);
    if (cs_e != CS_ERR_OK) {
        printf("failed to load thumb capstone engine");
        exit(-1);
    }
    cs_option(_cs, CS_OPT_DETAIL, CS_OPT_ON);
}

void Loader::close_engine() {
    ks_close(_ks_arm);
    ks_close(_ks_thumb);
    cs_close(&_cs);
}

uint32_t Loader::calc_load_base(void *buffer, ssize_t size) {
    // calc possible vector table
    int num_irq = 0;
    std::vector<uint32_t> vector_table;
    for (int i = 1; i < 512; i++) {
        if (i < 16 && (i+1) * 0x4 > size) {
            printf("code binary too short\n");
            exit(-1);
        }
        uint32_t entry = ((uint32_t *)buffer)[i];
        if (entry == 0)
            vector_table.push_back(entry - 1);
        else { 
            if ((entry & 1) != 1 || entry > ROM_OR_FLASH_END) {
                if (i < 16) {
                    printf("invalid vector table entry\n");
                    exit(-1);
                }
                break;
            }
            vector_table.push_back(entry - 1);
        }
    }
    num_irq = vector_table.size();
    CONFIG.firmware_info()->num_irq = num_irq; 

    // calc possible dummy function
    std::vector<uint32_t> possible_function;
    for (uint64_t i = (num_irq+1) * 0x4; i < (uint64_t)size; i += 0x2) {
        uint16_t insn = *((uint16_t *)((uint64_t)buffer + i));
        if (insn == 0x4770 || insn == 0xe7fe)
            possible_function.push_back(i);
        if (i % 0x4 == 0) {
            uint32_t insn = *((uint32_t *)((uint64_t)buffer + i));
            if (insn == 0xbffef7ff || insn == 0x0107f02d)
                possible_function.push_back(i);
        }
    }

    // calc most possible load base
    std::map<uint32_t, int> load_base_hit_map;
    for (auto addr = vector_table.begin(); addr != vector_table.end(); ++addr) {
        if (*addr == 0)
            continue;
        for (auto offset = possible_function.begin(); 
             offset != possible_function.end(); ++offset) { 
            if (*addr < *offset)
                continue;
            uint32_t delta = (*addr) - (*offset);
            if (delta % 0x1000 != 0)
                continue;
            if (load_base_hit_map.find(delta) == load_base_hit_map.end())
                load_base_hit_map[delta] = 1;
            else
                load_base_hit_map[delta]++;
        }
    }
    int max_hit = 0;
    uint32_t max_hit_load_base = 0;
    for (auto i = load_base_hit_map.begin(); i != load_base_hit_map.end(); ++i) {
        if (i->second > max_hit) {
            max_hit = i->second;
            max_hit_load_base = i->first;
        }
    }
    if (max_hit < 16) {
        printf("ERROR | khost: load base 0x%x may not correct\n", max_hit_load_base);
    }

    return max_hit_load_base;
}

static inline uint32_t padding_to_align(uint32_t start, uint32_t size) {
    return start + size + 0x3fff - ((start + size + 0x3fff) % 0x4000);
}

uint32_t Loader::calc_addition_base(uint32_t firmware_base, 
                                    uint32_t firmware_size, 
                                    uint32_t addition_size) {
    uint32_t wrapper_base = -1;
    uint32_t padding_end = padding_to_align(firmware_base, firmware_size);
    if (addition_size <= firmware_base) {
        wrapper_base = 0;
        // as near as possible
        while (wrapper_base + 0x4000 + addition_size <= firmware_base) {
            wrapper_base += 0x4000;
        }
    }
    else if (padding_end + addition_size <= ROM_OR_FLASH_END)
        wrapper_base = padding_end;
    return wrapper_base;
}

// find_and_patch_incompatible_insns
// buffer: pointer to the binary, size: size in bytes
// 
// Finds incompatible instructions in the firmware, e.g. MRS, MSR and CPS.
// Searchs the binary with the unit of uint16 and checks the instructions by 
// encoding. Incompatible instructions will be add to the map of the Board and 
// will be rewrite to SVC instruction in memory. 
void Loader::find_and_patch_incompatible_insns(void *buffer, ssize_t size) {
    // treat the buffer as thumb code buffer
    uint16_t *thumb_insn_buffer = (uint16_t *)buffer;
    ssize_t thumb_insn_size = size >> 1;

    // encoding for MSR under ARMv7-M
    // ref: https://developer.arm.com/documentation/ddi0403/d/System-Level-Architecture/System-Instruction-Details/ARMv7-M-system-instruction-descriptions/MSR?lang=en
    const uint32_t thumb_MSR_Part1_mask  = 0b1111111111110000;
    const uint32_t thumb_MSR_Part1_value = 0b1111001110000000;
    const uint32_t thumb_MSR_Part2_mask  = 0b1111001100000000;
    const uint32_t thumb_MSR_Part2_value = 0b1000000000000000;

    // encoding for MRS under ARMv7-M
    // ref: https://developer.arm.com/documentation/ddi0403/d/System-Level-Architecture/System-Instruction-Details/ARMv7-M-system-instruction-descriptions/MRS?lang=en
    const uint32_t thumb_MRS_Part1_mask  = 0b1111111111111111;
    const uint32_t thumb_MRS_Part1_value = 0b1111001111101111;
    const uint32_t thumb_MRS_Part2_mask  = 0b1111000000000000;
    const uint32_t thumb_MRS_Part2_value = 0b1000000000000000;

    // encoding for CPS under ARMv7-M
    // ref: https://developer.arm.com/documentation/ddi0403/d/System-Level-Architecture/System-Instruction-Details/ARMv7-M-system-instruction-descriptions/CPS?lang=en
    const uint32_t thumb_CPS_mask  = 0b1111111111101100;
    const uint32_t thumb_CPS_value = 0b1011011001100000;
        
    for (uint32_t i = 0; i < thumb_insn_size; i++) {
        // check we are in basic block
        bool found = false;
        for (auto block: *CONFIG.basic_blocks()) {
            uint32_t pc = CONFIG.firmware_info()->load_base + (i << 1);
            if (pc >= block.start && pc < block.end) {
                found = true;
                break;
            }
        }
        if (!found) {
            continue;
        }

        // find incompatible MSR instruction
        if ((thumb_insn_buffer[i] & thumb_MSR_Part1_mask) 
                == thumb_MSR_Part1_value 
            && (thumb_insn_buffer[i+1] & thumb_MSR_Part2_mask) 
                == thumb_MSR_Part2_value) {
            // decode MSR instruction
            uint32_t rn   = thumb_insn_buffer[i] & 0b1111;
            uint32_t mask = (thumb_insn_buffer[i+1] & 0b0000110000000000) >> 10;
            uint32_t sysm = thumb_insn_buffer[i+1] & 0b11111111;

            // adds this msr to the map in board, board frees the pointer
            CONFIG.add_incompatible_insn(
                CONFIG.firmware_info()->load_base + (i << 1),
                new_msr(rn, sysm, mask)
            );
            
            // rewrite to SVC #0xfe or BKPT #0x1
#ifndef RUN_PROFILE
            thumb_insn_buffer[i] = 0xdffe;
            thumb_insn_buffer[i+1] = 0xbf00;
#else
            thumb_insn_buffer[i] = BKPT_INCOMP_INSN;
            thumb_insn_buffer[i+1] = 0xbf00;
#endif
        }
        
        // find incompatible MRS instruction
        if ((thumb_insn_buffer[i] & thumb_MRS_Part1_mask) 
                == thumb_MRS_Part1_value 
            && (thumb_insn_buffer[i+1] & thumb_MRS_Part2_mask) 
                == thumb_MRS_Part2_value) {
            // decode MRS instruction
            uint32_t rd   = (thumb_insn_buffer[i+1] & 0b111100000000) >> 8;
            uint32_t sysm = thumb_insn_buffer[i+1] & 0b11111111;
            
            // adds this mrs to the map in board, board frees the pointer
            CONFIG.add_incompatible_insn(
                CONFIG.firmware_info()->load_base + (i << 1),
                new_mrs(rd, sysm)
            );

            // rewrite to SVC #0xfe or BKPT #0x1
#ifndef RUN_PROFILE
            thumb_insn_buffer[i] = 0xdffe; 
            thumb_insn_buffer[i+1] = 0xbf00;
#else
            thumb_insn_buffer[i] = BKPT_INCOMP_INSN;
            thumb_insn_buffer[i+1] = 0xbf00;    
#endif
        }

        // find incompatible CPS instruction
        if ((thumb_insn_buffer[i] & thumb_CPS_mask) == thumb_CPS_value) {
            // decode CPS instruction
            uint32_t im = (thumb_insn_buffer[i] & 0b10000) >> 4;
            uint32_t iv = (thumb_insn_buffer[i] & 0b10) >> 1;
            uint32_t fv = (thumb_insn_buffer[i] & 0b1);

            // adds this cps to the map in board, board frees the pointer
            CONFIG.add_incompatible_insn(
                CONFIG.firmware_info()->load_base + (i << 1),
                new_cps(im, iv, fv)
            );

            // rewrite to SVC #0xfe or BKPT #0x1
#ifndef RUN_PROFILE
            thumb_insn_buffer[i] = 0xdffe;
#else
            thumb_insn_buffer[i] = BKPT_INCOMP_INSN;
#endif
        }

        // find incompatible WFI instruction
        if (thumb_insn_buffer[i] == 0xbf30) {
            // rewrite to SVC #0xfd: trigger the interrupt
#ifndef RUN_PROFILE
            thumb_insn_buffer[i] = 0xdffd;
#endif

            // rewrite to NOP: skip the wfi (fuzzware use this)
            // thumb_insn_buffer[i] = 0xbf00;
        }
    }
}

}  // namespace khost
