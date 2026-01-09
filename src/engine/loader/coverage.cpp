#include "en_global.h"
#include "coverage.h"
#include "loader.h"
#include "data_def.h"
#if KVM_OPEN_DEBUG
    #include "autogen_rt_full_debug.h"
    #include "autogen_rt_para_debug.h"
#else
    #include "autogen_rt_full_normal.h"
    #include "autogen_rt_para_normal.h"
#endif

namespace khost {

std::string COVERAGE_ASM = R"delimiter(
    # add block count
    movw    r0, #0x%08x;
    movt    r0, #0x%08x;
    ldr     r1, [r0];
    add     r1, #1;
    str     r1, [r0];
    
    # check max block count, if reach the limit, exit execution
    movw    r0, #0x%08x;
    movt    r0, #0x%08x; 
    ldr     r2, [r0];
    cmp     r1, r2;
    blt     limit_not_reach;
    bkpt    #8;

limit_not_reach:
    # check irq trigger (RUNTIME_should_irq_trigger)
    push    {r12, lr};
    movw    r0, #0x%08x;
    movt    r0, #0x%08x;
    blx     r0;
    pop     {r12, lr};
    cmp     r0, #0x1;
    bne     irq_not_reach;
    svc     #0xff;

irq_not_reach:
    # obtain prev_location
    movw    r0, #0x%08x;
    movt    r0, #0x%08x;
    ldr     r1, [r0];

    # curr_location XOR prev_locaton
    movw    r2, #0x%08x;
    movt    r2, #0x%08x;
    eor     r1, r1, r2;

    # update prev_location
    lsr     r2, r2, #1;
    str     r2, [r0];

    # mod the max size of coverage map
    movw    r0, #0x%08x;
    movt    r0, #0x%08x;
    and     r1, r1, r0;

    # obtain coverage map
    movw    r3, #0x%08x;
    movt    r3, #0x%08x;
    ldrb    r2, [r3, r1];
    add     r2, r2, #1;
    strb    r2, [r3, r1];
)delimiter";

void *CoverageAddition::bytecode() {
    uint32_t rt_firmware_state = (CONFIG.mode() == FULL_MODE ? RT_FULL_firmware_state : RT_PARA_firmware_state);
    uint32_t block_count = rt_firmware_state + OFFSET_BLOCK_COUNT;
    uint32_t block_limit = rt_firmware_state + OFFSET_BLOCK_LIMIT;
    uint32_t instr_ctrl_addr = rt_firmware_state + OFFSET_BLOCK_LAST_ID;
    uint32_t current_location = (uint32_t)(addr() * 0x517cc1b727220a95ll);
    uint32_t coverage_map = rt_firmware_state + OFFSET_COVERAGE_MAP;
    
    uint32_t rt_should_irq_trigger = (CONFIG.mode() == FULL_MODE ? RT_FULL_should_irq_trigger : RT_PARA_should_irq_trigger);
    uint32_t helper_func = rt_should_irq_trigger|1;

    char asm_buffer[2048];
    sprintf(asm_buffer, COVERAGE_ASM.c_str(),
            word_low(block_count), word_high(block_count),
            word_low(block_limit), word_high(block_limit),
            word_low(helper_func), word_high(helper_func),
            word_low(instr_ctrl_addr), word_high(instr_ctrl_addr),
            word_low(current_location), word_high(current_location),
            word_low(COVERAGE_MAP_SIZE-1), word_high(COVERAGE_MAP_SIZE-1),
            word_low(coverage_map), word_high(coverage_map)
    );

    size_t size, count;
    unsigned char *ks_bytes;
    if (ks_asm(_loader->ks_thumb(), asm_buffer, 0, &ks_bytes, &size, &count) != KS_ERR_OK) {
        printf("failed to compile coverage\n");
        exit(-1);
    }
    void *code_part_buffer = mmap(NULL, size, 
                                    PROT_READ | PROT_WRITE, 
                                    MAP_SHARED | MAP_ANONYMOUS, -1, 0);
    memcpy(code_part_buffer, ks_bytes, size);
    ks_free(ks_bytes);
    return code_part_buffer;
}

uint32_t CoverageAddition::calc_size() {
    char asm_buffer[2048];
    sprintf(asm_buffer, COVERAGE_ASM.c_str(), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

    size_t size, count;
    unsigned char *ks_bytes;
    if (ks_asm(_loader->ks_thumb(), asm_buffer, 0, &ks_bytes, &size, &count) != KS_ERR_OK) {
        printf("failed to compile coverage\n");
        exit(-1);
    }
    ks_free(ks_bytes);
    return size;
}

void CoverageAddition::set_addr(uint32_t addr) {
    Addition::set_addr(addr);
}

}  // namespace khost
