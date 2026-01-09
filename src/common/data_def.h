#ifndef DATA_DEF_H
#define DATA_DEF_H

#include "rt_global.h"
#include "data_offset.h"

/// @brief info of the firmware
typedef struct CodeRange {
    uint32_t bin_start;
    uint32_t rewrite_start;
    uint32_t bin_end;
    uint32_t rewrite_end;
} __attribute__((packed, aligned(4))) CodeRange;

/// @brief global state of the firmware
typedef struct FirmwareState {
    // number of executed basic blocks
    uint32_t block_count;
    // max value of the basic blocks to be executed
    uint32_t block_limit;
    // number of executed basic blocks for interrupt trigger
    uint32_t block_next_irq;
    // id of the last executed basic block
    uint32_t block_last_id;
    // stack pointer of the runtime (unused)
    uint32_t runtime_sp;
    // next irq
    uint32_t pending_irq;
    // has mmio ranout
    uint32_t mmio_ranout;
    // coverage map of the firmware
    uint8_t coverage_map[COVERAGE_MAP_SIZE];
    // debug buffer of the firmware
    char debug_buffer[0x100];
} __attribute__((packed, aligned(4))) FirmwareState;

/// @brief context of the mcu
typedef struct FirmwareContext {
    uint32_t r0;
    uint32_t r1;
    uint32_t r2;
    uint32_t r3;
    uint32_t r4;
    uint32_t r5;
    uint32_t r6;
    uint32_t r7;
    uint32_t r8;
    uint32_t r9;
    uint32_t r10;
    uint32_t r11;
    uint32_t r12;
    uint32_t sp;
    uint32_t lr;
    uint32_t pc;
    uint32_t xpsr;
} __attribute__((packed, aligned(4))) FirmwareContext;

/// @brief vector info of the NVIC controler
typedef struct NvicVecInfo {
    // Exception priorities can range from -3 to 255; only the unmodifiable
    // priority values for RESET, NMI and HardFault can be negative.
    int16_t prio;
    uint8_t enabled;
    uint8_t pending;
    uint8_t active;
    uint8_t level;          // exceptions <=15 never set level
} __attribute__((packed, aligned(4))) NvicVecInfo;


/// @brief state of the NVIC controler
typedef struct NvicState {
    // number of irqs
    int num_irq;
    // state of irq vectors
    NvicVecInfo vectors[NVIC_MAX_VECTORS];
    // priority group in AIRCR
    uint8_t num_prio_bits;
    // higest prio pending enabled exception
    unsigned int vectpending;
    // group prio of the highest prio active exception
    int exception_prio;
    // group prior of the exception in vectpending
    int vectpending_prio;
	int irq_cnt; 
} __attribute__((aligned(4))) NvicState;


/// @brief state of the sysctl
typedef struct SysctlState {
    uint32_t vtor;
    uint32_t cpacr;
    uint32_t aircr;
    uint32_t fpccr;
    uint32_t ccr;
    uint32_t scr;
    uint32_t cfsr;      // Configurable Fault Status Register, CFSR 0xE000ED28
    uint32_t hfsr;      // HardFault Status Register, HFSR 0xE000ED2C
    uint32_t mmfar;
    uint32_t bfar;
} __attribute__((packed, aligned(4))) SysctlState;

/// @brief state of the emulated systick
typedef struct EmuSystickState {
    int enabled;
    int interrput;
    int core_lock;
    int count_flag;
    uint32_t reload_value;
    uint32_t last_block_cnt;
    uint32_t irq_cnt;
    uint32_t irq;
} __attribute__((packed, aligned(4))) EmuSystickState;

/// @brief state of the MCU
typedef struct McuState {
    // current mode
    uint32_t current_mode;
    
    // stack pointer
    // ref: https://developer.arm.com/documentation/ddi0403/d/System-Level-Architecture/System-Level-Programmers--Model/Registers/The-ARM-core-registers?lang=en
    uint32_t msp, psp;

    // special-purpose program status registers
    // ref: https://developer.arm.com/documentation/ddi0403/d/System-Level-Architecture/System-Level-Programmers--Model/Registers/The-special-purpose-program-status-registers--xPSR?lang=en
    uint32_t ipsr;
    
    // special-purpose mask registers
    // ref: https://developer.arm.com/documentation/ddi0403/d/System-Level-Architecture/System-Level-Programmers--Model/Registers/The-special-purpose-mask-registers?lang=en 
    uint32_t primask, faultmask, basepri;

    // special-purpose CONTROL register
    // ref: https://developer.arm.com/documentation/ddi0403/d/System-Level-Architecture/System-Level-Programmers--Model/Registers/The-special-purpose-CONTROL-register?lang=en
    uint32_t control;

} __attribute__((packed, aligned(4))) McuState;

/// @brief incompatible instrutions
typedef struct IncompatibleInsn {
    uint32_t addr;
    uint16_t type;
    union {
        // for MRS ans MSR
        struct { 
            uint16_t sysm;
            union {
                // for MRS
                uint16_t rd;
                // for MSR
                struct { uint16_t rn, mask; };
            };
        };
        // for CPS
        struct { uint16_t im, i, f; };
    };
} __attribute__((packed, aligned(4))) IncompatibleInsn;

/// fuzzware model types

#define FUZZWARE_BITEXTRACT     0
#define FUZZWARE_CONSTANT       1
#define FUZZWARE_PASSTHROUGH    2
#define FUZZWARE_SET            3
#define FUZZWARE_IDENTITY       4

/// @brief fuzzware model info
typedef struct FuzzwareModelInfo {
    uint32_t type;
    uint32_t access_size;
    uint32_t addr;
    uint32_t pc;
    uint32_t left_shift;
    uint32_t mask;
    uint32_t size;
    uint32_t val;
    uint32_t init_val;
    uint32_t val_cnt;
    uint32_t vals[RUNTIME_FUZZWARE_SET_SIZE];
} __attribute__((packed, aligned(4))) FuzzwareModelInfo;

typedef struct PassthroughNode {
    uint32_t addr;
    uint32_t value;
} __attribute__((packed, aligned(4))) PassthroughNode;

typedef struct AddressRange {
    uint32_t start;
    uint32_t end;
} __attribute__((packed, aligned(4))) AddressRange;

typedef struct HalDescriptor {
    uint32_t addr;
    const char *name;
} __attribute__((packed, aligned(4))) HalDescriptor;

#endif  // DATA_DEF_H
