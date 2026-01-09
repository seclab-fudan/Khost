#ifndef RT_GLOBAL_H
#define RT_GLOBAL_H

#include "global.h"
#include "stdint.h"

// count of the incompatible instructions in firmware
#define RUNTIME_INCOMPATIBLE_INSN_CNT   512

// count of the fuzzware model
#define RUNTIME_FUZZWARE_MODEL_CNT      10000

/// size of the models allocated to the same address
#define MAX_FUZZWARE_LUT_PRE_ADDR_SIZE  10000

// max size of the fuzzware set model
#define RUNTIME_FUZZWARE_SET_SIZE       8

// buffer for the passthrough
#define RUNTIME_PASSTHROUGH_BUFFER_SIZE 10000

// buffer for the input
#define RUNTIME_INPUT_BUFFER_SIZE       0x10000

// buffer for the address range
#define RUNTIME_ADDRESS_RANGE_SIZE      100

// max size of the RTOS task
#define RUNTIME_MAX_TASK_CNT            128

// period to check irq
#define RUNTIME_IRQ_CHECK_PERIOD        500

// period between irq
#define RUNTIME_IRQ_TRIGGER_PERIOD      500


#define MAKE_64BIT_MASK(shift, length) \
    (((~0ULL) >> (64 - (length))) << (shift))

#define FIELD(reg, field, shift, length)                                  \
    enum { SC_ ## reg ## _ ## field ## _SHIFT = (shift)};                  \
    enum { SC_ ## reg ## _ ## field ## _LENGTH = (length)};                \
    enum { SC_ ## reg ## _ ## field ## _MASK = MAKE_64BIT_MASK(shift, length)};

// Interrupt Controller Type Register
#define SC_REG_ICTR             0xE000E004
// CPUID Base Register (handled)
#define SC_REG_CPUID            0xE000ED00
// Interrupt Control and State Register (handled)
#define SC_REG_ICSR             0xE000ED04
FIELD(ICSR, VECTACTIVE, 0, 9)
FIELD(ICSR, RETTOBASE, 11, 1)
FIELD(ICSR, VECTPENDING, 12, 9)
FIELD(ICSR, ISRPENDING, 22, 1)
FIELD(ICSR, ISRPREEMPT, 23, 1)
FIELD(ICSR, PENDSTCLR, 25, 1)
FIELD(ICSR, PENDSTSET, 26, 1)
FIELD(ICSR, PENDSVCLR, 27, 1)
FIELD(ICSR, PENDSVSET, 28, 1)
FIELD(ICSR, NMIPENDSET, 31, 1)
// Vector Table Offset Register (handled)
#define SC_REG_VTOR             0xE000ED08
// Application Interrupt and Reset Control Register (handled)
#define SC_REG_AIRCR            0xE000ED0C
FIELD(AIRCR, VECTRESET, 0, 1)
FIELD(AIRCR, VECTCLRACTIVE, 1, 1)
FIELD(AIRCR, SYSRESETREQ, 2, 1)
FIELD(AIRCR, PRIGROUP, 8, 3)
FIELD(AIRCR, ENDIANNESS, 15, 1)
FIELD(AIRCR, VECTKEY, 16, 16)
// System Control Register
#define SC_REG_SCR              0xE000ED10
FIELD(SCR, SLEEPONEXIT, 1, 1)
FIELD(SCR, SLEEPDEEP, 2, 1)
FIELD(SCR, SEVONPEND, 4, 1)
// Configuration and Control Register (handled)
#define SC_REG_CCR              0xE000ED14
FIELD(CCR, NONBASETHRDENA, 0, 1)
FIELD(CCR, USERSETMPEND, 1, 1)
FIELD(CCR, UNALIGN_TRP, 3, 1)
FIELD(CCR, DIV_0_TRP, 4, 1)
FIELD(CCR, BFHFNMIGN, 8, 1)
FIELD(CCR, STKALIGN, 9, 1)
FIELD(CCR, DC, 16, 1)
FIELD(CCR, IC, 17, 1)
FIELD(CCR, BP, 18, 1)
// System Handler Priority Register 1 (handled)
#define SC_REG_SHPR1            0xE000ED18
// System Handler Priority Register 2 (handled)
#define SC_REG_SHPR2            0xE000ED1C
// System Handler Priority Register 3 (handled)
#define SC_REG_SHPR3            0xE000ED20
// System Handler Control and State Register
#define SC_REG_SHCSR            0xE000ED24
// Configurable Fault Status Register [BLANK]
#define SC_REG_CFSR             0xE000ED28
// HardFault Status Register [BLANK]
#define SC_REG_HFSR             0xE000ED2C
// Debug Fault Status Register [BLANK]
#define SC_REG_DFSR             0xE000ED30
// MemManage Fault Address Register [BLANK]
#define SC_REG_MMFAR            0xE000ED34
// BusFault Address Register [BLANK]
#define SC_REG_BFAR             0xE000ED38
// Auxiliary Fault Status Register [BLANK]
#define SC_REG_AFSR             0xE000ED3C
// Coprocessor Access Control Register (handled)
#define SC_REG_CPACR            0xE000ED88
// Floating Point Context Control Register (handled)
#define SC_REG_FPCCR            0xE000EF34
// Floating Point Context Address Register [BLANK]
#define SC_REG_FPCAR            0xE000EF38
// Floating Point Default Status Control Register [BLANK]
#define SC_REG_FPDSCR           0xE000EF3C
// Media and FP Feature Register 0
#define SC_REG_MVFR0            0xE000EF40
// Media and FP Feature Register 1
#define SC_REG_MVFR1            0xE000EF44
// Media and FP Feature Register 2
#define SC_REG_MVFR2            0xE000EF48


#endif  // RT_GLOBAL_H
