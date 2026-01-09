#ifndef EN_GLOBAL_H
#define EN_GLOBAL_H

#include "global.h"

#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <assert.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/stddef.h>
#include <linux/kvm.h>
#include <strings.h>
#include <mutex>

namespace khost {

#define ESR_ELx_EC_SHIFT	(26)
#define ESR_ELx_EC_WIDTH	(6)
#define ESR_ELx_EC_MASK		(uint32_t(0x3F) << ESR_ELx_EC_SHIFT)
#define ESR_ELx_EC(esr)		(((esr) & ESR_ELx_EC_MASK) >> ESR_ELx_EC_SHIFT)

char *sysreg_name(int id);    // converts system register id to name
uint64_t general_reg_id(int id);     // get kvm reg id for R_{id}

////////////////////////////////////////////////////////////////////////////////
// Globals 
////////////////////////////////////////////////////////////////////////////////
extern std::mutex print_mutex;

inline int64_t get_clock_realtime() {
    struct timespec ts;
    timespec_get(&ts, TIME_UTC);
    return ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

////////////////////////////////////////////////////////////////////////////////
// System Control Block
////////////////////////////////////////////////////////////////////////////////
#define MAKE_64BIT_MASK(shift, length) \
    (((~0ULL) >> (64 - (length))) << (shift))

#define FIELD(reg, field, shift, length)                                  \
    enum { SC_ ## reg ## _ ## field ## _SHIFT = (shift)};                  \
    enum { SC_ ## reg ## _ ## field ## _LENGTH = (length)};                \
    enum { SC_ ## reg ## _ ## field ## _MASK = MAKE_64BIT_MASK(shift, length)};

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

}  // namespace khost

#endif  // EN_GLOBAL_H