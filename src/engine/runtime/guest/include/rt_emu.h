#ifndef RT_EMU_H
#define RT_EMU_H

#include "rt_global.h"

// SPSR_ELx bits for exceptions taken from AArch32
#define PSR_AA32_MODE_MASK	0x0000001f
#define PSR_AA32_MODE_USR	0x00000010
#define PSR_AA32_MODE_FIQ	0x00000011
#define PSR_AA32_MODE_IRQ	0x00000012
#define PSR_AA32_MODE_SVC	0x00000013
#define PSR_AA32_MODE_ABT	0x00000017
#define PSR_AA32_MODE_HYP	0x0000001a
#define PSR_AA32_MODE_UND	0x0000001b
#define PSR_AA32_MODE_SYS	0x0000001f
#define PSR_AA32_T_BIT		0x00000020
#define PSR_AA32_F_BIT		0x00000040
#define PSR_AA32_I_BIT		0x00000080
#define PSR_AA32_A_BIT		0x00000100
#define PSR_AA32_E_BIT		0x00000200
#define PSR_AA32_PAN_BIT	0x00400000
#define PSR_AA32_SSBS_BIT	0x00800000
#define PSR_AA32_DIT_BIT	0x01000000
#define PSR_AA32_Q_BIT		0x08000000
#define PSR_AA32_V_BIT		0x10000000
#define PSR_AA32_C_BIT		0x20000000
#define PSR_AA32_Z_BIT		0x40000000
#define PSR_AA32_N_BIT		0x80000000
// If-Then execution state mask
#define PSR_AA32_IT_MASK	0x0600fc00	
#define PSR_AA32_GE_MASK	0x000f0000

// Low-level stepping controls
#define DBG_MDSCR_SS		(1 << 0)
#define DBG_SPSR_SS		    (1 << 21)

void emulate_inst(uint32_t addr, uint32_t dfar);

#endif  // RT_EMU_H