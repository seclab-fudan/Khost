#include "rt_global.h"
#include "rt_emu.h"
#include "rt_data.h"
#include "rt_debug.h"
#include "rt_cpu.h"
#include "rt_handler.h"
#include "rt_fuzzware.h"
#include "rt_systick.h"
#include "rt_sysctl.h"

#define DEBUG_CHECK_DFAR    1

// align with ARM document
#define SRType_LSL          0
#define TRUE                1
#define FALSE               0

// instruction types
#define INS_UNK             0
#define STR_REG             1
#define STR_IMM             2
#define STRH_REG            3
#define STRH_IMM            4
#define STRB_REG            5
#define STRB_IMM            6
#define LDR_REG             7
#define LDR_IMM             8
#define LDR_LIT             9
#define LDRH_REG            10
#define LDRH_IMM            11
#define LDRH_LIT            12
#define LDRSH_REG           13
#define LDRSH_IMM           14
#define LDRSH_LIT           15
#define LDRB_REG            16
#define LDRB_IMM            17
#define LDRB_LIT            18
#define LDRSB_REG           19
#define LDRSB_IMM           20
#define LDRSB_LIT           21

// handle result
#define DABT_UNHANDLED      0
#define DABT_HANDLED        1
#define DABT_ERROR          2

// instruction types
#define INSN_THUMB_16       1
#define INSN_THUMB_32       2

#if DEBUG_CHECK_DFAR
static uint32_t cur_dfar;
#endif

/// @brief gets type of the instruction in the firmware
/// @param addr guest physical address
/// @return type of the instruction
/// @ref https://developer.arm.com/documentation/ddi0403/d/Application-Level-Architecture/The-Thumb-Instruction-Set-Encoding/Thumb-instruction-set-encoding
static inline int insn_type(uint32_t addr) {
    uint32_t insn_opcode = *(uint32_t *)addr;

    /* A Thumb instructionL opcode[15:11] == 0b11101, 0b11110, 0b11111 is 32bit */
    if (((insn_opcode >> 11) & 0b11111) == 0b11101 ||
        ((insn_opcode >> 11) & 0b11111) == 0b11110 ||
        ((insn_opcode >> 11) & 0b11111) == 0b11111) {
        return INSN_THUMB_32;
    } else {
        return INSN_THUMB_16;
    }
}
static inline int insn_type(uint32_t) __attribute__((always_inline));

/// @brief adjust the if-then state of the firmware
static inline void adjust_itstate() {
    uint32_t itbits, cond;
	uint32_t cpsr = RUNTIME_firmware_context.xpsr;
	int is_arm = !(cpsr & PSR_AA32_T_BIT);

	if (is_arm || !(cpsr & PSR_AA32_IT_MASK))
		return;

	cond = (cpsr & 0xe000) >> 13;
	itbits = (cpsr & 0x1c00) >> (10 - 2);
	itbits |= (cpsr & (0x3 << 25)) >> 25;

	// Perform ITAdvance (see page A2-52 in ARM DDI 0406C)
	if ((itbits & 0x7) == 0)
		itbits = cond = 0;
	else
		itbits = (itbits << 1) & 0x1f;

	cpsr &= ~PSR_AA32_IT_MASK;
	cpsr |= cond << 13;
	cpsr |= (itbits & 0x1c) << (10 - 2);
	cpsr |= (itbits & 0x3) << 25;
	RUNTIME_firmware_context.xpsr = cpsr;
}
static inline void adjust_itstate() __attribute__((always_inline));

/// @brief skip the instruction when we have emulated it in the runtime
inline static void skip_instr() {
    // increase program counter
    uint32_t pc = RUNTIME_firmware_context.pc;
    int is_thumb;

	is_thumb = !!(RUNTIME_firmware_context.xpsr & PSR_AA32_T_BIT);
	if (is_thumb && insn_type(pc) == INSN_THUMB_16) {
		pc += 2;
    } else {
		pc += 4;
    }
    RUNTIME_firmware_context.pc = pc;

    // adjust if-then state
    adjust_itstate();

    // set the single step debug bit
    RUNTIME_firmware_context.xpsr &= ~DBG_SPSR_SS;
}
static inline void skip_instr() __attribute__((always_inline));

/// @brief emulate mmio memory write
/// @param addr memory address
/// @param len memory access length
/// @param value value to write
static void write_memory(uint32_t addr, uint32_t len, uint32_t value) {
    uint32_t pc = RUNTIME_firmware_context.pc;

    // mmio region for devices
    if ((0x40000000 <= addr && addr <= 0x5fffffff) ||
        (0xa0000000 <= addr && addr <= 0xdfffffff)) {
        // we check if the mmio access is valid
        if (unlikely(
            (len == 2 && addr % 2 != 0) || (len == 4 && addr % 4 != 0)
        )) {
#if KVM_OPEN_DEBUG
            if (unlikely(should_output(RT_OUTPUT_EMU))) {
                debug_clear();
                debug_append_str("invalid mmio access: mmio access not aligned\n");
                debug_print();
            }
#endif
            handler_dabt_fail();
        }
#if KVM_OPEN_DEBUG
        #if DEBUG_CHECK_DFAR
            if (unlikely(addr != cur_dfar)) {
                debug_clear();
                debug_append_str("[guest-emu] dfar unmatch when write: pc=");
                debug_append_int(pc);
                debug_append_str(", addr=");
                debug_append_int(addr);
                debug_append_str(", dfar=");
                debug_append_int(cur_dfar);
                debug_append_str("\n");
                debug_print();
                runtime_abort();
            }
        #endif
#endif
        fuzzware_handle_mmio_write(addr, pc, &value, len);
        return;
    }
    
    // mmio region for system
    else if (0xe0000000 <= addr && addr <= 0xe00fffff) {
        // mmio region for systick
        if (SYS_TICK_START <= addr && addr <= SYS_TICK_END) {
            int ret = systick_handle_mmio_write(addr, pc, &value, len);
            if (ret != 0) {
                handler_dabt_fail();
            }
            return;
        }
        // mmio region for nvic
        else if (NVIC_START <= addr && addr <= NVIC_END) {
            int ret = nvic_handle_mmio_write(addr, pc, &value, len);
            if (ret != 0) {
                handler_dabt_fail();
            }      
            return;
        }
        // mmio region for system control block
        else if ((SCB_START <= addr && addr <= SCB_END) ||
            (0xe000e000 <= addr && addr <= 0xe000e00f) ||
            (0xe000edf0 <= addr && addr <= 0xe000eeff) ||
            (0xe000ef00 <= addr && addr <= 0xe000ef4f) ||
            (0xe000ef50 <= addr && addr <= 0xe000ef8f) ||
            (0xe000ef90 <= addr && addr <= 0xe000efcf) || 
            (0xe000efd0 <= addr && addr <= 0xe000efff)) {
            int ret = sysctl_handle_mmio_write(addr, pc, &value, len);
            if (ret != 0) {
                handler_dabt_fail();
            }
            return;
        }
    }

    // other region
    handler_dabt_fail();
}

/// @brief emulate mmio memory read
/// @param addr memory address
/// @param len memory access length
/// @return read value 
static uint32_t read_memory(uint32_t addr, uint32_t len) {
    uint32_t pc = RUNTIME_firmware_context.pc;
  
    // mmio region for devices
    if ((0x40000000 <= addr && addr <= 0x5fffffff) ||
        (0xa0000000 <= addr && addr <= 0xdfffffff)) {
        // we check if the mmio access is valid
        if (unlikely(
            (len == 2 && addr % 2 != 0) || (len == 4 && addr % 4 != 0)
        )) {
#if KVM_OPEN_DEBUG
            if (unlikely(should_output(RT_OUTPUT_EMU))) {
                debug_clear();
                debug_append_str("invalid mmio access: mmio access not aligned\n");
                debug_print();
            }
#endif
            handler_dabt_fail();
        }
#if KVM_OPEN_DEBUG
        #if DEBUG_CHECK_DFAR
            if (unlikely(addr != cur_dfar)) {
                debug_clear();
                debug_append_str("[guest-emu] dfar unmatch when read: pc=");
                debug_append_int(pc);
                debug_append_str(", addr=");
                debug_append_int(addr);
                debug_append_str(", dfar=");
                debug_append_int(cur_dfar);
                debug_append_str("\n");
                debug_print();
                runtime_abort();
            }
        #endif  
#endif
        uint32_t value = 0;
        fuzzware_handle_mmio_read(addr, pc, &value, len);
        return value;
    }

    // mmio region for system
    else if (0xe0000000 <= addr && addr <= 0xe00fffff) {
        // mmio region for systick
        if (SYS_TICK_START <= addr && addr <= SYS_TICK_END) {
            uint32_t value = 0;
            int ret = systick_handle_mmio_read(addr, pc, &value, len);
            if (ret != 0) {
                handler_dabt_fail();
            }
            return value;
        }
        // mmio region for nvic
        else if (NVIC_START <= addr && addr <= NVIC_END) {
            uint32_t value = 0;
            int ret = nvic_handle_mmio_read(addr, pc, &value, len);
            if (ret != 0) {
                handler_dabt_fail();
            }
            return value;
        }
        // mmio region for system control block
        else if ((SCB_START <= addr && addr <= SCB_END) ||
            (0xe000e000 <= addr && addr <= 0xe000e00f) ||
            (0xe000edf0 <= addr && addr <= 0xe000eeff) ||
            (0xe000ef00 <= addr && addr <= 0xe000ef4f) ||
            (0xe000ef50 <= addr && addr <= 0xe000ef8f) ||
            (0xe000ef90 <= addr && addr <= 0xe000efcf) || 
            (0xe000efd0 <= addr && addr <= 0xe000efff)) {
            uint32_t value = 0;
            int ret = sysctl_handle_mmio_read(addr, pc, &value, len);
            if (ret != 0) {
                handler_dabt_fail();
            }   
            return value;
        }
    }

    // other region
    handler_dabt_fail();
    return 0;       // do not reach here
}

/// @brief emulate singed extension
/// @param data data to extend
/// @param len data length
/// @return uint32_t result
static inline uint32_t sign_ext(uint32_t data, uint32_t len) {
    switch (len)
    {
    case 0x2: {
        int16_t tmp = data & 0xffff;
        int32_t stmp = tmp;
        return (uint32_t)stmp;
    }
    case 0x1: {
        int8_t tmp = data & 0xff;
        int32_t stmp = tmp;
        return (uint32_t)stmp;
    }
    default:
        return data;
    }
}
static inline uint32_t sign_ext(uint32_t, uint32_t) __attribute__((always_inline));

/// @brief disasm and emulate str instruction
///        STR (register), STRH (register), STRB (register)
///        STR (immediate, Thumb), STRH (immediate, Thumb), STRB (immediate, Thumb)
/// @param type instruction type
/// @param inst pointer to opcode
/// @return DABT_UNHANDLED(0) on not handler, DABT_HANDLED(1) on handled, 
///         DABT_ERROR(2) on fail
static int emulate_str_inst(int type, uint16_t *inst) {
    int str_type = INS_UNK;
    uint32_t t, n, m, index, add, wback, shift_n, imm32;
    
    if (type == INSN_THUMB_16) {
        // STR (register) Encoding T1: STR<c> <Rt>, [<Rn>, <Rm>]
        if (((inst[0] >> 9) & 0b1111111) == 0b0101000) {
            t = (inst[0] >> 0) & 0b111;
            n = (inst[0] >> 3) & 0b111;
            m = (inst[0] >> 6) & 0b111;
            index = TRUE; add = TRUE; wback = FALSE;
            shift_n = 0;
            str_type = STR_REG;
        }
        // STRH (register) Encoding T1: STRH<c> <Rt>, [<Rn>, <Rm>]
        else if (((inst[0] >> 9) & 0b1111111) == 0b0101001) {
            t = (inst[0] >> 0) & 0b111;
            n = (inst[0] >> 3) & 0b111;
            m = (inst[0] >> 6) & 0b111;
            index = TRUE; add = TRUE; wback = FALSE;
            shift_n = 0;
            str_type = STRH_REG;
        }
        // STRB (register) Encoding T1: STRB<c> <Rt>, [<Rn>, <Rm>]
        else if (((inst[0] >> 9) & 0b1111111) == 0b0101010) {
            t = (inst[0] >> 0) & 0b111;
            n = (inst[0] >> 3) & 0b111;
            m = (inst[0] >> 6) & 0b111;
            index = TRUE; add = TRUE; wback = FALSE;
            shift_n = 0;
            str_type = STRB_REG;
        }
        // STR (immediate, Thumb) Encoding T1: STR<c> <Rt>, [<Rn>{, #<imm>}]
        else if (((inst[0] >> 11) & 0b11111) == 0b01100) {
            t = (inst[0] >> 0) & 0b111;
            n = (inst[0] >> 3) & 0b111;
            imm32 = ((inst[0] >> 6) & 0b11111) << 2;
            index = TRUE; add = TRUE; wback = FALSE;
            str_type = STR_IMM;
        }
        // STR (immediate, Thumb) Encoding T2: STR<c> <Rt>, [SP, #<imm>]
        else if (((inst[0] >> 11) & 0b11111) == 0b10010) {
            t = (inst[0] >> 8) & 0b111;
            n = 13;
            imm32 = ((inst[0] >> 0) & 0b11111111) << 2;
            index = TRUE; add = TRUE; wback = FALSE;
            str_type = STR_IMM;
        }
        // STRH (immediate, Thumb) Encoding T1: STRH<c> <Rt>, [<Rn>{, #<imm>}]
        else if (((inst[0] >> 11) & 0b11111) == 0b10000) {
            t = (inst[0] >> 0) & 0b111;
            n = (inst[0] >> 3) & 0b111;
            imm32 = ((inst[0] >> 6) & 0b11111) << 1;
            index = TRUE; add = TRUE; wback = FALSE;
            str_type = STRH_IMM;
        }
        // STRB (immediate, Thumb) Encoding T1: STRB<c> <Rt>, [<Rn>, #<imm5>]
        else if (((inst[0] >> 11) & 0b11111) == 0b01110) {
            t = (inst[0] >> 0) & 0b111;
            n = (inst[0] >> 3) & 0b111;
            imm32 = (inst[0] >> 6) & 0b11111;
            index = TRUE; add = TRUE; wback = FALSE;
            str_type = STRB_IMM;
        }
    }
    else if (type == INSN_THUMB_32) {
        // STR (register) Encoding T2: STR<c>.W <Rt>, [<Rn>, <Rm>{, LSL #<imm2>}]
        if (((inst[0] >> 4) & 0b111111111111) == 0b111110000100
            && ((inst[1] >> 6) & 0b111111) == 0b000000) {
            t = (inst[1] >> 12) & 0b1111;
            n = (inst[0] >> 0) & 0b1111;
            m = (inst[1] >> 0) & 0b1111;
            index = TRUE; add = TRUE; wback = FALSE;
            shift_n = (inst[1] >> 4) & 0b11;
            if (n == 15 || t == 15 || m == 13 || m == 15) {
                return DABT_ERROR;
            }
            str_type = STR_REG;
        }
        // STRH (register) Encoding T2: STRH<c>.W <Rt>, [<Rn>, <Rm>{, LSL #<imm2>}]
        else if (((inst[0] >> 4) & 0b111111111111) == 0b111110000010
            && ((inst[1] >> 6) & 0b111111) == 0b000000) {
            t = (inst[1] >> 12) & 0b1111;
            n = (inst[0] >> 0) & 0b1111;
            m = (inst[1] >> 0) & 0b1111;
            index = TRUE; add = TRUE; wback = FALSE;
            shift_n = (inst[1] >> 4) & 0b11;
            if (n == 15 || t == 13 || t == 15 || m == 13 || m == 15) {
                return DABT_ERROR;
            }
            str_type = STRH_REG;
        }
        //  STRB (register) Encoding T2: STRB<c>.W <Rt>, [<Rn>, <Rm>{, LSL #<imm2>}]
        else if (((inst[0] >> 4) & 0b111111111111) == 0b111110000000
            && ((inst[1] >> 6) & 0b111111) == 0b000000) {
            t = (inst[1] >> 12) & 0b1111;
            n = (inst[0] >> 0) & 0b1111;
            m = (inst[1] >> 0) & 0b1111;
            index = TRUE; add = TRUE; wback = FALSE;
            shift_n = (inst[1] >> 4) & 0b11;
            if (n == 15 || t == 13 || t == 15 || m == 13 || m == 15) {
                return DABT_ERROR;
            }
            str_type = STRB_REG;
        }
        // STR (immediate, Thumb) Encoding T3: STR<c>.W <Rt>, [<Rn>, #<imm12>]
        else if (((inst[0] >> 4) & 0b111111111111) == 0b111110001100) {
            t = (inst[1] >> 12) & 0b1111;
            n = (inst[0] >> 0) & 0b1111;
            imm32 = (inst[1] >> 0) & 0b111111111111;
            index = TRUE; add = TRUE; wback = FALSE;
            if (n == 15 || t == 15) {
                return DABT_ERROR;
            }
            str_type = STR_IMM;
        }
        // STR (immediate, Thumb) Encoding T4: 
        //     STR<c> <Rt>, [<Rn>, #-<imm8>] 
        //     STR<c> <Rt>, [<Rn>], #+/-<imm8>
        //     STR<c> <Rt>, [<Rn>, #+/-<imm8>]!
        else if (((inst[0] >> 4) & 0b111111111111) == 0b111110000100
            && ((inst[1] >> 11) & 0b1) == 0b1) {
            uint32_t P = (inst[1] >> 10) & 0b1;
            uint32_t U = (inst[1] >> 9) & 0b1;
            uint32_t W = (inst[1] >> 8) & 0b1;
            t = (inst[1] >> 12) & 0b1111;
            n = (inst[0] >> 0) & 0b1111;
            imm32 = (inst[1] >> 0) & 0b11111111;
            index = (P == 1); add = (U == 1); wback = (W == 1);
            if (P == 1 && U == 1 && W == 0) {
                return DABT_ERROR;
            }
            if (n == 13 && P == 1 && U == 0 && W == 1 && imm32 == 0b100) {
                return DABT_ERROR;
            }
            if (n == 15 || (P == 0 && W == 0)) {
                return DABT_ERROR;
            }
            if (t == 15 || (wback && n == t)) {
                return DABT_ERROR;
            }
            str_type = STR_IMM;
        }
        // STRH (immediate, Thumb) Encoding T2: STRH<c>.W <Rt>, [<Rn>{, #<imm12>}]
        else if (((inst[0] >> 4) & 0b111111111111) == 0b111110001010) {
            t = (inst[1] >> 12) & 0b1111;
            n = (inst[0] >> 0) & 0b1111;
            imm32 = (inst[1] >> 0) & 0b111111111111;
            index = TRUE; add = TRUE; wback = FALSE;
            if (n == 15 || t == 15 || t == 13) {
                return DABT_ERROR;
            }
            str_type = STRH_IMM;
        }
        // STRH (immediate, Thumb) Encoding T3:
        //     STRH<c> <Rt>, [<Rn>, #-<imm8>]
        //     STRH<c> <Rt>, [<Rn>], #+/-<imm8>
        //     STRH<c> <Rt>, [<Rn>, #+/-<imm8>]!
        else if (((inst[0] >> 4) & 0b111111111111) == 0b111110000010
            && ((inst[1] >> 11) & 0b1) == 0b1) {
            uint32_t P = (inst[1] >> 10) & 0b1;
            uint32_t U = (inst[1] >> 9) & 0b1;
            uint32_t W = (inst[1] >> 8) & 0b1;
            t = (inst[1] >> 12) & 0b1111;
            n = (inst[0] >> 0) & 0b1111;
            imm32 = (inst[1] >> 0) & 0b11111111;
            index = (P == 1); add = (U == 1); wback = (W == 1);
            if (P == 1 && U == 1 && W == 0) {
                return DABT_ERROR;
            }
            if (n == 15 || (P == 0 && W == 0)) {
                return DABT_ERROR;
            }
            if (t == 15 || t == 13 || (wback && n == t)) {
                return DABT_ERROR;
            }
            str_type = STRH_IMM;
        }
        // STRB (immediate, Thumb) Encoding T2: STRB<c>.W <Rt>, [<Rn>, #<imm12>]
        else if (((inst[0] >> 4) & 0b111111111111) == 0b111110001000) {
            t = (inst[1] >> 12) & 0b1111;
            n = (inst[0] >> 0) & 0b1111;
            imm32 = (inst[1] >> 0) & 0b111111111111;
            index = TRUE; add = TRUE; wback = FALSE;
            if (n == 15 || t == 15 || t == 13) {
                return DABT_ERROR;
            }
            str_type = STRB_IMM;
        }
        // STRB (immediate, Thumb) Encoding T3:
        //     STRB<c> <Rt>, [<Rn>, #-<imm8>]
        //     STRB<c> <Rt>, [<Rn>], #+/-<imm8>
        //     STRB<c> <Rt>, [<Rn>, #+/-<imm8>]!
        else if (((inst[0] >> 4) & 0b111111111111) == 0b111110000000
            && ((inst[1] >> 11) & 0b1) == 0b1) {
            uint32_t P = (inst[1] >> 10) & 0b1;
            uint32_t U = (inst[1] >> 9) & 0b1;
            uint32_t W = (inst[1] >> 8) & 0b1;
            t = (inst[1] >> 12) & 0b1111;
            n = (inst[0] >> 0) & 0b1111;
            imm32 = (inst[1] >> 0) & 0b11111111;
            index = (P == 1); add = (U == 1); wback = (W == 1);
            if (P == 1 && U == 1 && W == 0) {
                return DABT_ERROR;
            }
            if (n == 15 || (P == 0 && W == 0)) {
                return DABT_ERROR;
            }
            if (t == 15 || t == 13 || (wback && n == t)) {
                return DABT_ERROR;
            }
            str_type = STRB_IMM;
        }
    }

    if (str_type == INS_UNK) {
        return DABT_UNHANDLED;
    }

    uint32_t offset, offset_addr, address;
    uint32_t *R = (uint32_t *)&RUNTIME_firmware_context;
    switch (str_type)
    {
    case STR_REG:
        offset = R[m] << shift_n;
        offset_addr = add ? R[n] + offset : R[n] - offset;
        address = index ? offset_addr : R[n];
        write_memory(address, 0x4, R[t]);
        if (wback) {
            R[n] = offset_addr;
        }
        return DABT_HANDLED;
    case STR_IMM:
        offset_addr = add ? R[n] + imm32 : R[n] - imm32;
        address = index ? offset_addr : R[n];
        write_memory(address, 0x4, R[t]);
        if (wback) {
            R[n] = offset_addr;
        }
        return DABT_HANDLED;
    case STRH_REG:
        offset = R[m] << shift_n;
        offset_addr = add ? R[n] + offset : R[n] - offset;
        address = index ? offset_addr : R[n];
        write_memory(address, 0x2, R[t] & 0xffff);
        if (wback) {
            R[n] = offset_addr;
        }
        return DABT_HANDLED;
    case STRH_IMM:
        offset_addr = add ? R[n] + imm32 : R[n] - imm32;
        address = index ? offset_addr : R[n];
        write_memory(address, 0x2, R[t] & 0xffff);
        if (wback) {
            R[n] = offset_addr;
        }
        return DABT_HANDLED;
    case STRB_REG:
        offset = R[m] << shift_n;
        offset_addr = add ? R[n] + offset : R[n] - offset;
        address = index ? offset_addr : R[n];
        write_memory(address, 0x1, R[t] & 0xff);
        if (wback) {
            R[n] = offset_addr;
        }
        return DABT_HANDLED;
    case STRB_IMM:
        offset_addr = add ? R[n] + imm32 : R[n] - imm32;
        address = index ? offset_addr : R[n];
        write_memory(address, 0x1, R[t] & 0xff);
        if (wback) {
            R[n] = offset_addr;
        }
        return DABT_HANDLED;
    default:
        return DABT_ERROR;
    }
}

/// @brief disasm and emulate ldr instruction
///        LDR (register, Thumb), LDR (literal), LDR (immediate, Thumb),
///        LDRH (register), LDRH (literal), LDRH (immediate, Thumb)
///        LDRSH (register), LDRSH (literal), LDRSH (immediate)
///        LDRB (register), LDRB (literal), LDRB (immediate, Thumb)
///        LDRSB (register), LDRSB (literal), LDRSB (immediate)
/// @param type instruction type
/// @param inst pointer to opcode
/// @return DABT_UNHANDLED(0) on not handler, DABT_HANDLED(1) on handled, 
///         DABT_ERROR(2) on fail
static int emulate_ldr_inst(int type, uint16_t *inst) {
    int ldr_type = INS_UNK;
    uint32_t t, n, m, index, add, wback, shift_n, imm32;
    
    if (type == INSN_THUMB_16) {
        // LDR (register, Thumb) Encoding T1: LDR<c> <Rt>, [<Rn>, <Rm>]
        if (((inst[0] >> 9) & 0b1111111) == 0b0101100) {
            t = (inst[0] >> 0) & 0b111;
            n = (inst[0] >> 3) & 0b111;
            m = (inst[0] >> 6) & 0b111;
            shift_n = 0;
            ldr_type = LDR_REG;
        }
        // LDR (literal) Encoding T1: LDR<c> <Rt>, <label>
        else if (((inst[0] >> 11) & 0b11111) == 0b01001) {
            t = (inst[0] >> 8) & 0b111;
            imm32 = ((inst[0] >> 0) & 0b11111111) << 2;
            add = TRUE;
            ldr_type = LDR_LIT;
        }
        // LDR (immediate, Thumb) Encoding T1: LDR<c> <Rt>, [<Rn>{, #<imm>}]
        else if (((inst[0] >> 11) & 0b11111) == 0b01101) {
            t = (inst[0] >> 0) & 0b111;
            n = (inst[0] >> 3) & 0b111;
            imm32 = ((inst[0] >> 6) & 0b11111) << 2;
            index = TRUE; add = TRUE; wback = FALSE;
            ldr_type = LDR_IMM;
        }
        // LDR (immediate, Thumb) Encoding T2: LDR<c> <Rt>, [SP{, #<imm>}]
        else if (((inst[0] >> 11) & 0b11111) == 0b10011) {
            t = (inst[0] >> 8) & 0b111;
            n = 13;
            imm32 = ((inst[0] >> 0) & 0b11111111) << 2;
            index = TRUE; add = TRUE; wback = FALSE;
            ldr_type = LDR_IMM;
        }
        // LDRH (register) Encoding T1: LDRH<c> <Rt>, [<Rn>, <Rm>]
        else if (((inst[0] >> 9) & 0b1111111) == 0b0101101) {
            t = (inst[0] >> 0) & 0b111;
            n = (inst[0] >> 3) & 0b111;
            m = (inst[0] >> 6) & 0b111;
            shift_n = 0;
            index = TRUE; add = TRUE; wback = FALSE;
            ldr_type = LDRH_REG;  
        }
        // LDRH (immediate, Thumb) Encoding T1: LDRH<c> <Rt>, [<Rn>{, #<imm>}]
        else if (((inst[0] >> 11) & 0b11111) == 0b10001) {
            t = (inst[0] >> 0) & 0b111;
            n = (inst[0] >> 3) & 0b111;
            imm32 = ((inst[0] >> 6) & 0b11111) << 1;
            index = TRUE; add = TRUE; wback = FALSE;
            ldr_type = LDRH_IMM;
        } 
        // LDRB (register) Encoding T1: LDRB<c> <Rt>, [<Rn>, <Rm>]
        else if (((inst[0] >> 9) & 0b1111111) == 0b0101110) {
            t = (inst[0] >> 0) & 0b111;
            n = (inst[0] >> 3) & 0b111;
            m = (inst[0] >> 6) & 0b111;
            shift_n = 0;
            index = TRUE; add = TRUE; wback = FALSE;
            ldr_type = LDRB_REG;  
        }
        // LDRB (immediate, Thumb) Encoding T1: LDRB<c> <Rt>, [<Rn>{, #<imm5>}]
        else if (((inst[0] >> 11) & 0b11111) == 0b01111) {
            t = (inst[0] >> 0) & 0b111;
            n = (inst[0] >> 3) & 0b111;
            imm32 = ((inst[0] >> 6) & 0b11111);
            index = TRUE; add = TRUE; wback = FALSE;
            ldr_type = LDRB_IMM;
        } 
        // LDRSH (register) Encoding T1: LDRSH<c> <Rt>, [<Rn>, <Rm>]
        else if (((inst[0] >> 9) & 0b1111111) == 0b0101111) {
            t = (inst[0] >> 0) & 0b111;
            n = (inst[0] >> 3) & 0b111;
            m = (inst[0] >> 6) & 0b111;
            shift_n = 0;
            index = TRUE; add = TRUE; wback = FALSE;
            ldr_type = LDRSH_REG;  
        }
        // LDRSB (register) Encoding T1: LDRSB<c> <Rt>, [<Rn>, <Rm>]
        else if (((inst[0] >> 9) & 0b1111111) == 0b0101011) {
            t = (inst[0] >> 0) & 0b111;
            n = (inst[0] >> 3) & 0b111;
            m = (inst[0] >> 6) & 0b111;
            shift_n = 0;
            index = TRUE; add = TRUE; wback = FALSE;
            ldr_type = LDRSB_REG;  
        }
    }
    else if (type == INSN_THUMB_32) {
        // LDR (register, Thumb) Encoding T2: LDR<c>.W <Rt>, [<Rn>, <Rm>{, LSL #<imm2>}]
        if (((inst[0] >> 4) & 0b111111111111) == 0b111110000101
            && ((inst[1] >> 6) & 0b111111) == 0b000000) {
            t = (inst[1] >> 12) & 0b1111;
            n = (inst[0] >> 0) & 0b1111;
            m = (inst[1] >> 0) & 0b1111;
            shift_n = (inst[1] >> 4) & 0b11;
            if (m == 13 || m == 15) {
                return DABT_ERROR;
            }
            ldr_type = LDR_REG;
        }
        // LDR (literal) Encoding T2: 
        //     LDR<c>.W <Rt>, <label>
        //     LDR<c>.W <Rt>, [PC, #-0]
        else if ((inst[0] & 0b1111111101111111) == 0b1111100001011111) {
            t = (inst[1] >> 12) & 0b1111;
            imm32 = (inst[1] >> 0) & 0b111111111111;
            add = (inst[0] >> 7) & 0b1;
            ldr_type = LDR_LIT;
        } 
        // LDR (immediate, Thumb) Encoding T3: LDR<c>.W <Rt>, [<Rn>{, #<imm12>}]
        else if (((inst[0] >> 4) & 0b111111111111) == 0b111110001101) {
            t = (inst[1] >> 12) & 0b1111;
            n = (inst[0] >> 0) & 0b1111;
            imm32 = (inst[1] >> 0) & 0b111111111111;
            index = TRUE; add = TRUE; wback = FALSE;
            ldr_type = LDR_IMM;
        } 
        // LDR (immediate, Thumb) Encoding T4:
        //     LDR<c> <Rt>, [<Rn>, #-<imm8>]
        //     LDR<c> <Rt>, [<Rn>], #+/-<imm8>
        //     LDR<c> <Rt>, [<Rn>, #+/-<imm8>]!
        else if (((inst[0] >> 4) & 0b111111111111) == 0b111110000101
            && ((inst[1] >> 11) & 0b1) == 0b1) {
            uint32_t P = (inst[1] >> 10) & 0b1;
            uint32_t U = (inst[1] >> 9) & 0b1;
            uint32_t W = (inst[1] >> 8) & 0b1;
            t = (inst[1] >> 12) & 0b1111;
            n = (inst[0] >> 0) & 0b1111;
            imm32 = (inst[1] >> 0) & 0b11111111;
            index = (P == 1); add = (U == 1); wback = (W == 1);
            if (P == 1 && U == 1 && W == 0) {
                return DABT_ERROR;
            }
            if (n == 13 && P == 0 && U == 1 && W == 1 && imm32 == 0b100) {
                return DABT_ERROR;
            }
            if (P == 0 && W == 0) {
                return DABT_ERROR;
            }
            if (wback && n == t) {
                return DABT_ERROR;
            }
            ldr_type = LDR_IMM;
        }
        // LDRH (literal) Encoding T1: 
        //     LDRH<c> <Rt>, <label>
        //     LDRH<c> <Rt>, [PC, #-0]
        else if ((inst[0] & 0b1111111101111111) == 0b1111100000111111) {
            t = (inst[1] >> 12) & 0b1111;
            imm32 = (inst[1] >> 0) & 0b111111111111;
            add = (inst[0] >> 7) & 0b1;
            if (t == 13) {
                return DABT_ERROR;
            }
            ldr_type = LDRH_LIT;
        }
        // LDRH (register) Encoding T2: LDRH<c>.W <Rt>, [<Rn>, <Rm>{, LSL #<imm2>}]
        else if (((inst[0] >> 4) & 0b111111111111) == 0b111110000011
            && ((inst[1] >> 6) & 0b111111) == 0b000000) {
            t = (inst[1] >> 12) & 0b1111;
            n = (inst[0] >> 0) & 0b1111;
            m = (inst[1] >> 0) & 0b1111;
            shift_n = (inst[1] >> 4) & 0b11;
            index = TRUE; add = TRUE; wback = FALSE;
            if (t == 13 || m == 13 || m == 15) {
                return DABT_ERROR;
            }
            ldr_type = LDRH_REG;
        }
        // LDRH (immediate, Thumb) Encoding T2: LDRH<c>.W <Rt>, [<Rn>{, #<imm12>}]
        else if (((inst[0] >> 4) & 0b111111111111) == 0b111110001011) {
            t = (inst[1] >> 12) & 0b1111;
            n = (inst[0] >> 0) & 0b1111;
            imm32 = (inst[1] >> 0) & 0b111111111111;
            index = TRUE; add = TRUE; wback = FALSE;
            if (t == 13) {
                return DABT_ERROR;
            }
            ldr_type = LDRH_IMM;
        }
        // LDRH (immediate, Thumb) Encoding T3:
        //     LDRH<c> <Rt>, [<Rn>, #-<imm8>]
        //     LDRH<c> <Rt>, [<Rn>], #+/-<imm8>
        //     LDRH<c> <Rt>, [<Rn>, #+/-<imm8>]!
        else if (((inst[0] >> 4) & 0b111111111111) == 0b111110000011
            && ((inst[1] >> 11) & 0b1) == 0b1) {
            uint32_t P = (inst[1] >> 10) & 0b1;
            uint32_t U = (inst[1] >> 9) & 0b1;
            uint32_t W = (inst[1] >> 8) & 0b1;
            t = (inst[1] >> 12) & 0b1111;
            n = (inst[0] >> 0) & 0b1111;
            imm32 = (inst[1] >> 0) & 0b11111111;
            index = (P == 1); add = (U == 1); wback = (W == 1);
            if (P == 1 && U == 1 && W == 0) {
                return DABT_ERROR;
            }
            if (P == 0 && W == 0) {
                return DABT_ERROR;
            }
            if ((wback && n == t) || t == 13 || (t == 15 && W == 1)) {
                return DABT_ERROR;
            }
            ldr_type = LDRH_IMM;
        }
        // LDRB (literal) Encoding T1:
        //     LDRB<c> <Rt>, <label>
        //     LDRB<c> <Rt>, [PC, #-0]
        else if ((inst[0] & 0b1111111101111111) == 0b1111100000011111) {
            t = (inst[1] >> 12) & 0b1111;
            imm32 = (inst[1] >> 0) & 0b111111111111;
            add = (inst[0] >> 7) & 0b1;
            if (t == 13) {
                return DABT_ERROR;
            }
            ldr_type = LDRB_LIT;
        }
        // LDRB (register) Encoding T2: LDRB<c>.W <Rt>, [<Rn>, <Rm>{, LSL #<imm2>}]
        else if (((inst[0] >> 4) & 0b111111111111) == 0b111110000001
            && ((inst[1] >> 6) & 0b111111) == 0b000000) {
            t = (inst[1] >> 12) & 0b1111;
            n = (inst[0] >> 0) & 0b1111;
            m = (inst[1] >> 0) & 0b1111;
            shift_n = (inst[1] >> 4) & 0b11;
            index = TRUE; add = TRUE; wback = FALSE;
            if (t == 13 || m == 13 || m == 15) {
                return DABT_ERROR;
            }
            ldr_type = LDRB_REG;
        }
        // LDRB (immediate, Thumb) Encoding T2: LDRB<c>.W <Rt>, [<Rn>{, #<imm12>}]
        else if (((inst[0] >> 4) & 0b111111111111) == 0b111110001001) {
            t = (inst[1] >> 12) & 0b1111;
            n = (inst[0] >> 0) & 0b1111;
            imm32 = (inst[1] >> 0) & 0b111111111111;
            index = TRUE; add = TRUE; wback = FALSE;
            if (t == 13) {
                return DABT_ERROR;
            }
            ldr_type = LDRB_IMM;
        }
        // LDRB (immediate, Thumb) Encoding T3:
        //     LDRB<c> <Rt>, [<Rn>, #-<imm8>]
        //     LDRB<c> <Rt>, [<Rn>], #+/-<imm8>
        //     LDRB<c> <Rt>, [<Rn>, #+/-<imm8>]!
        else if (((inst[0] >> 4) & 0b111111111111) == 0b111110000001
            && ((inst[1] >> 11) & 0b1) == 0b1) {
            uint32_t P = (inst[1] >> 10) & 0b1;
            uint32_t U = (inst[1] >> 9) & 0b1;
            uint32_t W = (inst[1] >> 8) & 0b1;
            t = (inst[1] >> 12) & 0b1111;
            n = (inst[0] >> 0) & 0b1111;
            imm32 = (inst[1] >> 0) & 0b11111111;
            index = (P == 1); add = (U == 1); wback = (W == 1);
            if (P == 1 && U == 1 && W == 0) {
                return DABT_ERROR;
            }
            if (P == 0 && W == 0) {
                return DABT_ERROR;
            }
            if ((wback && n == t) || t == 13 || (t == 15 && W == 1)) {
                return DABT_ERROR;
            }
            ldr_type = LDRB_IMM;
        }
        // LDRSH (literal) Encoding T1:
        //     LDRSH<c> <Rt>, <label>
        //     LDRSH<c> <Rt>, [PC, #-0]
        else if ((inst[0] & 0b1111111101111111) == 0b1111100100111111) {
            t = (inst[1] >> 12) & 0b1111;
            imm32 = (inst[1] >> 0) & 0b111111111111;
            add = (inst[0] >> 7) & 0b1;
            if (t == 13) {
                return DABT_ERROR;
            }
            ldr_type = LDRSH_LIT;
        }
        // LDRSH (immediate) Encoding T1: LDRSH<c> <Rt>, [<Rn>, #<imm12>]
        else if (((inst[0] >> 4) & 0b111111111111) == 0b111110011011) {
            t = (inst[1] >> 12) & 0b1111;
            n = (inst[0] >> 0) & 0b1111;
            imm32 = (inst[1] >> 0) & 0b111111111111;
            index = TRUE; add = TRUE; wback = FALSE;
            if (t == 13) {
                return DABT_ERROR;
            }
            ldr_type = LDRSH_IMM;
        }
        // LDRSH (immediate) Encoding T2:
        //     LDRSH<c> <Rt>, [<Rn>, #-<imm8>]
        //     LDRSH<c> <Rt>, [<Rn>], #+/-<imm8>
        //     LDRSH<c> <Rt>, [<Rn>, #+/-<imm8>]!
        else if (((inst[0] >> 4) & 0b111111111111) == 0b111110010011
            && ((inst[1] >> 11) & 0b1) == 0b1) {
            uint32_t P = (inst[1] >> 10) & 0b1;
            uint32_t U = (inst[1] >> 9) & 0b1;
            uint32_t W = (inst[1] >> 8) & 0b1;
            t = (inst[1] >> 12) & 0b1111;
            n = (inst[0] >> 0) & 0b1111;
            imm32 = (inst[1] >> 0) & 0b11111111;
            index = (P == 1); add = (U == 1); wback = (W == 1);
            if (P == 1 && U == 1 && W == 0) {
                return DABT_ERROR;
            }
            if (P == 0 && W == 0) {
                return DABT_ERROR;
            }
            if ((wback && n == t) || t == 13 || (t == 15 && W == 1)) {
                return DABT_ERROR;
            }
            ldr_type = LDRSH_IMM;
        }
        // LDRSH (register) Encoding T2: LDRSH<c>.W <Rt>, [<Rn>, <Rm>{, LSL #<imm2>}]
        else if (((inst[0] >> 4) & 0b111111111111) == 0b111110010011
            && ((inst[1] >> 6) & 0b111111) == 0b000000) {
            t = (inst[1] >> 12) & 0b1111;
            n = (inst[0] >> 0) & 0b1111;
            m = (inst[1] >> 0) & 0b1111;
            shift_n = (inst[1] >> 4) & 0b11;
            index = TRUE; add = TRUE; wback = FALSE;
            if (t == 13 || m == 13 || m == 15) {
                return DABT_ERROR;
            }
            ldr_type = LDRSH_REG;
        }
        // LDRSB (literal) Encoding T1: 
        //     LDRSB<c> <Rt>, <label>
        //     LDRSB<c> <Rt>, [PC, #-0]
        else if ((inst[0] & 0b1111111101111111) == 0b1111100100011111) {
            t = (inst[1] >> 12) & 0b1111;
            imm32 = (inst[1] >> 0) & 0b111111111111;
            add = (inst[0] >> 7) & 0b1;
            if (t == 13) {
                return DABT_ERROR;
            }
            ldr_type = LDRSB_LIT;
        }
        // LDRSB (immediate) Encoding T1: LDRSB<c> <Rt>, [<Rn>, #<imm12>]
        else if (((inst[0] >> 4) & 0b111111111111) == 0b111110011001) {
            t = (inst[1] >> 12) & 0b1111;
            n = (inst[0] >> 0) & 0b1111;
            imm32 = (inst[1] >> 0) & 0b111111111111;
            index = TRUE; add = TRUE; wback = FALSE;
            if (t == 13) {
                return DABT_ERROR;
            }
            ldr_type = LDRSB_IMM;
        }
        // LDRSB (immediate) Encoding T2:
        //     LDRSB<c> <Rt>, [<Rn>, #-<imm8>]
        //     LDRSB<c> <Rt>, [<Rn>], #+/-<imm8>
        //     LDRSB<c> <Rt>, [<Rn>, #+/-<imm8>]!
        else if (((inst[0] >> 4) & 0b111111111111) == 0b111110010001
            && ((inst[1] >> 11) & 0b1) == 0b1) {
            uint32_t P = (inst[1] >> 10) & 0b1;
            uint32_t U = (inst[1] >> 9) & 0b1;
            uint32_t W = (inst[1] >> 8) & 0b1;
            t = (inst[1] >> 12) & 0b1111;
            n = (inst[0] >> 0) & 0b1111;
            imm32 = (inst[1] >> 0) & 0b11111111;
            index = (P == 1); add = (U == 1); wback = (W == 1);
            if (P == 1 && U == 1 && W == 0) {
                return DABT_ERROR;
            }
            if (P == 0 && W == 0) {
                return DABT_ERROR;
            }
            if ((wback && n == t) || t == 13 || (t == 15 && W == 1)) {
                return DABT_ERROR;
            }
            ldr_type = LDRSB_IMM;
        }
        // LDRSB (register) Encoding T2: LDRSB<c>.W <Rt>, [<Rn>, <Rm>{, LSL #<imm2>}]
        else if (((inst[0] >> 4) & 0b111111111111) == 0b111110010001
            && ((inst[1] >> 6) & 0b111111) == 0b000000) {
            t = (inst[1] >> 12) & 0b1111;
            n = (inst[0] >> 0) & 0b1111;
            m = (inst[1] >> 0) & 0b1111;
            shift_n = (inst[1] >> 4) & 0b11;
            index = TRUE; add = TRUE; wback = FALSE;
            if (t == 13 || m == 13 || m == 15) {
                return DABT_ERROR;
            }
            ldr_type = LDRSB_REG;
        }
    }

    if (ldr_type == INS_UNK) {
        return DABT_UNHANDLED;
    }
    // we don't let value read from mmio be set to PC
    if (t == 15) {
        return DABT_ERROR;
    }

    uint32_t offset, offset_addr, address, data, base;
    uint32_t *R = (uint32_t *)&RUNTIME_firmware_context;
    switch (ldr_type)
    {
    case LDR_IMM:
        offset_addr = add ? R[n] + imm32 : R[n] - imm32;
        address = index ? offset_addr : R[n];
        data = read_memory(address, 0x4);
        if (wback) {
            R[n] = offset_addr;
        }
        R[t] = data;
        return DABT_HANDLED;
    case LDR_LIT:
        base = R[15] & 0xfffffffc;
        address = add ? base + imm32 : base - imm32;
        R[t] = read_memory(address, 0x4);
        return DABT_HANDLED;
    case LDR_REG:
        offset = R[m] << shift_n;
        offset_addr = R[n] + offset;
        address = offset_addr;
        R[t] = read_memory(address, 0x4);
        return DABT_HANDLED;
    case LDRH_IMM:
        offset_addr = add ? R[n] + imm32 : R[n] - imm32;
        address = index ? offset_addr : R[n];
        data = read_memory(address, 0x2);
        if (wback) {
            R[n] = offset_addr;
        }
        R[t] = data;
        return DABT_HANDLED;
    case LDRH_LIT:
        base = R[15] & 0xfffffffc;
        address = add ? base + imm32 : base - imm32;
        R[t] = read_memory(address, 0x2);
        return DABT_HANDLED;
    case LDRH_REG:
        offset = R[m] << shift_n;
        offset_addr = add ? R[n] + offset : R[n] - offset;
        address = index ? offset_addr : R[n];
        data = read_memory(address, 0x2);
        if (wback) {
            R[n] = offset_addr;
        }
        R[t] = data;
        return DABT_HANDLED;
    case LDRB_IMM:
        offset_addr = add ? R[n] + imm32 : R[n] - imm32;
        address = index ? offset_addr : R[n];
        R[t] = read_memory(address, 0x1);
        if (wback) {
            R[n] = offset_addr;
        }
        return DABT_HANDLED;
    case LDRB_LIT:
        base = R[15] & 0xfffffffc;
        address = add ? base + imm32 : base - imm32;
        R[t] = read_memory(address, 0x1);
        return DABT_HANDLED;
    case LDRB_REG:
        offset = R[m] << shift_n;
        offset_addr = add ? R[n] + offset : R[n] - offset;
        address = index ? offset_addr : R[n];
        R[t] = read_memory(address, 0x1);
        if (wback) {
            R[n] = offset_addr;
        }
        return DABT_HANDLED;
    case LDRSH_IMM:
        offset_addr = add ? R[n] + imm32 : R[n] - imm32;
        address = index ? offset_addr : R[n];
        data = read_memory(address, 0x2);
        if (wback) {
            R[n] = offset_addr;
        }
        R[t] = sign_ext(data, 0x2);
        return DABT_HANDLED;
    case LDRSH_LIT:
        base = R[15] & 0xfffffffc;
        address = add ? base + imm32 : base - imm32;
        data = read_memory(address, 0x2);
        R[t] = sign_ext(data, 0x2);
        return DABT_HANDLED;
    case LDRSH_REG:
        offset = R[m] << shift_n;
        offset_addr = add ? R[n] + offset : R[n] - offset;
        address = index ? offset_addr : R[n];
        data = read_memory(address, 0x2);
        if (wback) {
            R[n] = offset_addr;
        }
        R[t] = sign_ext(data, 0x2);
        return DABT_HANDLED;
    case LDRSB_IMM:
        offset_addr = add ? R[n] + imm32 : R[n] - imm32;
        address = index ? offset_addr : R[n];
        R[t] = sign_ext(read_memory(address, 0x1), 0x1);
        if (wback) {
            R[n] = offset_addr;
        }
        return DABT_HANDLED;
    case LDRSB_LIT:
        base = R[15] & 0xfffffffc;
        address = add ? base + imm32 : base - imm32;
        R[t] = sign_ext(read_memory(address, 0x1), 0x1);
        return DABT_HANDLED;
    case LDRSB_REG:
        offset = R[m] << shift_n;
        offset_addr = add ? R[n] + offset : R[n] - offset;
        address = index ? offset_addr : R[n];
        R[t] = sign_ext(read_memory(address, 0x1), 0x1);
        if (wback) {
            R[n] = offset_addr;
        }
        return DABT_HANDLED;
    default:
        return DABT_ERROR;
    }
}

/// @brief disasm the instruction which caused the abort and emulate it
/// @param addr instruction pyhsical address
/// @param dfar fault location
void emulate_inst(uint32_t addr, uint32_t dfar) {
    int ret, type = insn_type(addr);

#if DEBUG_CHECK_DFAR
    cur_dfar = dfar;
#else
    (void)(dfar);   // we do not use dfar
#endif

    // try to emulate ldr
    ret = emulate_ldr_inst(type, (uint16_t *)addr);
    if (ret != DABT_UNHANDLED) {
        if (ret == DABT_ERROR) {
            goto invalid_inst;
        }
        skip_instr();
        return;
    }

    // try to emulate str
    ret = emulate_str_inst(type, (uint16_t *)addr);
    if (ret != DABT_UNHANDLED) {
        if (ret == DABT_ERROR) {
            goto invalid_inst;
        }
        skip_instr();
        return;
    }

invalid_inst:
    // if the instruction is invalid, the firmware is crashed
    // so we abort the firmware
#if KVM_OPEN_DEBUG
    if (unlikely(should_output(RT_OUTPUT_EMU))) {
        debug_clear();
        debug_append_str("[guest-handler] invalid instruction @ ");
        debug_append_int(addr);
        debug_append_str(" code = ");
        debug_append_int(*(uint32_t *)addr);
        debug_append_str(type == INSN_THUMB_16 ? " (Thumb16)\n" : " (Thumb32)\n");
        debug_print();
    }
#endif
    handler_dabt_fail();
}
