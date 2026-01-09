#ifndef GLOBAL_H
#define GLOBAL_H

////////////////////////////////////////////////////////////////////////////////
// CPU Option
////////////////////////////////////////////////////////////////////////////////
#define CORTEXM3_CPUID      0x412fc230
#define CORTEXM4_CPUID      0x410fc241
#define CORTEXM7_CPUID      0x410fc270

#define CPUID_VALUE         CORTEXM4_CPUID

#define PROTECT_RUNTIME         0
#define PRINT_TRAMPOLINE        0

// #define KVM_OPEN_DEBUG          0

/* size of the coverage map */
/* must be a power of 2 */
#define COVERAGE_MAP_SIZE       (1 << 16)

/* period of irq trigger (unit: ns) */
/* default value: 1us */
#define SYSTICK_PERIOD_CPU      1000
#define SYSTICK_PERIOD_REF      1000

/* max count of basic block to execute */
/* default value: 3000000 */
#define MAX_BLOCK               3000000

/* max count of basic block to execute without any read of mmio */
/* default value: 1500 */
/* WE DON'T USE THIS */
#define MAX_BLOCK_WITHOUT_MMIO  1500

/* timeout for exploring basic blocks (unit: us) */
/* default value: 10ms */
#define EXPLORE_TIMEOUT         10000

/* max count of interrupts */
/* default value: 450 */
#define DEFAULT_MAX_INTERRUPTS  1000

////////////////////////////////////////////////////////////////////////////////
// Execution State
////////////////////////////////////////////////////////////////////////////////
#define EXIT_OK                 0
#define EXIT_FIRMWARE_CRASH     1
#define EXIT_INTERNAL_CRASH     2
#define EXIT_TIMEOUT            3
#define EXIT_MODEL              4
#define EXIT_DEBUG_SINGLESTEP   5
#define EXIT_DEBUG_BREAKPOINT   6
#define EXIT_DEBUG_WATCHPOINT   7
#define HANDLED                -1
#define UNHANDLED              -2

////////////////////////////////////////////////////////////////////////////////
// Memory settings
////////////////////////////////////////////////////////////////////////////////
// kvm memory regions
#define MEMORY_FLAGS            0

// guest page size
#define PAGE_SIZE               0x100000

// ROM or Flash memory region
#define ROM_OR_FLASH_SLOT       0
#define ROM_OR_FLASH_FLAGS      MEMORY_FLAGS
#define ROM_OR_FLASH_START      0x00000000
#define ROM_OR_FLASH_END        0x1fffffff
#define ROM_OR_FLASH_SIZE       (ROM_OR_FLASH_END - ROM_OR_FLASH_START + 1)

// SRAM memory region
#define SRAM_SLOT               1
#define SRAM_FLAGS              MEMORY_FLAGS
#define SRAM_START              0x20000000
#define SRAM_END                0x3fffffff
#define SRAM_SIZE               (SRAM_END - SRAM_START + 1)

// on-chip devices mmio region
#define ON_CHIP_DEV_START       0x40000000
#define ON_CHIP_DEV_END         0x5fffffff
#define ON_CHIP_DEV_SIZE        (ON_CHIP_DEV_END - ON_CHIP_DEV_START + 1)

// Write-Back, Write-Allocate memory region
#define WBWA_RAM_SLOT           2
#define WBWA_RAM_FLAGS          MEMORY_FLAGS
#define WBWA_RAM_START          0x60000000
#define WBWA_RAM_END            0x7fffffff
#define WBWA_RAM_SIZE           (WBWA_RAM_END - WBWA_RAM_START + 1)

// Write-Through memory region
#define WT_RAM_SLOT             3 
#define WT_RAM_FLAGS            MEMORY_FLAGS
#define WT_RAM_START            0x80000000
#define WT_RAM_END              0x9fffffff
#define WT_RAM_SIZE             (WT_RAM_END - WT_RAM_START + 1)

// Device, Shareable mmio region
#define S_DEV_START             0xa0000000
#define S_DEV_END               0xbfffffff
#define S_DEV_SIZE              (S_DEV_END - S_DEV_START + 1)

// Device, No-Shareable mmio region
#define NS_DEV_START            0xc0000000
#define NS_DEV_END              0xdfffffff
#define NS_DEV_SIZE             (NS_DEV_END - NS_DEV_START + 1)

// PPB area 
#define PPB_START               0xe0000000
#define SC_START                0xe000e000
#define SYS_TICK_START          0xe000e010
#define SYS_TICK_END            0xe000e0ff
#define SYS_TICK_SIZE           (SYS_TICK_END - SYS_TICK_START + 1)
#define NVIC_START              0xe000e100
#define NVIC_END                0xe000ecff
#define NVIC_SIZE               (NVIC_END - NVIC_START + 1)
#define SCB_START               0xe000ed00
#define SCB_END                 0xe000ed8f
#define SCB_SIZE                (SCB_END - SCB_START + 1)
#define MPU_START               0xe000ed90
#define MPU_END                 0xe000edef
#define MPU_SIZE                (MPU_END - MPU_START + 1)
#define PPB_END                 0xe00fffff

// memory region for vendor devices
#define VEN_SYS_DEV_START       0xe0100000
#define VEN_SYS_DEV_END         0xefffffff

// memory region for runtime
#define RUNTIME_SLOT            4
#define RUNTIME_FLAGS           MEMORY_FLAGS
#define RUNTIME_START           0xf0000000
#define RUNTIME_END             (0xffffffff - PAGE_SIZE)
#define RUNTIME_SIZE            (RUNTIME_END - RUNTIME_START + 1)

// memory region for handle exception return
#define EXC_RETURN_START        (RUNTIME_END + 1)
#define EXC_RETURN_END          0xffffffff
#define EXC_RETURN_SIZE         (EXC_RETURN_END - EXC_RETURN_START + 1)

// kvm defines
#define COMPAT_PSR_F_BIT	    0x00000040
#define COMPAT_PSR_I_BIT	    0x00000080
#define COMPAT_PSR_E_BIT	    0x00000200
#define COMPAT_PSR_T_BIT        0x00000020
#define COMPAT_PSR_MODE_SYS     0x0000001f
#define COMPAT_PSR_MODE_USR     0x00000010
#define COMPAT_PSR_MODE_SVC	    0x00000013

#define ARMv7A_VEC_RESET        0
#define ARMv7A_VEC_UND          1
#define ARMv7A_VEC_SVC          2
#define ARMv7A_VEC_PREFETCH     3
#define ARMv7A_VEC_DABT         4
#define ARMv7A_VEC_RES          5
#define ARMv7A_VEC_IRQ          6
#define ARMv7A_VEC_FIQ          7
#define ARMv7A_VEC_MAX          ( ARMv7A_VEC_FIQ + 1 )

#define ARMv7M_MRS              0
#define ARMv7M_MSR              1
#define ARMv7M_CPS              2

////////////////////////////////////////////////////////////////////////////////
// CPU Modes
////////////////////////////////////////////////////////////////////////////////
#define ARMv7M_THREAD       0
#define ARMv7M_HANDLER      1

////////////////////////////////////////////////////////////////////////////////
// Registers
// ref: https://developer.arm.com/documentation/ddi0403/d/System-Level-Architecture/System-Instruction-Details/About-the-ARMv7-M-system-instructions/Special-register-encodings-used-in-ARMv7-M-system-instructions?lang=en
////////////////////////////////////////////////////////////////////////////////
// ARMv7M core registers
#define ARMv7M_APSR         0
#define ARMv7M_IAPSR        1
#define ARMv7M_EAPSR        2
#define ARMv7M_XPSR         3
#define ARMv7M_IPSR         5
#define ARMv7M_EPSR         6
#define ARMv7M_IEPSR        7
#define ARMv7M_MSP          8
#define ARMv7M_PSP          9
#define ARMv7M_PRIMASK      16
#define ARMv7M_BASERPI      17
#define ARMv7M_BASERPI_MAX  18
#define ARMv7M_FAULTMASK    19
#define ARMv7M_CONTROL      20

// ARMv7M core register bits
#define ARMv7M_PSR_N_OFFSET             31
#define ARMv7M_PSR_N_MASK               (1 << 31)
#define ARMv7M_PSR_Z_OFFSET             30
#define ARMv7M_PSR_Z_MASK               (1 << 30)
#define ARMv7M_PSR_C_OFFSET             29
#define ARMv7M_PSR_C_MASK               (1 << 29)
#define ARMv7M_PSR_V_OFFSET             28
#define ARMv7M_PSR_V_MASK               (1 << 28)
#define ARMv7M_PSR_Q_OFFSET             27
#define ARMv7M_PSR_Q_MASK               (1 << 27)
#define ARMv7M_PSR_GE_OFFSET            16
#define ARMv7M_PSR_GE_MASK              (0b1111 << 16)
#define ARMv7M_PSR_EXC_NUM_OFFSET       0
#define ARMv7M_PSR_EXC_NUM_MASK         (0b111111111)
#define ARMv7M_PSR_ICIIT_OFFSET         0
#define ARMv7M_PSR_ICIIT_MASK           (0b00000110000000001111110000000000)
#define ARMv7M_PSR_T_OFFSET             24
#define ARMv7M_PSR_T_MASK               (1 << 24)
#define ARMv7M_CONTROL_PRIV_OFFSET      0
#define ARMv7M_CONTROL_PRIV_MASK        0b1
#define ARMv7M_CONTROL_SPSEL_OFFSET     1
#define ARMv7M_CONTROL_SPSEL_MASK       0b10
#define ARMv7M_CONTROL_FPCA_OFFSET      2
#define ARMv7M_CONTROL_FPCA_MASK        0b100

// ARMv7M core register masks
#define ARMv7M_APSR_nzcvq   2
#define ARMv7M_APSR_g       1
#define ARMv7M_APSR_nzcvqg  3

// ARMv8A AArch32 registers under KVM
#define ARMv8A_R_OFFSET     0x2
#define ARMv8A_R0           0x6030000000100000ll
#define ARMv8A_R1           0x6030000000100002ll
#define ARMv8A_R2           0x6030000000100004ll
#define ARMv8A_R3           0x6030000000100006ll
#define ARMv8A_R4           0x6030000000100008ll
#define ARMv8A_R5           0x603000000010000All
#define ARMv8A_R6           0x603000000010000Cll
#define ARMv8A_R7           0x603000000010000Ell
#define ARMv8A_R8_usr       0x6030000000100010ll
#define ARMv8A_R9_usr       0x6030000000100012ll
#define ARMv8A_R10_usr      0x6030000000100014ll
#define ARMv8A_R11_usr      0x6030000000100016ll
#define ARMv8A_R12_usr      0x6030000000100018ll
#define ARMv8A_SP_usr       0x603000000010001All
#define ARMv8A_LR_usr       0x603000000010001Cll
#define ARMv8A_PC           0x6030000000100040ll
#define ARMv8A_R8_fiq       0x6030000000100030ll
#define ARMv8A_R9_fiq       0x6030000000100032ll
#define ARMv8A_R10_fiq      0x6030000000100034ll  
#define ARMv8A_R11_fiq      0x6030000000100036ll   
#define ARMv8A_R12_fiq      0x6030000000100038ll
#define ARMv8A_SP_fiq       0x603000000010003All
#define ARMv8A_LR_fiq       0x603000000010003Cll
#define ARMv8A_SP_irq       0x6030000000100022ll
#define ARMv8A_LR_irq       0x6030000000100020ll
#define ARMv8A_SP_abt       0x603000000010002All
#define ARMv8A_LR_abt       0x6030000000100028ll
#define ARMv8A_SP_svc       0x6030000000100026ll
#define ARMv8A_LR_svc       0x6030000000100024ll
#define ARMv8A_SP_und       0x603000000010002Ell
#define ARMv8A_LR_und       0x603000000010002Cll
#define ARMv8A_APSR         0x6030000000100042ll
#define ARMv8A_SPSR_fiq     0x6060000000100050ll
#define ARMv8A_SPSR_irq     0x603000000010004ell
#define ARMv8A_SPSR_abt     0x603000000010004all
#define ARMv8A_SPSR_svc     0x6030000000100048ll
#define ARMv8A_SPSR_und     0x603000000010004cll
#define ARMv8A_FPSCR        0x0
#define ARMv8A_FPSR         0x60200000001000d4ll
#define ARMv8A_FPCR         0x60200000001000d5ll
#define ARMv8A_V0           0x6040000000100054ll
#define ARMv8A_V_OFFSET     0x4

#define ARMv8A_MODE_usr     0x10
#define ARMv8A_MODE_sys     0x1f
#define ARMv8A_MODE_fiq     0x11
#define ARMv8A_MODE_irq     0x12
#define ARMv8A_MODE_abt     0x17
#define ARMv8A_MODE_svc     0x13
#define ARMv8A_MODE_und     0x1b

////////////////////////////////////////////////////////////////////////////////
// Additional MMIO Registers for Host-Guest communication
////////////////////////////////////////////////////////////////////////////////
// Special mmio work as HLT 
#define SPECIAL_MMIO_HLT            0xe000ed98
// Special mmio work as OUTPUT 
#define SPECIAL_MMIO_DEBUG_OUTPUT   0xe000ed9c

////////////////////////////////////////////////////////////////////////////////
// Special BKPTs for Host-Guest communication
////////////////////////////////////////////////////////////////////////////////
#define BKPT_INCOMP_INSN            0xbe01
#define BKPT_BREAKPOINT             0xbe02
#define BKPT_TIMEOUT                0xbe03
#define BKPT_COVERAGE               0xbe04
#define BKPT_MEASURE_TIME           0xbe05
#define BKPT_COVERAGE_NOREPORT      0xbe06
#define BKPT_EMU_TIMER              0xbe07
#define BKPT_THUMB_FIRMWARE_EXIT    0xbe08

#define BKPT_ARM_UND                0xe1200071
#define BKPT_ARM_SVC                0xe1200072
#define BKPT_ARM_PREFETCH           0xe1200073
#define BKPT_ARM_DATA               0xe1200074
#define BKPT_ARM_IRQ                0xe1200075
#define BKPT_ARM_DEBUG              0xe1200076
#define BKPT_ARM_FIQ                0xe1200077

#define BKPT_ARM_FIRMWARE_CRASH     0xe1200078
#define BKPT_ARM_INTERNAL_CRASH     0xe1200079
#define BKPT_ARM_FIRMWARE_EXIT      0xe120007a
#define BKPT_ARM_FIRMWARE_TIMEOUT   0xe120007b
#define BKPT_ARM_FIRMWARE_SNAPSHOT  0xe120007c

#define BKPT_MULTI                  0xbeff
#define BP_TYPE_COVERAGE            0x1
#define BP_TYPE_EXIT_POINT          0x2
#define BP_TYPE_USR                 0x3

////////////////////////////////////////////////////////////////////////////////
// NVIC Controler
////////////////////////////////////////////////////////////////////////////////
// Base address
#define NVIC_BASE_ADDR          0xe000e000
// Highest permitted number of exceptions (architectural limit)
#define NVIC_MAX_VECTORS        512
// Number of internal exceptions
#define NVIC_INTERNAL_VECTORS   16
// First irq number
#define NVIC_FIRST_IRQ          NVIC_INTERNAL_VECTORS
// no exception priority
#define NVIC_NOEXC_PRIO         0x100
// Priority bits (1~8)
#define NVIC_PRIO_BITS          8
// Max number of irqs
#define NVIC_MAX_IRQ           ( NVIC_MAX_VECTORS - NVIC_FIRST_IRQ )

// NVIC Internal Exceptions
#define ARMv7M_EXCP_RESET       1
#define ARMv7M_EXCP_NMI         2
#define ARMv7M_EXCP_HARD        3
#define ARMv7M_EXCP_MEM         4
#define ARMv7M_EXCP_BUS         5
#define ARMv7M_EXCP_USAGE       6
#define ARMv7M_EXCP_SECURE      7
#define ARMv7M_EXCP_SVC         11
#define ARMv7M_EXCP_DEBUG       12
#define ARMv7M_EXCP_PENDSV      14
#define ARMv7M_EXCP_SYSTICK     15

////////////////////////////////////////////////////////////////////////////////
// GIC Controler
////////////////////////////////////////////////////////////////////////////////
// CPU Interface 
#define ARM_GIC_CPUI_BASE       0xE0101000
// Distributor Interface
#define ARM_GIC_DIST_BASE       0xE0100000

////////////////////////////////////////////////////////////////////////////////
// Systick
////////////////////////////////////////////////////////////////////////////////
#define SYSTICK_ENABLE    (1 << 0)
#define SYSTICK_TICKINT   (1 << 1)
#define SYSTICK_CLKSOURCE (1 << 2)
#define SYSTICK_COUNTFLAG (1 << 16)

#define SYSCALIB_NOREF (1U << 31)
#define SYSCALIB_SKEW (1U << 30)
#define SYSCALIB_TENMS ((1U << 24) - 1)

#define SYSTICK_RELOAD_VAL_MASK     0x00ffffff

#define SYSTICK_1_MS                15000
#define SYSTICK_SCALE               (SYSTICK_1_MS / 10)
#define SYSTICK_CALIBRATION_VALUE   (10 * SYSTICK_1_MS)
#define SYSTICK_MIN_VALUE           500
#define SYSTICK_MAX_VALUE           1500

////////////////////////////////////////////////////////////////////////////////
// Systick
////////////////////////////////////////////////////////////////////////////////
#define RUNTIME_SVC_IRQ             0xff
#define RUNTIME_INCOMPATIBLE_INSN   0xfe
#define RUNTIME_WFI                 0xfd

// ref: https://developer.arm.com/documentation/ddi0403/d/System-Level-Architecture/System-Level-Programmers--Model/ARMv7-M-exception-model/Fault-behavior

#define BFSR_BFARVALID          (7 + 8)
#define MMFSR_MMFARVALID        (7 + 0)

// [NEVER] HardFault on vector read error
#define HARD_FAULT_VECTTBL      0x00
#define HFSR_VECTTBL            1

// HardFault on fault escalation
// ---> RAISE by runtime
#define HARD_FAULT_FORCED       0x01
#define HFSR_FORCED             30

// [NEVER] HardFault on breakpoint (BKPT) escalation
#define HARD_FAULT_DEBUGEVT     0x02
#define HFSR_DEBUGEVT           31

// [NEVER] BusFault on exception entry stack memory operations
#define BUS_FAULT_STKERR        0x10
#define BFSR_STKERR             (4 + 8)

// [NEVER] BusFault on exception return stack memory operations
#define BUS_FAULT_UNSTKERR      0x11
#define BFSR_UNSTKERR           (3 + 8)

// BusFault on instruction fetch, precise
// ---> TRANS from ARMv7-A prefetch abort
#define BUS_FAULT_IBUSERR       0x12
#define BFSR_IBUSERR            (0 + 8)

// BusFault on data access, precise
// ---> TRANS from ARMv7-A data abort
#define BUS_FAULT_PRECISERR     0x13
#define BFSR_PRECISERR          (1 + 8)

// BusFault, bus error on data bus, imprecise
// ---> TRANS from ARMv7-A data abort
#define BUS_FAULT_IMPRECISERR   0x14
#define BFSR_IMPRECISERR        (2 + 8)

// [NEVER] MemManage fault on exception entry stack memory operations
#define MEM_FAULT_MSTKERR       0x20
#define MMFSR_MSTKERR           (4 + 0)     

// [NEVER] MemManage fault on exception return stack memory operations
#define MEM_FAULT_MUNSTKERR     0x21
#define MMFSR_MUNSTKERR         (3 + 0)

// [NEVER] MemManage fault on data access
#define MEM_FAULT_DACCVIOL      0x22
#define MMFSR_DACCVIOL          (1 + 0)

// MemManage fault on instruction access
// ---> TRANS from ARMv7-A prefetch abort (XN)
#define MEM_FAULT_IACCVIOL      0x23
#define MMFSR_IACCVIOL          (0 + 0)

// [NEVER] UsageFault, No coprocessor
#define USAGE_FAULT_NOCP        0x30
#define UFSR_NOCP               (3 + 16)

// UsageFault, Undefined Instruction
// ---> TRANS from ARMv7-A undefined
#define USAGE_FAULT_UNDEFINSTR  0x31
#define UFSR_UNDEFINSTR         (0 + 16)

// UsageFault, attempt to execute an instruction when EPSR.T==0
// ---> TRANS from ARMv7-A undefined
#define USAGE_FAULT_INVSTATE    0x32
#define UFSR_INVSTATE           (1 + 16)

// UsageFault, exception return integrity check failures
// ---> RAISE by runtime cpu
#define USAGE_FAULT_INVPC       0x33
#define UFSR_INVPC              (2 + 16)

// UsageFault, illegal unaligned load or store
// ---> TRANS from ARMv7-A data abort FS=00001 Alignment Fault
#define USAGE_FAULT_UNALIGNED   0x34
#define UFSR_UNALIGNED          (8 + 16)

// [NEVER] UsageFault, divide by 0
#define USAGE_FAULT_DIVBYZERO   0x35
#define UFSR_DIVBYZERO          (9 + 16)

#define RT_OUTPUT_CPU           (1 << 0)
#define EN_OUTPUT_CPU           (1 << 0)
#define RT_OUTPUT_EMU           (1 << 1)
#define RT_OUTPUT_FAULT         (1 << 2)
#define RT_OUTPUT_FUZZWARE      (1 << 3)
#define EN_OUTPUT_FUZZWARE      (1 << 3)
#define RT_OUTPUT_HANDLER       (1 << 4)
#define RT_OUTPUT_NVIC          (1 << 5)
#define EN_OUTPUT_NVIC          (1 << 5)
#define RT_OUTPUT_SYSCTL        (1 << 6)
#define EN_OUTPUT_SYSCTL        (1 << 6)
#define RT_OUTPUT_SYSTICK       (1 << 7)
#define EN_OUTPUT_SYSTICK       (1 << 7)
#define RT_OUTPUT_TRIGGER       (1 << 8)
#define EN_OUTPUT_BOARD         (1 << 9)
#define EN_OUTPUT_LOADER        (1 << 10)

#define RT_OUTPUT_SHUTUP        (0b00000000)
#define RT_OUTPUT_ALL           (0b11111111)

#define OUTPUT_LEVEL_DEBUG      0
#define OUTPUT_LEVEL_INFO       1
#define OUTPUT_LEVEL_ERROR      2

#define PARA_MODE               0
#define FULL_MODE               1

#define likely(x)       __builtin_expect((x),1)
#define unlikely(x)     __builtin_expect((x),0)

#endif  // GLOBAL_H
