#include "global.h"

.section .rt_vector_table,"ax",%progbits
.arm
.syntax unified

.global RUNTIME_vector_table

RUNTIME_vector_table:
#ifdef PARAVIRT
    ldr     pc, =handler_reset
    ldr     pc, =handler_und_report_host
    ldr     pc, =handler_svc_report_host
    ldr     pc, =handler_prefetch_report_host
    ldr     pc, =handler_dabt_report_host
    b       .
    ldr     pc, =handler_irq_report_host
    ldr     pc, =handler_fiq_report_host
#else
    ldr     pc, =handler_reset
    ldr     pc, =handler_und_entry_exit
    ldr     pc, =handler_svc_entry_exit
    ldr     pc, =handler_prefetch_entry_exit
    ldr     pc, =handler_dabt_entry_exit
    b       .
    ldr     pc, =handler_irq_entry_exit
    ldr     pc, =handler_fiq_report_host
#endif

.section .rt_page_table,"a",%progbits
.type RUNTIME_page_table, %object
.syntax unified 

/* TEX[2] = 0b0, TEX[1:0] = 0b01, C = 0b0, B = 0b1 */
#define DEVICE              ((0b0001 << 16) + (0b001 << 12) + (0b011 << 1))
/* TEX[2] = 0b1, TEX[1:0] = 0b01, C = 0b0, B = 0b1 */
#define NORMAL_WBWA         ((0b0001 << 16) + (0b101 << 12) + (0b011 << 1))
/* TEX[2] = 0b1, TEX[1:0] = 0b11, C = 0b1, B = 0b1 */
#define NORMAL_WBnWA        ((0b0001 << 16) + (0b111 << 12) + (0b111 << 1))
/* AP[2] = 0b0, AP[1:0] = 0b11 */
#define AP_RW               ((0b0 << 15) + (0b11 << 10))
/* AP[2] = 0b1, AP[1:0] = 0b11 */
#define AP_R                ((0b1 << 15) + (0b11 << 10))
/* XN = 0b0 */
#define PER_X               (0b0 << 4)
/* XN = 0b1 */
#define PER_NX              (0b1 << 4)
/* DOMAIN = 0 */
#define DOMAIN_0            (0b0 << 5)
/* DOMAIN = 1 */
#define DOMAIN_1            (0b1 << 5)

    .global RUNTIME_page_table

RUNTIME_page_table:
    .set    sect, 0
    /* memory region for rom or flash */
    .rept   ROM_OR_FLASH_SIZE / PAGE_SIZE
    .word   sect + NORMAL_WBWA + DOMAIN_0           /*default: + AP_R + PER_X*/
    .set    sect, sect + PAGE_SIZE
    .endr
    /* memory region for sram */
    .rept   SRAM_SIZE / PAGE_SIZE
    .word   sect + NORMAL_WBWA + DOMAIN_0           /*default: + AP_RW + PER_NX*/
    .set    sect, sect + PAGE_SIZE
    .endr
    /* mmio region for on-chip devices */
    .rept   ON_CHIP_DEV_SIZE / PAGE_SIZE
#ifdef PARAVIRT
    .word   sect + DEVICE + AP_RW + PER_NX + DOMAIN_0
#else
    .word   sect + NORMAL_WBnWA + DOMAIN_1          /*default: + AP_RW + PER_NX*/
#endif
    .set    sect, sect + PAGE_SIZE
    .endr
    /* memory region for wbwa-ram */
    .rept   WBWA_RAM_SIZE / PAGE_SIZE
    .word   sect + NORMAL_WBWA + DOMAIN_0           /*default: + AP_RW + PER_NX*/
    .set    sect, sect + PAGE_SIZE
    .endr
    /* memory region for wt-ram */
    .rept   WT_RAM_SIZE / PAGE_SIZE
    .word   sect + NORMAL_WBWA + DOMAIN_0           /*default: + AP_RW + PER_NX*/
    .set    sect, sect + PAGE_SIZE
    .endr
    /* mmio region for off-chip devices */
    .rept   (VEN_SYS_DEV_END - S_DEV_START + 1) / PAGE_SIZE
#ifdef PARAVIRT
    .word   sect + DEVICE + AP_RW + PER_NX + DOMAIN_0
#else
    .word   sect + NORMAL_WBnWA + DOMAIN_1          /*default: AP_RW + PER_NX*/
#endif
    .set    sect, sect + PAGE_SIZE
    .endr
    /* dmz for page table, vector table and data */
    .word   sect + NORMAL_WBWA + AP_RW + PER_X + DOMAIN_0
    .set    sect, sect + PAGE_SIZE
    /* memory region for runtime */
    .rept   RUNTIME_SIZE / PAGE_SIZE - 1
#if PROTECT_RUNTIME
    .word   sect + NORMAL_WBWA + AP_RW + PER_X + DOMAIN_1
#else
    .word   sect + NORMAL_WBWA + AP_RW + PER_X + DOMAIN_0
#endif
    .set    sect, sect + PAGE_SIZE
    .endr
    /* memory region for exception return */
    .rept   EXC_RETURN_SIZE / PAGE_SIZE
    .word   sect + DEVICE + AP_RW + PER_NX
    .set    sect, sect + PAGE_SIZE
    .endr
