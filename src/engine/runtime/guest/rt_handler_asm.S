#include "global.h"
#include "data_offset.h"

.section .rt_dmz_handler,"ax",%progbits
.arm
.syntax unified

/* reset handler */
.global handler_reset

/* undefined handlers */
.global handler_und_report_host
.global handler_und_entry_exit
.global handler_und_fail

/* svc handlers */
.global handler_svc_report_host
.global handler_svc_entry_exit

/* prefetch abort handlers */
.global handler_prefetch_report_host
.global handler_prefetch_entry_exit
.global handler_prefetch_fail

/* data abort handlers */
.global handler_dabt_report_host
.global handler_dabt_entry_exit
.global handler_dabt_fail

/* irq handlers */
.global handler_irq_report_host
.global handler_irq_entry_exit

/* fiq handlers */
.global handler_fiq_report_host

/* [0x0] reset ============================================================== */

.func
handler_reset:
#ifdef PARAVIRT
    /* acknowlege potential gic irq */
    ldr     r0, =#0xe010100c
    ldr     r1, [r0]
    ldr     r0, =#0xe0101010
    str     r1, [r0]
#endif
    cpsid   aif, #0x1f

    /* configure vector table address to VBAR */
    ldr     r0, =RUNTIME_vector_table
    movs    r0, r0, LSR#5
    movs    r0, r0, LSL#5
    mcr     p15, 0, r0, c12, c0, 0

    /* set CPACR: enable access to CP10 and CP11 */
    mrc     p15, 0, r0, c1, c0, 2
    orr     r0, r0, #0xf00000
    mcr     p15, 0, r0, c1, c0, 2
    
    /* clear tlb */
    /* TLBIALL: invalidate unified TLB */
    mov     r0, #0
    mcr     p15, 0, r0, c8, c7, 0
    /* DTLBIALL: invalidate data TLB */
    mov     r0, #0
    mcr     p15, 0, r0, c8, c6, 0
    /* ITLBIALL: invalidate instruction TLB */
    mov     r0, #0
    mcr     p15, 0, r0, c8, c5, 0
    dsb     sy
    isb     sy

    /* configure float point extension */
    mov     r1, #0x40000000
    vmsr    fpexc, r1
    dsb     sy
    isb     sy

    /* set TTBCR:
     * EAE = Use the 32-bit translation system
     * N = 0
     */
    mov     r0, #0
    mcr     p15, 0, r0, c2, c0, 2

    /* set TTBR0:
     * S =  Non-shareable
     * NOS = Memory is Outer Shareable
     * RGN = Normal memory, Inner Write-Back no Write-Allocate Cacheable
     * IRGN = Normal memory, Inner Write-Back no Write-Allocate Cacheable
     */
    ldr     r0, =RUNTIME_page_table
    movs    r0, r0, LSR#14
    movs    r0, r0, LSL#14
    orrs    r0, r0, #0x59
    mcr     p15, 0, r0, c2, c0, 0

#ifndef DISABLE_APT
    /* set DACR: */
    /* all domains to mgr */
    mov     r0, #0xf  
    mcr     p15, 0, r0, c3, c0, 0

    /* set SCTLR: 
     * M = EL1 and EL0 stage 1 address translation enabled
     * A = Alignment fault checking disabled
     * C = Can be cached at all levels of data and unified cache
     * I = Icache eanble
     * Z = Branch 
     * TRE = TEX remap disabled
     */
    mrc     p15, 0, r0, c1, c0, 0
    bic     r0, r0, #0x2
    mov     r1, #0x1805
    orr     r0, r0, r1
    mcr     p15, 0, r0, c1, c0, 0
    isb
    dsb

#else
    /* set DACR: */
    /* all domains to mgr */
    mov     r0, #0xffffffff 
    mcr     p15, 0, r0, c3, c0, 0

    /* set SCTLR: 
     * M = EL1 and EL0 stage 1 address translation disabled
     * A = Alignment fault checking disabled
     * C = Can be cached at all levels of data and unified cache
     * I = Icache eanble
     * Z = Branch 
     * TRE = TEX remap disabled
     */
    mrc     p15, 0, r0, c1, c0, 0
    bic     r0, r0, #0x7
    @ mov     r1, #0x1804
    @ orrs    r0, r0, r1
    mov     r0, 0
    mcr     p15, 0, r0, c1, c0, 0
    isb
    dsb
#endif

    /* copy the data segment initializers from flash to SRAM */
    /* hack here: bss is reset by the host */
    ldr     r0, =_start_rt_data
    ldr     r1, =_end_rt_data
    ldr     r2, =_start_rt_flash_data
    movs    r3, #0
    b       loop_copy_data_init
copy_data_init:
    ldr     r4, [r2, r3]
    str     r4, [r0, r3]
    adds    r3, r3, #4
loop_copy_data_init:
    adds    r4, r0, r3
    cmp     r4, r1
    bcc     copy_data_init
    
    /* call static constructors */
    bl      __libc_init_array

    /* call fuzzware init */
#ifndef PARAVIRT
    bl      reset_firmware
    bl      cpu_reset
    bl      nvic_reset
    bl      sysctl_reset
    bl      systick_reset
    bl      fuzzware_reset
    bl      trigger_reset
#endif
    
    /* call the application's entry point.*/
    cpsie   aif, #0x1f
    ldr     r0, =RUNTIME_initial_sp
    ldr     sp, [r0]
    ldr     r0, =RUNTIME_firmware_entry
    ldr     r0, [r0]

#ifndef PARAVIRT
    /* DOMAIN_1: firmware has no access */
    /* DOMAIN_0: firmware access with permission check */
    mov     r1, #0x1
    mcr     p15, 0, r1, c3, c0, 0
    isb
#endif
    push    {r0}
    mov     r0, #0x0
    mov     r1, #0x0
    mov     r2, #0x0
    mov     r3, #0x0
    mov     r4, #0x0
    mov     r5, #0x0
    mov     r6, #0x0
    mov     r7, #0x0
    mov     r8, #0x0
    mov     r9, #0x0
    mov     r10, #0x0
    mov     r11, #0x0
    mov     r12, #0x0
    mov     lr, #0x0
    pop     {pc}
.endfunc

/* [0x1] undefined ========================================================== */

.func
handler_und_report_host:
    bkpt    #0x1
.endfunc

.func
handler_und_entry_exit:
    /* go to sys mode and enable AIF */
    cpsid   aif, #0x1f

    /* hack: temporarily use firmware's stack to save r4 */
    push    { r4 }

#if PROTECT_RUNTIME
    /* enable access to all domains */
    mov     r4, #0xf 
    mcr     p15, 0, r4, c3, c0, 0  
    isb 
#endif
    
    /* store context */    
	ldr     r4, =RUNTIME_firmware_context
    stmia   r4, { r0-r3 }
    str     r5, [r4, #OFFSET_CONTEXT_R5]
    /* hack: pop r4 to r5 and store r4 */
    pop     { r5 }
    str     r5, [r4, #OFFSET_CONTEXT_R4]
    mov     r5, r4
    add     r5, r5, #OFFSET_CONTEXT_R6
    stmia   r5, { r6-r12, sp, lr }
    /* lr and spsr is banked in ARMv8-A, go back to und mode and find them */
    cpsid   aif, #0x1b
    mov     r5, lr
    sub     r5, r5, #0x2
    str     r5, [r4, #OFFSET_CONTEXT_PC]
    mrs     r5, spsr
    str     r5, [r4, #OFFSET_CONTEXT_XPSR]
    cpsid   aif, #0x1f

    /* change stack to the runtime stack */    
    ldr     sp, =_end_rt_stack

    /* dive into runtime */
    /* R0 = DFSR R1 = DFAR */
    ldr     r4, =und_handler
    blx     r4

    /* checks if there's exception to take  */
    ldr     r4, =cpu_check_irq
    blx     r4

    /* restore context */
	ldr     r4, =RUNTIME_firmware_context
    ldmia   r4, { r0-r3 }
    mov     r5, r4
    add     r5, r5, #OFFSET_CONTEXT_R6
    ldmia   r5, { r6-r12, sp, lr }


    /* go back to abt mode and prepare eret */
    /* hack: exception handler will set right PC for us */
    cpsid   aif, #0x1b
    ldr     lr, [r4, #OFFSET_CONTEXT_PC]
    ldr     r5, [r4, #OFFSET_CONTEXT_XPSR]
    msr     spsr_cxsf, r5
    ldr     r5, [r4, #OFFSET_CONTEXT_R5]
    ldr     r4, [r4, #OFFSET_CONTEXT_R4]

#if PROTECT_RUNTIME
    /* restore access to all domains */
    cpsid   aif, #0x1f
    push    { r0 }
    mov     r0, #0x1  
    mcr     p15, 0, r0, c3, c0, 0
    isb
    pop     { r0 }
    cpsid   aif, #0x1b
#endif

    subs    pc, lr, #0
.endfunc

.func
handler_und_fail:
    /* restore state to enception entry */
    ldr     r4, =RUNTIME_firmware_context
	ldmia	r4, { r0-r3}
	mov		r5, r4
	add		r5, r5, #OFFSET_CONTEXT_R6
	ldmia	r5,	{ r6-r12, sp, lr}

    cpsid   aif, #0x1b
    ldr     r5, [r4, #OFFSET_CONTEXT_PC]
    ldr     r5, [r4, #OFFSET_CONTEXT_XPSR]
    msr     spsr_cxsf, r5
    ldr     r5, [r4, #OFFSET_CONTEXT_R5]
    ldr     r4, [r4, #OFFSET_CONTEXT_R4]

    /* go back to host */
    bkpt    #0x1
.endfunc

/* [0x2] SVC ================================================================ */

.func
handler_svc_report_host:
    bkpt    #0x2
.endfunc

.func
handler_svc_entry_exit:
    /* go to sys mode and enable AIF */
    cpsid   aif, #0x1f
    
    /* hack: temporarily use firmware's stack to save r4 */
    push    { r4 }
    
#if PROTECT_RUNTIME
    /* enable access to all domains */
    mov     r4, #0xf 
    mcr     p15, 0, r4, c3, c0, 0  
    isb 
#endif

    /* store context */
	ldr     r4, =RUNTIME_firmware_context
    stmia   r4, { r0-r3 }
    str     r5, [r4, #OFFSET_CONTEXT_R5]
    /* hack: pop r4 to r5 and store r4 */
    pop     { r5 }
    str     r5, [r4, #OFFSET_CONTEXT_R4]
    mov     r5, r4
    add     r5, r5, #OFFSET_CONTEXT_R6
    stmia   r5, { r6-r12, sp, lr }
    /* lr and spsr is banked in ARMv8-A, go back to svc mode and find them */
    cpsid   aif, #0x13
    mov     r5, lr
    str     r5, [r4, #OFFSET_CONTEXT_PC]
    mrs     r5, spsr
    str     r5, [r4, #OFFSET_CONTEXT_XPSR]
    cpsid   aif, #0x1f

    /* change stack to the runtime stack */    
    ldr     sp, =_end_rt_stack

    /* dive into runtime */
    /* R0 = DFSR R1 = DFAR */
    ldr     r4, =svc_handler
    blx     r4

    /* checks if there's exception to take  */
    ldr     r4, =cpu_check_irq
    blx     r4

    /* restore context */
	ldr     r4, =RUNTIME_firmware_context
    ldmia   r4, { r0-r3 }
    mov     r5, r4
    add     r5, r5, #OFFSET_CONTEXT_R6
    ldmia   r5, { r6-r12, sp, lr }


    /* go back to abt mode and prepare eret */
    /* hack: exception handler will set right PC for us */
    cpsid   aif, #0x13
    ldr     lr, [r4, #OFFSET_CONTEXT_PC]
    ldr     r5, [r4, #OFFSET_CONTEXT_XPSR]
    msr     spsr_cxsf, r5
    ldr     r5, [r4, #OFFSET_CONTEXT_R5]
    ldr     r4, [r4, #OFFSET_CONTEXT_R4]

#if PROTECT_RUNTIME
    /* restore access to all domains */
    cpsid   aif, #0x1f
    push    { r0 }
    mov     r0, #0x1  
    mcr     p15, 0, r0, c3, c0, 0
    isb
    pop     { r0 }
    cpsid   aif, #0x13
#endif

    subs    pc, lr, #0
.endfunc

/* [0x3] Prefetch Abort ===================================================== */

.func
handler_prefetch_report_host:
    bkpt    #0x3
.endfunc

.func
handler_prefetch_entry_exit:
    /* go to sys mode and enable AIF */
    cpsid   aif, #0x1f

    /* hack: temporarily use firmware's stack to save r4 */
    push    { r4 }

#if PROTECT_RUNTIME
    /* enable access to all domains */
    mov     r4, #0xf 
    mcr     p15, 0, r4, c3, c0, 0  
    isb 
#endif

    /* store context */
	ldr     r4, =RUNTIME_firmware_context
    stmia   r4, { r0-r3 }
    str     r5, [r4, #OFFSET_CONTEXT_R5]
    /* hack: pop r4 to r5 and store r4 */
    pop     { r5 }
    str     r5, [r4, #OFFSET_CONTEXT_R4]
    mov     r5, r4
    add     r5, r5, #OFFSET_CONTEXT_R6
    stmia   r5, { r6-r12, sp, lr }
    /* lr and spsr is banked in ARMv8-A, go back to svc mode and find them */
    cpsid   aif, #0x17
    mov     r5, lr
    sub     r5, r5, #0x4
    str     r5, [r4, #OFFSET_CONTEXT_PC]
    mrs     r5, spsr
    str     r5, [r4, #OFFSET_CONTEXT_XPSR]
    cpsid   aif, #0x1f

    /* change stack to the runtime stack */
    ldr     sp, =_end_rt_stack

    /* dive into runtime */
    /* R0 = DFSR R1 = DFAR */
    ldr     r4, =prefetch_handler
    blx     r4

    /* checks if there's exception to take  */
    ldr     r4, =cpu_check_irq
    blx     r4

    /* restore context */
    ldr     r4, =RUNTIME_firmware_context
    ldmia   r4, { r0-r3 }
    mov     r5, r4
    add     r5, r5, #OFFSET_CONTEXT_R6
    ldmia   r5, { r6-r12, sp, lr }


    /* go back to abt mode and prepare eret */
    /* hack: exception handler will set right PC for us */
    cpsid   aif, #0x17
    ldr     lr, [r4, #OFFSET_CONTEXT_PC]
    ldr     r5, [r4, #OFFSET_CONTEXT_XPSR]
    msr     spsr_cxsf, r5
    ldr     r5, [r4, #OFFSET_CONTEXT_R5]
    ldr     r4, [r4, #OFFSET_CONTEXT_R4]

#if PROTECT_RUNTIME
    /* restore access to all domains */
    cpsid   aif, #0x1f
    push    { r0 }
    mov     r0, #0x1  
    mcr     p15, 0, r0, c3, c0, 0
    isb
    pop     { r0 }
    cpsid   aif, #0x17
#endif

    subs    pc, lr, #0
.endfunc

.func
handler_prefetch_fail:
    /* restore state to enception entry */
	ldr     r4, =RUNTIME_firmware_context
    ldmia   r4, { r0-r3 }
    mov     r5, r4
    add     r5, r5, #OFFSET_CONTEXT_R6
    ldmia   r5, { r6-r12, sp, lr }


    cpsid   aif, #0x17
    ldr     r5, [r4, #OFFSET_CONTEXT_PC]
    add     lr, r5, #0x4
    ldr     r5, [r4, #OFFSET_CONTEXT_XPSR]
    msr     spsr_cxsf, r5
    ldr     r5, [r4, #OFFSET_CONTEXT_R5]
    ldr     r4, [r4, #OFFSET_CONTEXT_R4]

    /* go back to host */
    bkpt    #0x3
.endfunc

/* [0x4] Data Abort ========================================================= */

.func
handler_dabt_report_host:
    bkpt    #0x4
.endfunc

.func
handler_dabt_entry_exit:
    /* go to sys mode and enable AIF */
    cpsid   aif, #0x1f

    /* hack: temporarily use firmware's stack to save r4 */
    push    { r4 }

    /* enable access to all domains */
    mov     r4, #0xf 
    mcr     p15, 0, r4, c3, c0, 0  
    isb 

    /* store context */
    ldr     r4, =RUNTIME_firmware_context
	stmia	r4,	{ r0-r3 }
	str     r5, [r4, #OFFSET_CONTEXT_R5]
    /* hack: pop r4 to r5 and store r4 */
    pop     { r5 }
    str     r5, [r4, #OFFSET_CONTEXT_R4]
	mov		r5, r4
	add     r5, r5, #OFFSET_CONTEXT_R6
	stmia   r5, { r6-r12, sp, lr }
    /* lr and spsr is banked in ARMv8-A, go back to abt mode and find them */
    cpsid   aif, #0x17
    mov     r5, lr
    sub     r5, r5, #0x8
    str     r5, [r4, #OFFSET_CONTEXT_PC]
    mrs     r5, spsr
    str     r5, [r4, #OFFSET_CONTEXT_XPSR]
    cpsid   aif, #0x1f

    /* change stack to the runtime stack */
    ldr     sp, =_end_rt_stack

    /* dive into runtime */
    /* R0 = DFSR R1 = DFAR */
    mrc     p15, 0, r1, c6, c0, 0
    mrc     p15, 0, r0, c5, c0, 0
    ldr     r4, =dabt_handler
    blx     r4

    /* checks if there's exception to take  */
    ldr     r4, =cpu_check_irq
    blx     r4

    /* restore context */
    ldr     r4, =RUNTIME_firmware_context
	ldmia	r4, { r0-r3 }
	mov		r5, r4
	add		r5, r5, #OFFSET_CONTEXT_R6
	ldmia	r5, { r6-r12, sp, lr }
    /* go back to abt mode and prepare eret */
    /* hack: exception handler will set right PC for us */
    cpsid   aif, #0x17
    ldr     lr, [r4, #OFFSET_CONTEXT_PC]
    ldr     r5, [r4, #OFFSET_CONTEXT_XPSR]
    msr     spsr_cxsf, r5
    ldr     r5, [r4, #OFFSET_CONTEXT_R5]
    ldr     r4, [r4, #OFFSET_CONTEXT_R4]
    
    /* restore access to all domains */
    cpsid   aif, #0x1f
    push    { r0 }
    mov     r0, #0x1  
    mcr     p15, 0, r0, c3, c0, 0
    isb
    pop     { r0 }
    cpsid   aif, #0x17

    subs    pc, lr, #0
.endfunc

.func
handler_dabt_fail:
    /* restore state to enception entry */
	ldr     r4, =RUNTIME_firmware_context
    ldmia   r4, { r0-r3 }
    mov     r5, r4
    add     r5, r5, #OFFSET_CONTEXT_R6
    ldmia   r5, { r6-r12, sp, lr }


    cpsid   aif, #0x17
    ldr     r5, [r4, #OFFSET_CONTEXT_PC]
    add     lr, r5, #0x8
    ldr     r5, [r4, #OFFSET_CONTEXT_XPSR]
    msr     spsr_cxsf, r5
    ldr     r5, [r4, #OFFSET_CONTEXT_R5]
    ldr     r4, [r4, #OFFSET_CONTEXT_R4]

    /* go back to host */
    bkpt    #0x4
.endfunc

/* [0x6] IRQ ================================================================ */

.func
handler_irq_report_host:
    cpsid   aif, #0x1f
    /* ack to the gic controller */
    push    {r0, r1}
    mov     r0, #0x100c
    movt    r0, #0xe010
    ldr     r1, [r0]
    mov     r0, #0x1010
    movt    r0, #0xe010
    str     r1, [r0]
    pop     {r0, r1}
    cpsie   aif, #0x1f
    bkpt    #0x5
.endfunc

.func
handler_irq_entry_exit:
    /* go to sys mode, acknowledge irq and disable AIF */
    cpsid   aif, #0x1f
    push    {r0, r1}
    mov     r0, #0x100c
    movt    r0, #0xe010
    ldr     r1, [r0]
    mov     r0, #0x1010
    movt    r0, #0xe010
    str     r1, [r0]
    pop     {r0, r1}

    /* hack: temporarily use firmware's stack to save r4 */
    push    { r4 }

#if PROTECT_RUNTIME
    /* enable access to all domains */
    mov     r4, #0xf 
    mcr     p15, 0, r4, c3, c0, 0  
    isb 
#endif

    /* store context */
	ldr     r4, =RUNTIME_firmware_context
    stmia   r4, { r0-r3 }
    str     r5, [r4, #OFFSET_CONTEXT_R5]
    /* hack: pop r4 to r5 and store r4 */
    pop     { r5 }
    str     r5, [r4, #OFFSET_CONTEXT_R4]
    mov     r5, r4
    add     r5, r5, #OFFSET_CONTEXT_R6
    stmia   r5, { r6-r12, sp, lr }
    /* lr and spsr is banked in ARMv8-A, go back to irq mode and find them */
    cpsid   aif, #0x12
    mov     r5, lr
    sub     r5, r5, #0x4
    str     r5, [r4, #OFFSET_CONTEXT_PC]
    mrs     r5, spsr
    str     r5, [r4, #OFFSET_CONTEXT_XPSR]
    cpsid   aif, #0x1f

    /* change stack to the runtime stack */    
    ldr     sp, =_end_rt_stack

    /* checks if there's exception to take  */
    ldr     r4, =cpu_check_irq
    blx     r4

    /* restore context */
	ldr     r4, =RUNTIME_firmware_context
    ldmia   r4, { r0-r3 }
    mov     r5, r4
    add     r5, r5, #OFFSET_CONTEXT_R6
    ldmia   r5, { r6-r12, sp, lr }


    /* go back to abt mode and prepare eret */
    /* hack: exception handler will set right PC for us */
    cpsid   aif, #0x12
    ldr     lr, [r4, #OFFSET_CONTEXT_PC]
    ldr     r5, [r4, #OFFSET_CONTEXT_XPSR]
    msr     spsr_cxsf, r5
    ldr     r5, [r4, #OFFSET_CONTEXT_R5]
    ldr     r4, [r4, #OFFSET_CONTEXT_R4]

#if PROTECT_RUNTIME
    /* restore access to all domains */
    cpsid   aif, #0x1f
    push    { r0 }
    mov     r0, #0x1  
    mcr     p15, 0, r0, c3, c0, 0
    isb
    pop     { r0 }
    cpsid   aif, #0x12
#endif

    subs    pc, lr, #0
.endfunc

/* [0x7] FIQ ================================================================ */

.func
handler_fiq_report_host:
    bkpt    #0x7
.endfunc
