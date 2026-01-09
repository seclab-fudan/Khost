/* linker script for runtime */

/* sets entry to the reset handler in exception table */
ENTRY(RUNTIME_vector_table)

/* size of the flash code */
_size_rt_flash = 16M;

/* size of the runtime stack */
_size_rt_stack = 1M;

/* size of the runtime heap */
_size_rt_heap = 1M;

/* sets memory region for the runtime */
MEMORY {
    /* runtime starts at ARMv7-M unused 0xf0000000 address space */
    FLASH (xr) : ORIGIN = 0xf0000000, LENGTH = _size_rt_flash
    /* we reserve 1M space for EXC_RETURN prefetch abort */
    RAM (rw) : ORIGIN = (0xf0000000 + _size_rt_flash), LENGTH = (255M - _size_rt_flash)
}

/* address space of the runtime
 * ############################################################################
 * #  .data  #  .bss  #       rt_heap        #            rt_stack            #
 * #         #        #                      # Reserved by _rt_min_stack_size #
 * ############################################################################
 * ^-- RAM start      ^-- _end_rt_bss                _end_rt_stack, RAM end --^
*/

/* highest address of the runtime stack */
_end_rt_stack = ORIGIN(RAM) + LENGTH(RAM);

/* runtime sections */
SECTIONS {
    /* page table of the runtime: align to 0x4000 and fill up with 0 */
    .rt_page_table : {
        . = ALIGN(0x4000);
        KEEP( *(.rt_page_table) )
        . = ALIGN(0x4000);
    } >FLASH =0

    /* vector table of the runtime: align to 0x20 and fill up with 0 */
    .rt_vector_table : {
        . = ALIGN(0x20);
        KEEP( *(.rt_vector_table) )
        . = ALIGN(0x20);
    } >FLASH =0

    /* common access area */
    .rt_dmz : {
        . = ALIGN(0x4);
        KEEP( *(.rt_dmz_handler) )
        KEEP( *(.rt_dmz_data) )
        *(.rodata)          /* .rodata sections (constants, strings, etc.) */
        *(.rodata*)         /* .rodata* sections (constants, strings, etc.) */
        . = ALIGN(0x100000);
    }

    /* text of the runtime: align to 0x4 and fill up with 0 */
    .text : {
        . = ALIGN(0x4);
        *(.text)            /* .text section (code) */          
        *(.text*)           /* .text* section (code) */
        *(.glue_7)          /* glue arm to thumb code */
        *(.glue_7t)         /* glue thumb to arm code */
        *(eh_frame)         /* exception backtrace for C++ */
        KEEP( *(.init) )
        KEEP( *(.fini) )
        . = ALIGN(0x4);
        _end_rt_text = .;   /* end of the text in runtime */
    } >FLASH =0

    /* staff for C++ exception handling */
    .ARM.extab : {
        . = ALIGN(0x4);
        *(.ARM.extab* .gnu.linkonce.armextab.*)
        . = ALIGN(0x4);
    } >FLASH = 0
    .ARM : {
        . = ALIGN(0x4);
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
        . = ALIGN(0x4);
    } >FLASH = 0

    /* staff for preinit_array */
    .preinit_array : {
        . = ALIGN(0x4);
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP (*(.preinit_array*))
        PROVIDE_HIDDEN (__preinit_array_end = .);
        . = ALIGN(4);
    } >FLASH = 0

    /* staff for init_array */
    .init_array : {
        . = ALIGN(0x4);
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP (*(SORT(.init_array.*)))
        KEEP (*(.init_array*))
        PROVIDE_HIDDEN (__init_array_end = .);
        . = ALIGN(0x4);
    } >FLASH = 0

    /* staff for fini_array */
    .fini_array : {
        . = ALIGN(0x4);
        PROVIDE_HIDDEN (__fini_array_start = .);
        KEEP (*(SORT(.fini_array.*)))
        KEEP (*(.fini_array*))
        PROVIDE_HIDDEN (__fini_array_end = .);
        . = ALIGN(0x4);
    } >FLASH = 0

    /* initialized data sections into "RAM" type memory */
    _start_rt_flash_data = LOADADDR(.data);
    .data : {
        . = ALIGN(0x4);
        _start_rt_data = .;     /* symbol at data start */
        *(.data)                /* .data sections */
        *(.data*)               /* .data* sections */
        . = ALIGN(0x4);
        _end_rt_data = .;       /* symbol at data end */
    } >RAM AT>FLASH = 0

    /* uninitialized data section into "RAM" type memory */
    .bss : {
        . = ALIGN(0x4);
        _start_rt_bss = .;
        __bss_start__ = _start_rt_bss;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(0x4);
        _end_rt_bss = .;         
        __bss_end__ = _end_rt_bss;
    } >RAM

    /* checks that there is enough "RAM" type memory left */
    .rt_heap_and_stack : {
        . = ALIGN(8);
        PROVIDE ( end = . );
        PROVIDE ( _end = . );
        . = . + _size_rt_heap;
        . = . + _size_rt_stack;
        . = ALIGN(8);
    } >RAM

    /* remove information from the compiler libraries */
    /DISCARD/ : {
        libc.a ( * )
        libm.a ( * )
        libgcc.a ( * )
    }
    .ARM.attributes 0 : { *(.ARM.attributes) }
}
