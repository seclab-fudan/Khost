#ifndef DATA_OFFSET_H
#define DATA_OFFSET_H

#define OFFSET_BLOCK_COUNT      0x0
#define OFFSET_BLOCK_LIMIT      0x4
#define OFFSET_BLOCK_NEXT_IRQ   0x8
#define OFFSET_BLOCK_LAST_ID    0xc
#define OFFSET_RUNTIME_SP       0x10
#define OFFSET_PENDING_IRQ      0x14
#define OFFSET_MMIO_RANOUT      0x18
#define OFFSET_COVERAGE_MAP     0x1c
#define OFFSET_DEBUG_BUFFER     (0x1c + COVERAGE_MAP_SIZE)

#define OFFSET_CONTEXT_R0       0x00
#define OFFSET_CONTEXT_R1       0x04 
#define OFFSET_CONTEXT_R2       0x08 
#define OFFSET_CONTEXT_R3       0x0c 
#define OFFSET_CONTEXT_R4       0x10 
#define OFFSET_CONTEXT_R5       0x14 
#define OFFSET_CONTEXT_R6       0x18 
#define OFFSET_CONTEXT_R7       0x1c 
#define OFFSET_CONTEXT_R8       0x20 
#define OFFSET_CONTEXT_R9       0x24
#define OFFSET_CONTEXT_R10      0x28  
#define OFFSET_CONTEXT_R11      0x2c 
#define OFFSET_CONTEXT_R12      0x30 
#define OFFSET_CONTEXT_SP       0x34 
#define OFFSET_CONTEXT_LR       0x38 
#define OFFSET_CONTEXT_PC       0x3c 
#define OFFSET_CONTEXT_XPSR     0x40

#endif