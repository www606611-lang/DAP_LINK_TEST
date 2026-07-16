-uinterruptVectors
--stack_size=512

MEMORY
{
    FLASH      (RX)  : origin = 0x00000000, length = 0x00003000
    SRAM       (RWX) : origin = 0x20200000, length = 0x00007FE0
    BCR_CONFIG (R)   : origin = 0x41C00000, length = 0x000000FF
    BSL_CONFIG (R)   : origin = 0x41C00100, length = 0x00000080
}

SECTIONS
{
    .intvecs: > 0x00000000
    .text: palign(8) {} > FLASH
    .const: palign(8) {} > FLASH
    .cinit: palign(8) {} > FLASH
    .pinit: palign(8) {} > FLASH
    .rodata: palign(8) {} > FLASH
    .ARM.exidx: palign(8) {} > FLASH
    .init_array: palign(8) {} > FLASH
    .binit: palign(8) {} > FLASH
    .TI.ramfunc: load=FLASH, palign(8), run=SRAM, table(BINIT)
    .vtable: > SRAM
    .args: > SRAM
    .data: > SRAM
    .bss: > SRAM
    .sysmem: > SRAM
    .stack: > SRAM (HIGH)
    .BCRConfig: {} > BCR_CONFIG
    .BSLConfig: {} > BSL_CONFIG
}
