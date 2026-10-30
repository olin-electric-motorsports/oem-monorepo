/* STM32G441KB: 128 KiB flash, 22 KiB SRAM + a separate 10 KiB CCM bank.
 * Use only ordinary SRAM; do not place the stack past its physical end.
 * Symbols match common/startup.c. No bootloader offset is assumed. */
ENTRY(Reset_Handler)
/* Where flash and RAM are */
MEMORY {
    FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 128K
    RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 22K
}
_estack = ORIGIN(RAM) + LENGTH(RAM); /* the stack starts at the top of RAM */
SECTIONS {
    /* The interrupt vector table goes first, at the start of flash */
    .isr_vector : { KEEP(*(.isr_vector)) } > FLASH
    /* Code and constants */
    .text : {
        *(.text*) *(.rodata*) *(.glue_7*)
        KEEP(*(.init)) KEEP(*(.fini))
        . = ALIGN(4);
    } > FLASH
    /* Stack unwinding tables */
    .ARM.extab : { *(.ARM.extab*) } > FLASH
    .ARM.exidx : {
        __exidx_start = .; *(.ARM.exidx*) __exidx_end = .;
    } > FLASH
    /* Variables with starting values: kept in flash, copied to RAM at startup */
    _sidata = LOADADDR(.data);
    .data : {
        . = ALIGN(4); _sdata = .;
        *(.data*) *(.RamFunc*)
        . = ALIGN(4); _edata = .;
    } > RAM AT > FLASH
    /* Variables that start at zero */
    .bss (NOLOAD) : {
        . = ALIGN(4); _sbss = .;
        *(.bss*) *(COMMON)
        . = ALIGN(4); _ebss = .;
    } > RAM
    /* Room for the heap and the stack after the variables */
    .heap_stack (NOLOAD) : {
        . = ALIGN(8); end = .; _end = .;
        . += 0x200; /* reserve heap */
        . += 0x800; /* reserve minimum stack */
    } > RAM
}
