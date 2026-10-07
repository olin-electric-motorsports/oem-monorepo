/* STM32G441KB: 128 KiB flash, 22 KiB SRAM + a separate 10 KiB CCM bank.
 * Use only ordinary SRAM; do not place the stack past its physical end.
 * Symbols match common/startup.c. No bootloader offset is assumed. */
ENTRY(Reset_Handler)
MEMORY {
    FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 128K
    RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 22K
}
_estack = ORIGIN(RAM) + LENGTH(RAM);
SECTIONS {
    .isr_vector : { KEEP(*(.isr_vector)) } > FLASH
    .text : {
        *(.text*) *(.rodata*) *(.glue_7*)
        KEEP(*(.init)) KEEP(*(.fini))
        . = ALIGN(4);
    } > FLASH
    .ARM.extab : { *(.ARM.extab*) } > FLASH
    .ARM.exidx : {
        __exidx_start = .; *(.ARM.exidx*) __exidx_end = .;
    } > FLASH
    _sidata = LOADADDR(.data);
    .data : {
        . = ALIGN(4); _sdata = .;
        *(.data*) *(.RamFunc*)
        . = ALIGN(4); _edata = .;
    } > RAM AT > FLASH
    .bss (NOLOAD) : {
        . = ALIGN(4); _sbss = .;
        *(.bss*) *(COMMON)
        . = ALIGN(4); _ebss = .;
    } > RAM
    .heap_stack (NOLOAD) : {
        . = ALIGN(8); end = .; _end = .;
        . += 0x200; /* reserve heap */
        . += 0x800; /* reserve minimum stack */
    } > RAM
}
