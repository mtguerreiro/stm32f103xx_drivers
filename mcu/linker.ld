/* Attention: Heap is not considered here */
MEMORY
{
    FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 128K
    RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 20K
}

/* Stack pointer */
__stack = ORIGIN(RAM) + LENGTH(RAM);

SECTIONS
{

    . = ORIGIN(FLASH);
    
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } > FLASH
    
    .boot :
    {
        . = ALIGN(4);
        KEEP(*(.boot))
        KEEP(*(.boot*))
        . = ALIGN(4);  
    } > FLASH
    
    .text :
    {
        . = ALIGN(4);
        *(.text)
        *(.text*)
        *(.glue_7)
        *(.glue_7t)
        . = ALIGN(4);
    } > FLASH

    .rodata :
    {
        . = ALIGN(4);
        *(.rodata)
        *(.rodata*)
        . = ALIGN(4);
    } > FLASH
    
    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > FLASH
    
    .ARM.exidx :
    {
        __exidx_start = .;
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        __exidx_end = .;
    } > FLASH   

    .preinit_array :
    {
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP (*(.preinit_array*))
        PROVIDE_HIDDEN (__preinit_array_end = .);
    } > FLASH
    .init_array :
    {
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP (*(SORT(.init_array.*)))
        KEEP (*(.init_array*))
        PROVIDE_HIDDEN (__init_array_end = .);
    } > FLASH
    .fini_array :
    {
        PROVIDE_HIDDEN (__fini_array_start = .);
        KEEP (*(SORT(.fini_array.*)))
        KEEP (*(.fini_array*))
        PROVIDE_HIDDEN (__fini_array_end = .);
    } > FLASH
    
    __sdata_flash = LOADADDR(.data);
    .data :
    {
        . = ALIGN(4);
        __sdata_ram = .;
        *(.data)
        *(.data*)
        . = ALIGN(4);
        __edata_ram = .;
    } > RAM AT > FLASH
    
    .bss : 
    {
        . = ALIGN(4);
        __sbss = .;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        __ebss = .;
    } > RAM
    
    .ARM.attributes 0 : { *(.ARM.attributes) }   
}