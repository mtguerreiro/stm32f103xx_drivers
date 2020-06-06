/* Attention: Heap is not considered here */
MEMORY
{
    FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 512K
    RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 64K
}

/* Stack pointer */
__stack = ORIGIN(RAM) + LENGTH(RAM);

SECTIONS
{

    . = ORIGIN(FLASH);
    
    /* ISR vector table */
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } > FLASH
    
    /* Boot space */
    .boot :
    {
        . = ALIGN(4);
        KEEP(*(.boot))
        KEEP(*(.boot*))
        . = ALIGN(4);  
    } > FLASH
    
    /* Code */
    .text :
    {
        . = ALIGN(4);
        *(.text)
        *(.text*)
        *(.glue_7)
        *(.glue_7t)
        . = ALIGN(4);
    } > FLASH

	/* Read-only data */
    .rodata :
    {
        . = ALIGN(4);
        *(.rodata)
        *(.rodata*)
        . = ALIGN(4);
    } > FLASH
    
    /* ARM stack unwinding. For more, see below. */
    /* http://infocenter.arm.com/help/topic/com.arm.doc.ihi0044e/index.html */
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
    
    /* Initialized variables. Here, __sdata_flash points to the start of the */
    /* .data section in the flash memory, and __sdata_ram points to where */
    /* the .data section starts in the RAM memory. In addition, __eedata_ram */
    /* points to where the .data section ends in the RAM memory. */
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
    
    /* Unitialized variables. Here, __sbss points to where the .bss section */
    /* begins in the RAM memory, while __ebss points to where the .bss */
    /* section ends. */
    .bss : 
    {
        . = ALIGN(4);
        __sbss = .;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        __ebss = .;
        end = __ebss;
    } > RAM
    
    .ARM.attributes 0 : { *(.ARM.attributes) }   
}