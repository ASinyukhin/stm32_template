/* Entry Point */
ENTRY(Reset_Handler)

__mcu_flash_size = 64K;
__mcu_ram_size = 20K;

__mcu_flash_page_size = 1K;
__mcu_flash_eeprom_pages = 1;


/* Generate a link error if heap and stack don't fit into RAM */
_Min_Heap_Size = 0;      /* required amount of heap  */
_Min_Stack_Size = 0x100; /* required amount of stack */

__mcu_flash_rom_offset = 0x08000000;
__mcu_ram_offset = 0x20000000;

__mcu_flash_eeprom_size = __mcu_flash_eeprom_pages * __mcu_flash_page_size;
__mcu_flash_rom_size = __mcu_flash_size - __mcu_flash_eeprom_size;
__mcu_flash_eeprom_offset = __mcu_flash_rom_offset + __mcu_flash_rom_size;

/* Highest address of the user mode stack */
_estack = __mcu_ram_offset + __mcu_ram_size;    /* end of RAM */

/* Specify the memory areas */
MEMORY
{
  rom (rx)      :   ORIGIN = __mcu_flash_rom_offset,     LENGTH = __mcu_flash_rom_size
  flash(rxw)    :   ORIGIN = __mcu_flash_eeprom_offset,  LENGTH = __mcu_flash_eeprom_size
  ram (xrw)     :   ORIGIN = __mcu_ram_offset,           LENGTH = __mcu_ram_size
}



/* Section Definitions */ 
SECTIONS 
{ 
    .text : 
    { 
        KEEP(*(.isr_vector .isr_vector.*))
        *(.text .text.* .gnu.linkonce.t.*)
        *(.glue_7t) *(.glue_7)
        *(.rodata .rodata* .gnu.linkonce.r.*)
    } > rom
    
    .ARM.extab : 
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > rom
    
    .ARM.exidx :
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > rom


    . = ALIGN(4);
    _etext = .;
    _sidata = .; 
    		
    .data : AT (_etext) 
    { 
        _sdata = .; 
        *(.data .data.*) 
        . = ALIGN(4); 
        _edata = . ;
    } > ram

    /* .bss section which is used for uninitialized data */ 
    .bss (NOLOAD) : 
    { 
        _sbss = . ; 
        *(.bss .bss.*) 
        *(COMMON) 
        . = ALIGN(4); 
        _ebss = . ; 
    } > ram
    
    .ram_storage (NOLOAD) :
    {
        . = ALIGN(4);
        *(.ram_storage .ram_storage.*)
    } > ram
    
    /* stack section */
    .co_stack (NOLOAD):
    {
        . = ALIGN(8);
        *(.co_stack .co_stack.*)
    } > ram
       
    . = ALIGN(4); 
    _end = . ; 
} 
