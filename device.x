MEMORY
{
  FLASH(RX) : ORIGIN = 0x08000000, LENGTH = 64K
  RAM(WX) : ORIGIN = 0x20000000, LENGTH = 12K
  /* Note that SRAM2 is also mapped at 0x10000000. */
}

end_of_ram = ORIGIN(RAM) + LENGTH(RAM);
EXTERN(main);
ENTRY(main);

SECTIONS
{
  .text : {
     KEEP(*(.vectors*)),
     *(.text*)
     *(SORT_BY_ALIGNMENT(.rodata*))
  } > FLASH
  .data : {
     *(SORT_BY_ALIGNMENT(.rwdata*))
  } > RAM
  .bss (NOLOAD) : {
     __bss_start = .;
     *(SORT_BY_ALIGNMENT(.bss*))
     __bss_end = .;
  } > RAM
  .noinit (NOLOAD) : {
     *(SORT_BY_ALIGNMENT(.noinit*))
  } > RAM
  .stack_sizes (INFO): {
     KEEP(*(.stack_sizes));
  }
}
