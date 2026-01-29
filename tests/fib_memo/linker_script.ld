ENTRY(_start)

SECTIONS
{
  . = 0x1000;          /* Start of the text section */
  .text : { *(.text) } /* All .text sections from input files */

  . = ALIGN(4);        /* Align to 4-byte boundary */
  .data : { *(.data) } /* All .data sections from input files */

  . = ALIGN(4);        /* Align to 4-byte boundary */
  .bss : { *(.bss) *(COMMON) } /* All .bss sections and COMMON symbols */

  /* Set the stack to start at address 0x8000 */
  . = 0x8000;
  PROVIDE(_stack = .);
}
