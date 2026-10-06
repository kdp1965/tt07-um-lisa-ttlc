/*======================================================================*/
/* Linker script for nr5m100_nexys board with Newlib support            */
/*======================================================================*/

/*----------------------------------------------------------------------*/
/* Setup                                                                */
/*----------------------------------------------------------------------*/

/* The OUTPUT_ARCH command specifies the machine architecture where the
   argument is one of the names used in the BFD library. More
   specifically one of the entires in bfd/cpu-mips.c */

OUTPUT_ARCH( "lisa" )

/* Define the memory regions were we put stuff */
MEMORY
{
  code_sram (rx)  : ORIGIN = 0x0000, LENGTH = 8K 
  /*data_sram (rw)  : ORIGIN = 0x0004, LENGTH = 32764*/
  data_sram (rw)  : ORIGIN = 0x0004, LENGTH = 124
}


_stack_size = 32;

/* Define the beginning and ending stack */
_stack_start = ORIGIN(data_sram) + LENGTH(data_sram);
_stack_end   = _stack_start - _stack_size;


/* The ENTRY command specifies the entry point (ie. first instruction to
   execute). The symbol _start is defined in crt0.S */

ENTRY( reset_vec )

/*----------------------------------------------------------------------*/
/* Sections                                                             */
/*----------------------------------------------------------------------*/
SECTIONS
{
  /*--------------------------------------------------------------------*/
  /* Code and read-only segment                                         */
  /*--------------------------------------------------------------------*/
  .text : 
  {
    _vectors = .;
    KEEP(*(.text.vec))
    _evectors = .;

    /* Begining of code and text segment */
    _stext = .;
    *(.text)
    *(.text.*)

    KEEP( *(.init) )
    KEEP( *(.fini) )

    /* Read only data */
    *(.rodata)
    *(.rodata.*)
    _etext = .;
  } > code_sram

  /*--------------------------------------------------------------------*/
  /* Initialized data segment                                           */
  /*--------------------------------------------------------------------*/
  .data : 
  {
    _sdata = .;
    *(.data)
    *(.data.*)

    /* End of initialized data segment */
    _edata = .;
  } > data_sram AT > code_sram

  /*--------------------------------------------------------------------*/
  /* Uninitialized .bss data segment                                    */
  /*--------------------------------------------------------------------*/
  .bss : 
  {
    _bss_start = .;
    *(.bss)
    *(.bss.*)
    _bss_end = .;

    _virt_regs = .;
  } > data_sram

}


