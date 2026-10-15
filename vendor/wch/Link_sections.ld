/* Output sections shared by Link.ld (standalone image at 0x00000000) and Link_app.ld (the app behind the
 * bootloader at 0x00002100, src/core/ultra_update.h). Included by both; MEMORY comes from the including script. */

PHDRS
{
    flash PT_LOAD FLAGS(5);
    ramcode PT_LOAD FLAGS(5);
    ramdata PT_LOAD FLAGS(6);
}

SECTIONS
{

	.init :
	{
		_sinit = .;
		. = ALIGN(4);
		KEEP(*(SORT_NONE(.init)))
		. = ALIGN(4);
		_einit = .;
	} >FLASH AT>FLASH :flash

  .vector :
  {
      *(.vector);
	  . = ALIGN(4);		/* QuinLED: was ALIGN(64): only pads the start of .text (mtvec's base needs 4 bytes) */
  } >FLASH AT>FLASH :flash

	.text :
	{
		. = ALIGN(4);
		INCLUDE text_order.ld		/* QuinLED (round 9): the code's order for the c.jal / c.j reach (tools/link_order.py; empty: the input order) */
		*(.text)
		*(.text.*)
		*(.rodata)
		*(.rodata*)
		*(.srodata .srodata.*)		/* small constants: flash, as .rodata (not copied into RAM) */
		*(.gnu.linkonce.t.*)
		. = ALIGN(4);
	} >FLASH AT>FLASH :flash 

	.fini :
	{
		KEEP(*(SORT_NONE(.fini)))
		. = ALIGN(4);
	} >FLASH AT>FLASH :flash

	PROVIDE( _etext = . );
	PROVIDE( _eitcm = . );	

	.preinit_array  :
	{
	  PROVIDE_HIDDEN (__preinit_array_start = .);
	  KEEP (*(.preinit_array))
	  PROVIDE_HIDDEN (__preinit_array_end = .);
	} >FLASH AT>FLASH :flash 
	
	.init_array     :
	{
	  PROVIDE_HIDDEN (__init_array_start = .);
	  KEEP (*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
	  KEEP (*(.init_array EXCLUDE_FILE (*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o ) .ctors))
	  PROVIDE_HIDDEN (__init_array_end = .);
	} >FLASH AT>FLASH :flash 
	
	.fini_array     :
	{
	  PROVIDE_HIDDEN (__fini_array_start = .);
	  KEEP (*(SORT_BY_INIT_PRIORITY(.fini_array.*) SORT_BY_INIT_PRIORITY(.dtors.*)))
	  KEEP (*(.fini_array EXCLUDE_FILE (*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o ) .dtors))
	  PROVIDE_HIDDEN (__fini_array_end = .);
	} >FLASH AT>FLASH :flash 
	
	.ctors          :
	{
	  /* gcc uses crtbegin.o to find the start of
	     the constructors, so we make sure it is
	     first.  Because this is a wildcard, it
	     doesn't matter if the user does not
	     actually link against crtbegin.o; the
	     linker won't look for a file to match a
	     wildcard.  The wildcard also means that it
	     doesn't matter which directory crtbegin.o
	     is in.  */
	  KEEP (*crtbegin.o(.ctors))
	  KEEP (*crtbegin?.o(.ctors))
	  /* We don't want to include the .ctor section from
	     the crtend.o file until after the sorted ctors.
	     The .ctor section from the crtend file contains the
	     end of ctors marker and it must be last */
	  KEEP (*(EXCLUDE_FILE (*crtend.o *crtend?.o ) .ctors))
	  KEEP (*(SORT(.ctors.*)))
	  KEEP (*(.ctors))
	} >FLASH AT>FLASH :flash 
	
	.dtors          :
	{
	  KEEP (*crtbegin.o(.dtors))
	  KEEP (*crtbegin?.o(.dtors))
	  KEEP (*(EXCLUDE_FILE (*crtend.o *crtend?.o ) .dtors))
	  KEEP (*(SORT(.dtors.*)))
	  KEEP (*(.dtors))
	} >FLASH AT>FLASH :flash 

	.dalign :
	{
		. = ALIGN(4);
		PROVIDE(_data_vma = .);
	} >RAM AT>FLASH :ramcode	

	.dlalign :
	{
		. = ALIGN(4); 
		PROVIDE(_data_lma = .);
	} >FLASH AT>FLASH :flash

    /* Included in startup's existing contiguous _data_lma -> _data_vma copy. */
    .ramtext :
    {
        . = ALIGN(4);
        __ramtext_start = .;
        KEEP(*(SORT_NONE(.ramtext) SORT_NONE(.ramtext.*))) /* keep ISR order with --sort-section=alignment */
        . = ALIGN(4);
        __ramtext_end = .;
    } >RAM AT>FLASH :ramcode

	.data :
	{
    	*(.gnu.linkonce.r.*)
    	*(.data .data.*)
    	*(.gnu.linkonce.d.*)
        /* Startup copies words; each input section keeps its own required
         * alignment (including any future 8-byte .sdata). Avoid an extra pad. */
        . = ALIGN(4);
    	PROVIDE( __global_pointer$ = . + 0x600 );	/* QuinLED: not + 0x800. gp's +-2 KB window then starts 0x200
							   below the small data: all of .sdata / .sbss (~3.1 KB) is in it,
							   clear of the reserve the linker keeps at the window's edges */
    	*(SORT_BY_ALIGNMENT(.sdata) SORT_BY_ALIGNMENT(.sdata.*))	/* QuinLED: by alignment, here and in .sbss: LTO
											   emits the variables in its own order (less padding) */
		*(.sdata2.*)
    	*(.gnu.linkonce.s.*)
    	. = ALIGN(4);
		PROVIDE( _edata = .);
	} >RAM AT>FLASH :ramdata

	.bss :
	{
		. = ALIGN(4);
		PROVIDE( _sbss = .);
  	    *(SORT_BY_ALIGNMENT(.sbss*))
        *(.gnu.linkonce.sb.*)
		*(.bss*)
     	*(.gnu.linkonce.b.*)		
		*(COMMON*)
		. = ALIGN(4);
		PROVIDE( _ebss = .);
	} >RAM AT>FLASH :ramdata

	PROVIDE( _end = _ebss);
	ASSERT(_ebss <= ORIGIN(RAM) + LENGTH(RAM) - __stack_size, "RAM overlaps reserved stack")
	/* (0.6.5) every byte of the image (code, constants, the SRAM code's and .data's load copies) in the zero-wait
	   64 KB: the flash helpers and everything timing-critical run from there; the upper CodeFlash holds records only.
	   QL_DIAG_BIG_SLOT: a measurement-only link (never flashed) may define it to go beyond */
	ASSERT(DEFINED(QL_DIAG_BIG_SLOT) || ORIGIN(FLASH) + LENGTH(FLASH) <= 0x10000, "the image must stay in the zero-wait flash")
	PROVIDE( end = . );

	/* QuinLED (0.6.5): the reset breadcrumbs (main.c bcrumb), 32 bytes no reset clears, right below the stack
	   (0x200047E0 with the 2 KB stack; 0x200049E0 with STACK_SIZE=1536): outside the hand-over block (0x20000000-1F)
	   and the bootloader's RAM (bootloader/bl.ld: its buffers end at 0x20004274, its stack starts at 0x20004A00:
	   asserted; test/test_d1_map.py). The startup neither copies nor clears it. One 4 KB %hi page: isr.S stores with one
	   lui. */
	/* (0.6.9, B2) 96 bytes: the breadcrumbs first (.noinit), then the event log (main.c evl, .noinit.evl) */
	.noinit ORIGIN(RAM) + LENGTH(RAM) - __stack_size - 96 (NOLOAD) :
	{
		KEEP(*(.noinit))
		KEEP(*(.noinit.*))
	} >RAM
	ASSERT(_ebss <= ADDR(.noinit) && SIZEOF(.noinit) <= 96, "RAM: .bss overlaps the reset breadcrumbs (.noinit)")
	ASSERT(((ADDR(.noinit) + 0x800) >> 12) == ((ADDR(.noinit) + 95 + 0x800) >> 12), ".noinit: one %hi page")
	ASSERT(ADDR(.noinit) + 96 <= 0x20004A00, ".noinit: below the bootloader's stack (a smaller STACK_SIZE moves it up)")

    .stack ORIGIN(RAM) + LENGTH(RAM) - __stack_size :
    {
        PROVIDE( _heap_end = . );   
        . = ALIGN(4);
        PROVIDE(_susrstack = . );
        . = . + __stack_size;
        PROVIDE( _eusrstack = .);
    } >RAM :ramdata

}
