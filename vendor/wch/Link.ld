ENTRY( _start )

/* the stack: 2 KB. tools/build_target.sh STACK_SIZE=<bytes> (QL_STACK_SIZE, 0.6.6): 1536 is ready (the static worst
 * case 880 B with the ISRs nested, tools/stack.py) and waits for the painted `stack=` of the bench's 30-day soak */
__stack_size = DEFINED(QL_STACK_SIZE) ? QL_STACK_SIZE : 2048;

PROVIDE( _stack_size = __stack_size );


MEMORY
{  
/* CH32V20x_D6 - CH32V203F6-CH32V203G6-CH32V203C6 */
/*
	FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 32K
	RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 10K
*/ 

/* CH32V20x_D6 - CH32V203K8-CH32V203C8-CH32V203G8-CH32V203F8 */
/**/
	/* the zero-wait 64 KB (0.6.5: the record pages are in the upper CodeFlash, src/hal/board.h; 0.6.4's layout:
	   Link_lower.ld) */
	FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 0x10000
	RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 20K
 
  
/* CH32V20x_D8 - CH32V203RB
   CH32V20x_D8W - CH32V208x
   FLASH + RAM supports the following configuration
   FLASH-128K + RAM-64K
   FLASH-144K + RAM-48K
   FLASH-160K + RAM-32K

	FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 160K
	RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 32K
*/
}


INCLUDE Link_sections.ld
