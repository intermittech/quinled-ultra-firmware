ENTRY( _start )

__stack_size = 2048;

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
	/* 64 KB minus the current calibration page (256 B, 0xF600), the power-on scene log (2 KB, 0xF700) and the
	   settings page (256 B, 0xFF00), see src/hal/board.h */
	FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 0xF600
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
