/* branchpk3.c */

#include "..\PICkit3OS\include\processor.h"

#include "..\PICkit3OS\include\types.h"
#include "..\PICkit3OS\include\system.h"

//--------------------------------------------------------------------------------------
// FUSES
// Macros for Configuration Fuse Registers (copied from device header file): 
//
//_CONFIG3( WPEND_WPSTARTMEM & WPCFG_WPCFGDIS & WPDIS_WPDIS & WPEND_WPENDMEM )
//_CONFIG2( IESO_ON & PLLDIV_DIV3 & FNOSC_PRIPLL & FCKSM_CSECME & OSCIOFNC_OFF & IOL1WAY_OFF & DISUVREG_OFF & POSCMOD_HS )
//_CONFIG1( JTAGEN_OFF & GCP_OFF & ICS_PGx2 & FWDTEN_OFF )


#define PLL_96MHZ_ON    0xF7FF
_CONFIG2(IESO_OFF & PLL_96MHZ_ON & PLLDIV_DIV3 & FNOSC_PRIPLL & POSCMOD_HS & IOL1WAY_OFF)   // Primary HS OSC with PLL, USBPLL /3
_CONFIG1(JTAGEN_OFF & ICS_PGx2 & FWDTEN_OFF)        // JTAG off, watchdog timer off


//--------------------------------------------------------------------------------------
// Function prototypes
WORD ReadProgramLoc(DWORD Address);

#define     pin_BUTTON          _RB8
#define     tris_BUTTON         _TRISB8

#define		pin_LED_ACTIVE		_LATB9
#define		tris_LED_ACTIVE		_TRISB9


//--------------------------------------------------------------------------------------
// Main Branch called. Has nothing to do with Keeloq and steers the PK3 between going
// to the OS or a bootloader
//
int main(void)
{
    WORD 	data=0;
    BOOL	Jump2Program = TRUE;
        
	// push button can override jump
	AD1PCFG = 0xFFF2;			// turn off A2D double functions	
	tris_BUTTON = 1;
    
    data = ReadProgramLoc(PK3_BL_OS_SWITCH_ADDRESS);
    // TODO CRC check
    //Jump2Program = crcRegion();
    
    // now see if there is a button override    
    if( (data == PK3_JUMP_TO_BOOT) || !pin_BUTTON)
    {
	    Jump2Program = FALSE;
    }

	if(Jump2Program)
		__asm__ ("goto 0x820");	    
	else
	{
		// otherwise go to the bootloader
	    // Use alternative IVT table
		_ALTIVT = 1;
		
		#if 0
			while(1)
			{
				tris_LED_ACTIVE = 0; // output
				pin_LED_ACTIVE = 1;
			}	
		#else
			__asm__ ("goto 0xF020");	    
		#endif
	}	
}

WORD ReadProgramLoc(DWORD Address)
{
    register int addr;	
	register unsigned int val;	
	WORD oldPage;	

    oldPage = TBLPAG;
    // set the high byte
    TBLPAG = ((unsigned char *)&Address)[2];

	addr = (WORD)Address; 
	__asm__("tblrdl [%1], %0" : "=r"(val) : "r" (addr)); 
	
    TBLPAG = oldPage;

	return(val);  
}
