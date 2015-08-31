;void __attribute__((__interrupt__)) _INT1Interrupt(void)
;void __attribute__((__interrupt__)) _INT2Interrupt(void)
;void __attribute__((__interrupt__)) _INT3Interrupt(void)
;void __attribute__((__interrupt__)) _CNInterrupt(void)


	.equ 	IVTTABLEBASE,0x824
	#define _BIG_IVT_



	.equ 	AVTTABLEBASE,0xF024


.section ivt_table,code
	; Names are incorrect for dspic33f and the pic24F
    .pword	IVTTABLEBASE + 0x00	;__ReservedTrap0
    .pword	IVTTABLEBASE + 0x04	;__OscillatorFail
    .pword	IVTTABLEBASE + 0x08	;__AddressError
    .pword	IVTTABLEBASE + 0x0C	;__StackError
    .pword	IVTTABLEBASE + 0x10	;__MathError   
    .pword	IVTTABLEBASE + 0x14	;__DMAErrorTrap
    .pword	IVTTABLEBASE + 0x18	;__ReservedTrap6
    .pword	IVTTABLEBASE + 0x1C	;__ReservedTrap7
    .pword	IVTTABLEBASE + 0x20	;__INT0Interrupt
    .pword	IVTTABLEBASE + 0x24	;__IC1Interrup
    .pword	IVTTABLEBASE + 0x28	;__OC1Interrup
    .pword	IVTTABLEBASE + 0x2c	;__T1Interrupt
    .pword	IVTTABLEBASE + 0x30	;__IC2Interrup
    .pword	IVTTABLEBASE + 0x34	;__OC2Interrup
    .pword	IVTTABLEBASE + 0x38	;__T2Interrupt
    .pword	IVTTABLEBASE + 0x3c	;__T3Interrupt
    .pword	IVTTABLEBASE + 0x40	;__SPI1Interrupt
    .pword	IVTTABLEBASE + 0x44	;__U1RXInterrupt
    .pword	IVTTABLEBASE + 0x48	;__U1TXInterrupt
    .pword	IVTTABLEBASE + 0x4c	;__ADCInterrupt
    .pword	IVTTABLEBASE + 0x50	;__NVMInterrupt
    .pword	IVTTABLEBASE + 0x54	;__SI2CInterrupt
    .pword	IVTTABLEBASE + 0x58	;__MI2CInterrupt
    .pword	IVTTABLEBASE + 0x5C	;__CNInterrupt
    .pword	IVTTABLEBASE + 0x60	;__INT1Interrupt
    .pword	IVTTABLEBASE + 0x64	;__IC7Interrup
    .pword	IVTTABLEBASE + 0x68	;__IC8Interrup
    .pword	IVTTABLEBASE + 0x6C	;__OC3Interrup
    .pword	IVTTABLEBASE + 0x70	;__OC4Interrup
    .pword	IVTTABLEBASE + 0x74	;__T4Interrupt
    .pword	IVTTABLEBASE + 0x78	;__T5Interrupt
    .pword	IVTTABLEBASE + 0x7C	;__INT2Interrupt
    .pword	IVTTABLEBASE + 0x80	;__U2RXInterrupt
    .pword	IVTTABLEBASE + 0x84	;__U2TXInterrupt
    .pword	IVTTABLEBASE + 0x88	;__SPI2Interrupt
    .pword	IVTTABLEBASE + 0x8C	;__C1Interrupt
    .pword	IVTTABLEBASE + 0x90	;__IC3Interrupt
    .pword	IVTTABLEBASE + 0x94	;__IC4Interrupt
    .pword	IVTTABLEBASE + 0x98	;__IC5Interrupt
    .pword	IVTTABLEBASE + 0x9C	;__IC6Interrupt
    .pword	IVTTABLEBASE + 0xA0	;__OC5Interrupt
    .pword	IVTTABLEBASE + 0xA4	;__OC6Interrupt
    .pword	IVTTABLEBASE + 0xA8	;__OC7Interrupt
    .pword	IVTTABLEBASE + 0xAC	;__OC8Interrupt
    .pword	IVTTABLEBASE + 0xB0	;__INT3Interrupt
    .pword	IVTTABLEBASE + 0xB4	;__INT4Interrupt
    .pword	IVTTABLEBASE + 0xB8	;__C2Interrupt
    .pword	IVTTABLEBASE + 0xBC	;__PWMInterrupt
    .pword	IVTTABLEBASE + 0xC0	;__QEIInterrupt
    .pword	IVTTABLEBASE + 0xC4	;__DCIInterrupt
    .pword	IVTTABLEBASE + 0xC8	;__LVDInterrupt
    .pword	IVTTABLEBASE + 0xCC	;__FLTAInterrupt
    .pword	IVTTABLEBASE + 0xD0	;__FLTBInterrupt
    .pword	IVTTABLEBASE + 0xD4	;__Interrupt45
    .pword	IVTTABLEBASE + 0xD8	;__Interrupt46
    .pword	IVTTABLEBASE + 0xDC	;__Interrupt47
    .pword	IVTTABLEBASE + 0xE0	;__Interrupt48
    .pword	IVTTABLEBASE + 0xE4	;__Interrupt49
    .pword	IVTTABLEBASE + 0xE8	;__Interrupt50
    .pword	IVTTABLEBASE + 0xEC	;__Interrupt51
    .pword	IVTTABLEBASE + 0xF0	;__Interrupt52
    .pword	IVTTABLEBASE + 0xF4	;__Interrupt53    
    
    ; start of additions for dspic33f (64)
#if defined(_BIG_IVT_)
    .pword	IVTTABLEBASE + 0xF8	;
    .pword	IVTTABLEBASE + 0xFC	;
    
    .pword	IVTTABLEBASE + 0x100	;
    .pword	IVTTABLEBASE + 0x104	;
    .pword	IVTTABLEBASE + 0x108	;
    .pword	IVTTABLEBASE + 0x10C	;
    
    .pword	IVTTABLEBASE + 0x110	;
    .pword	IVTTABLEBASE + 0x114	;
    .pword	IVTTABLEBASE + 0x118	;
    .pword	IVTTABLEBASE + 0x11C	;
    
    .pword	IVTTABLEBASE + 0x120	;
    .pword	IVTTABLEBASE + 0x124	;
    .pword	IVTTABLEBASE + 0x128	;
    .pword	IVTTABLEBASE + 0x12C	;
    
    .pword	IVTTABLEBASE + 0x130	;
    .pword	IVTTABLEBASE + 0x134	;
    .pword	IVTTABLEBASE + 0x138	;
    .pword	IVTTABLEBASE + 0x13C	;
    
    .pword	IVTTABLEBASE + 0x140	;
    .pword	IVTTABLEBASE + 0x144	;
    .pword	IVTTABLEBASE + 0x148	;
    .pword	IVTTABLEBASE + 0x14C	;
    
    .pword	IVTTABLEBASE + 0x150	;
    .pword	IVTTABLEBASE + 0x154	;
    .pword	IVTTABLEBASE + 0x158	;
    .pword	IVTTABLEBASE + 0x15C	;
    
    .pword	IVTTABLEBASE + 0x160	;
    .pword	IVTTABLEBASE + 0x164	;
    .pword	IVTTABLEBASE + 0x168	;
    .pword	IVTTABLEBASE + 0x16C	;
    
    .pword	IVTTABLEBASE + 0x170	;
    .pword	IVTTABLEBASE + 0x174	;
    .pword	IVTTABLEBASE + 0x178	;
    .pword	IVTTABLEBASE + 0x17C	;
    
    .pword	IVTTABLEBASE + 0x180	;
    .pword	IVTTABLEBASE + 0x184	;
    .pword	IVTTABLEBASE + 0x188	;
    .pword	IVTTABLEBASE + 0x18C	;
    
    .pword	IVTTABLEBASE + 0x190	;
    .pword	IVTTABLEBASE + 0x194	;
    .pword	IVTTABLEBASE + 0x198	;
    .pword	IVTTABLEBASE + 0x19C	;
    
    .pword	IVTTABLEBASE + 0x1A0	;
    .pword	IVTTABLEBASE + 0x1A4	;
    .pword	IVTTABLEBASE + 0x1A8	;
    .pword	IVTTABLEBASE + 0x1AC	;
    
    .pword	IVTTABLEBASE + 0x1B0	;
    .pword	IVTTABLEBASE + 0x1B4	;
    .pword	IVTTABLEBASE + 0x1B8	;
    .pword	IVTTABLEBASE + 0x1BC	;
    
    .pword	IVTTABLEBASE + 0x1C0	;
    .pword	IVTTABLEBASE + 0x1C4	;
    .pword	IVTTABLEBASE + 0x1C8	;
    .pword	IVTTABLEBASE + 0x1CC	;
    
    .pword	IVTTABLEBASE + 0x1D0	;
    .pword	IVTTABLEBASE + 0x1D4	;
    .pword	IVTTABLEBASE + 0x1D8	;
    .pword	IVTTABLEBASE + 0x1DC	;
    
    .pword	IVTTABLEBASE + 0x1E0	;
    .pword	IVTTABLEBASE + 0x1E4	;
    .pword	IVTTABLEBASE + 0x1E8	;
    .pword	IVTTABLEBASE + 0x1EC	;

    .pword	IVTTABLEBASE + 0x1F0	;
    .pword	IVTTABLEBASE + 0x1F4	;
#endif
    
.section avt_table,code    
	; Names are incorrect for dspic33f
	.pword	AVTTABLEBASE + 0x00	;__AltReservedTrap0
    .pword	AVTTABLEBASE + 0x04	;__AltOscillatorFail
    .pword	AVTTABLEBASE + 0x08	;__AltAddressError
    .pword	AVTTABLEBASE + 0x0c	;__AltStackError
    .pword	AVTTABLEBASE + 0x10	;__AltMathError
    .pword	AVTTABLEBASE + 0x14	;__AltReservedTrap5
    .pword	AVTTABLEBASE + 0x18	;__AltReservedTrap6
    .pword	AVTTABLEBASE + 0x1c	;__AltReservedTrap7
    
    .pword	AVTTABLEBASE + 0x20	;__AltINT0Interrupt
    .pword	AVTTABLEBASE + 0x24	;__AltIC1Interrupt
    .pword	AVTTABLEBASE + 0x28	;__AltOC1Interrupt
    .pword	AVTTABLEBASE + 0x2c	;__AltT1Interrupt
    .pword	AVTTABLEBASE + 0x30	;__AltIC2Interrupt
    .pword	AVTTABLEBASE + 0x34	;__AltOC2Interrupt
    .pword	AVTTABLEBASE + 0x38	;__AltT2Interrupt
    .pword	AVTTABLEBASE + 0x3c	;__AltT3Interrupt
    
    .pword	AVTTABLEBASE + 0x40	;__AltSPI1Interrupt
    .pword	AVTTABLEBASE + 0x44	;__AltU1RXInterrupt
    .pword	AVTTABLEBASE + 0x48	;__AltU1TXInterrupt
    .pword	AVTTABLEBASE + 0x4c	;__AltADCInterrupt
    .pword	AVTTABLEBASE + 0x50	;__AltNVMInterrupt
    .pword	AVTTABLEBASE + 0x54 ;__AltSI2CInterrupt
    .pword	AVTTABLEBASE + 0x5A	;__AltMI2CInterrupt
    .pword	AVTTABLEBASE + 0x5C	;__AltCNInterrupt
    
    .pword	AVTTABLEBASE + 0x60	;__AltINT1Interrupt
    .pword	AVTTABLEBASE + 0x64	;__AltIC7Interrupt
    .pword	AVTTABLEBASE + 0x68	;__AltIC8Interrupt
    .pword	AVTTABLEBASE + 0x6C	;__AltOC3Interrupt
    .pword	AVTTABLEBASE + 0x70	;__AltOC4Interrupt
    .pword	AVTTABLEBASE + 0x74	;__AltT4Interrupt
    .pword	AVTTABLEBASE + 0x78	;__AltT5Interrupt
    .pword	AVTTABLEBASE + 0x7C	;__AltINT2Interrupt
        
    .pword	AVTTABLEBASE + 0x80	;__AltU2RXInterrupt
    .pword	AVTTABLEBASE + 0x84	;__AltU2TXInterrupt
    .pword	AVTTABLEBASE + 0x88	;__AltSPI2Interrupt
    .pword	AVTTABLEBASE + 0x8C	;__AltC1Interrupt
    .pword	AVTTABLEBASE + 0x90	;__AltIC3Interrupt
    .pword	AVTTABLEBASE + 0x94	;__AltIC4Interrupt
    .pword	AVTTABLEBASE + 0x98	;__AltIC5Interrupt
    .pword	AVTTABLEBASE + 0x9C	;__AltIC6Interrupt
    
    .pword	AVTTABLEBASE + 0xA0	;__AltOC5Interrupt
    .pword	AVTTABLEBASE + 0xa4	;__AltOC6Interrupt
    .pword	AVTTABLEBASE + 0xa8	;__AltOC7Interrupt
    .pword	AVTTABLEBASE + 0xaC	;__AltOC8Interrupt
    .pword	AVTTABLEBASE + 0xB0	;__AltINT3Interrupt
    .pword	AVTTABLEBASE + 0xb4	;__AltINT4Interrupt
    .pword	AVTTABLEBASE + 0xb8	;__AltC2Interrupt
    .pword	AVTTABLEBASE + 0xbC	;__AltPWMInterrupt
    
    .pword	AVTTABLEBASE + 0xc0	;__AltQEIInterrupt
    .pword	AVTTABLEBASE + 0xc4	;__AltDCIInterrupt
    .pword	AVTTABLEBASE + 0xc8	;__AltLVDInterrupt
    .pword	AVTTABLEBASE + 0xcC	;__AltFLTAInterrupt
    .pword	AVTTABLEBASE + 0xd0	;__AltFLTBInterrupt
    .pword	AVTTABLEBASE + 0xd4	;__AltInterrupt45
    .pword	AVTTABLEBASE + 0xd8	;__AltInterrupt46
    .pword	AVTTABLEBASE + 0xdC	;__AltInterrupt47
    
    .pword	AVTTABLEBASE + 0xe0	;__AltInterrupt48
    .pword	AVTTABLEBASE + 0xe4	;__AltInterrupt49
    .pword	AVTTABLEBASE + 0xe8	;__AltInterrupt50
    .pword	AVTTABLEBASE + 0xeC	;__AltInterrupt51
    .pword	AVTTABLEBASE + 0xf0	;__AltInterrupt52
    .pword	AVTTABLEBASE + 0xf4	;__AltInterrupt53

#if defined(_BIG_IVT_)
   ; start of additions for dspic33f (64)
   .pword	AVTTABLEBASE + 0xF8	;
    .pword	AVTTABLEBASE + 0xFC	;
    
    .pword	AVTTABLEBASE + 0x100	;
    .pword	AVTTABLEBASE + 0x104	;
    .pword	AVTTABLEBASE + 0x108	;
    .pword	AVTTABLEBASE + 0x10C	;
    
    .pword	AVTTABLEBASE + 0x110	;
    .pword	AVTTABLEBASE + 0x114	;
    .pword	AVTTABLEBASE + 0x118	;
    .pword	AVTTABLEBASE + 0x11C	;
    
    .pword	AVTTABLEBASE + 0x120	;
    .pword	AVTTABLEBASE + 0x124	;
    .pword	AVTTABLEBASE + 0x128	;
    .pword	AVTTABLEBASE + 0x12C	;
    
    .pword	AVTTABLEBASE + 0x130	;
    .pword	AVTTABLEBASE + 0x134	;
    .pword	AVTTABLEBASE + 0x138	;
    .pword	AVTTABLEBASE + 0x13C	;
    
    .pword	AVTTABLEBASE + 0x140	;
    .pword	AVTTABLEBASE + 0x144	;
    .pword	AVTTABLEBASE + 0x148	;
    .pword	AVTTABLEBASE + 0x14C	;
    
    .pword	AVTTABLEBASE + 0x150	;
    .pword	AVTTABLEBASE + 0x154	;
    .pword	AVTTABLEBASE + 0x158	;
    .pword	AVTTABLEBASE + 0x15C	;
    
    .pword	AVTTABLEBASE + 0x160	;
    .pword	AVTTABLEBASE + 0x164	;
    .pword	AVTTABLEBASE + 0x168	;
    .pword	AVTTABLEBASE + 0x16C	;
    
    .pword	AVTTABLEBASE + 0x170	;
    .pword	AVTTABLEBASE + 0x174	;
    .pword	AVTTABLEBASE + 0x178	;
    .pword	AVTTABLEBASE + 0x17C	;
    
    .pword	AVTTABLEBASE + 0x180	;
    .pword	AVTTABLEBASE + 0x184	;
    .pword	AVTTABLEBASE + 0x188	;
    .pword	AVTTABLEBASE + 0x18C	;
    
    .pword	AVTTABLEBASE + 0x190	;
    .pword	AVTTABLEBASE + 0x194	;
    .pword	AVTTABLEBASE + 0x198	;
    .pword	AVTTABLEBASE + 0x19C	;
    
    .pword	AVTTABLEBASE + 0x1A0	;
    .pword	AVTTABLEBASE + 0x1A4	;
    .pword	AVTTABLEBASE + 0x1A8	;
    .pword	AVTTABLEBASE + 0x1AC	;
    
    .pword	AVTTABLEBASE + 0x1B0	;
    .pword	AVTTABLEBASE + 0x1B4	;
    .pword	AVTTABLEBASE + 0x1B8	;
    .pword	AVTTABLEBASE + 0x1BC	;
    
    .pword	AVTTABLEBASE + 0x1C0	;
    .pword	AVTTABLEBASE + 0x1C4	;
    .pword	AVTTABLEBASE + 0x1C8	;
    .pword	AVTTABLEBASE + 0x1CC	;
    
    .pword	AVTTABLEBASE + 0x1D0	;
    .pword	AVTTABLEBASE + 0x1D4	;
    .pword	AVTTABLEBASE + 0x1D8	;
    .pword	AVTTABLEBASE + 0x1DC	;
    
    .pword	AVTTABLEBASE + 0x1E0	;
    .pword	AVTTABLEBASE + 0x1E4	;
    .pword	AVTTABLEBASE + 0x1E8	;
    .pword	AVTTABLEBASE + 0x1EC	;

    .pword	AVTTABLEBASE + 0x1F0	;
    .pword	AVTTABLEBASE + 0x1F4	;
#endif
