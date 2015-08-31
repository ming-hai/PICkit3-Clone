/*********************************************************************
 *
 * Interrupt vectors routines
 *
 *********************************************************************
 * FileName:            traps.c
 * Dependencies:        pickit3.h
 * Processor:           PIC24
 * Assembler/Compiler:  MPLAB C30 3.xx
 * Linker:              MPLAB C30 3.xx
 * Company:             Microchip Technology, Inc.
 *
 * Copyright (c) 2012 Microchip Technology Inc. All rights reserved.
 * 
 * Software License Agreement
 *
 * The software supplied herewith by Microchip Technology Incorporated
 * (the “Company”) for its PICmicro® Microcontroller is intended and
 * supplied to you, the Company’s customer, for use solely and
 * exclusively on Microchip PICmicro Microcontroller products. The
 * software is owned by the Company and/or its supplier, and is
 * protected under applicable copyright laws. All rights are reserved.
 * Any use in violation of the foregoing restrictions may subject the
 * user to criminal sanctions under applicable laws, as well as to
 * civil liability for the breach of the terms and conditions of this
 * license.
 *
 * THIS SOFTWARE IS PROVIDED IN AN “AS IS” CONDITION. NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 * TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
********************************************************************
 * Change log
 * Initial Release
 *
 ********************************************************************/

#include "pickit3.h"

#ifdef __DEBUG
    BOOL gblSTAYINTRAP = TRUE;
#else
    #define gblSTAYINTRAP  1
#endif

/* ****************************************************************
* Standard Exception Vector handlers if ALTIVT (INTCON2<15>) = 0  *
*                                                                 *
* Not required for labs but good to always include                *
******************************************************************/
void __attribute__((__interrupt__,__no_auto_psv__)) _OscillatorFail(void)
{

        INTCON1bits.OSCFAIL = 0;
        _HALT();
        while(gblSTAYINTRAP);
}

void __attribute__((__interrupt__,__no_auto_psv__)) _AddressError(void)
{

        INTCON1bits.ADDRERR = 0;
        _HALT();
        while(gblSTAYINTRAP);
}

void __attribute__((__interrupt__,__no_auto_psv__)) _StackError(void)
{

        INTCON1bits.STKERR = 0;
        _HALT();
        while(gblSTAYINTRAP);
}

void __attribute__((__interrupt__,__no_auto_psv__)) _MathError(void)
{

        INTCON1bits.MATHERR = 0;
        _HALT();        
        while(gblSTAYINTRAP);
}

#ifdef __DEBUG
void __attribute__((__interrupt__,__no_auto_psv__)) _DefaultInterrupt(void)
{
    
    _HALT();
    while(gblSTAYINTRAP);
}   
#endif



/* ****************************************************************
* Alternate Exception Vector handlers if ALTIVT (INTCON2<15>) = 1 *
*                                                                 *
* Not required for labs but good to always include                *
******************************************************************/
void __attribute__((__interrupt__,__no_auto_psv__)) _AltOscillatorFail(void)
{

        INTCON1bits.OSCFAIL = 0;
        while(gblSTAYINTRAP);
}

void __attribute__((__interrupt__,__no_auto_psv__)) _AltAddressError(void)
{

        INTCON1bits.ADDRERR = 0;
        while(gblSTAYINTRAP);
}

void __attribute__((__interrupt__,__no_auto_psv__)) _AltStackError(void)
{

        INTCON1bits.STKERR = 0;
        while(gblSTAYINTRAP);
}

void __attribute__((__interrupt__,__no_auto_psv__)) _AltMathError(void)
{

        INTCON1bits.MATHERR = 0;
        while(gblSTAYINTRAP);
}



