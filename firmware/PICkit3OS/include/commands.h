/*********************************************************************
 *
 * PICkit 3 System structures
 *
 *********************************************************************
 * FileName:            commands.h
 * Dependencies:    
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

#ifndef COMMANDS_DOT_H
#define COMMANDS_DOT_H

// BYTE  - 8bit unsigned data type
// BOOL  - 8bit unsigned data type with value FALSE - 0x00 or TRUE = !FALSE
// WORD  - 16bit unsigned data type
// DWORD - 32bit unsigned data type

//********************************************************************
// Structure types
//********************************************************************

//********************************************************************
// System Version Struct
//
// Read back from the Host whenever its curious as to what exists in 
// space
//
//********************************************************************

typedef struct _SYS_VER
{
    BYTE    TYPE;                 
    BYTE    MAJOR;              
    BYTE    MINOR;              
    BYTE    REV;                
}SYS_VER;

#define FILLVERVAR(a,b) _FILLVERVAR(a,b)
#define _FILLVERVAR(var,a,b,c,d) var.TYPE = a; var.MAJOR = b; var.MINOR = c; var.REV = d
//********************************************************************
// System Voltages Struct
//********************************************************************

typedef struct  _SYSVOLTAGES
{
    WORD        VPP;            // System Voltage: Vpp voltage
    WORD        VDD;            // System Voltage: Vdd voltage
    WORD        VDDTARGET;      // System Voltage: Target Vdd
}SYSVOLTAGES,PSYSVOLTAGES;

//********************************************************************
// Bootloader header Struct
//********************************************************************
typedef struct
{
    WORD            MagicKey;  
    SYS_VER         Version;        
}BL_HEADER, BL_FOOTER;

//********************************************************************
// End
//********************************************************************

#endif      // COMMANDS_DOT_H

