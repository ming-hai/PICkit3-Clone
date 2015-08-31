/*********************************************************************
 *
 * Serial SPI header file
 *
 *********************************************************************
 * FileName:            spi_serial_flash.h
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

#ifndef SPI_SERIAL_FLASH
#define SPI_SERIAL_FLASH

#include "commands.h"       // SYS_VER

#define CODE_SEE_ADDR       0xF0
#define PARMS_SEE_ADDR      0xF1

#define CODE_SPACE_SEE_ADDRESS          0x80000000uL

void InitSEEs(void);
bool WriteSerialEEBuffer(DWORD Address, BYTE *Buf, DWORD Size);
bool ReadSerialEEBuffer(DWORD Address, BYTE *Buf, DWORD Size);

#define SIZEOF(s,m)             ((size_t) sizeof(((s *)0)->m)) 

// Use these functions to read and write to the serial eeprom's sections
#define SeeStructRead(M,D)               ReadSerialEEBuffer(offsetof(SEE_ALLOC_TABLE,M), (u8 *)D, SIZEOF(SEE_ALLOC_TABLE,M))
#define SeeStructWrite(M,D)              WriteSerialEEBuffer(offsetof(SEE_ALLOC_TABLE,M), (u8 *)D, SIZEOF(SEE_ALLOC_TABLE,M))

// Various non-volatile parameters such as the serial number are part of this structure
typedef union
{
    BYTE    ExpansionRoom[0x100];
    struct
    {
        char    SerialNumber[14];
        SYS_VER Version;
        char    ToolName[32]; // Unit ID/Friendly Name
    };
}SEE_PARAMETERS;

typedef union _SEE_ALLOC_TABLE
{
    struct _complete
    {
        DWORD   ExpansionRoom[40];
        WORD    Crc;        // incase we add it
    } complete;

    struct _members
    {
        SYS_VER         version;
        SEE_PARAMETERS  Parameters;
    } mbr;

} SEE_ALLOC_TABLE;

#endif   // SPI_SERIAL_FLASH
