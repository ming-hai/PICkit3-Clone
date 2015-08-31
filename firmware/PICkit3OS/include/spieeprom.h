/*********************************************************************
 *
 * SPI EEPROM header file
 *
 *********************************************************************
 * FileName:            spieeprom.h
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

#ifndef SPIEEPROM_DOT_H
#define SPIEEPROM_DOT_H

// peripheral configurations
#define SPI_MASTER          0x013B  // select 8-bit master mode, CKE=1, CKP=0   // 8MHZ
#define SPI_MASTER_SLOW     0x013E  // select 8-bit master mode, CKE=1, CKP=0   // 4Mhz
#define SPI_ENABLE          0x8000  // enable SPI port, clear status


/************************************************************************
* EEPROM Commands                                                       *
*                                                                       *
************************************************************************/
#define EEPROM_PAGE_SIZE    (unsigned)64
#define EEPROM_PAGE_MASK    (unsigned)0x003f

// Read Commands
#define EEPROM_CMD_READ     (unsigned)0b00000011    // 03h Read Array
#define EEPROM_CMD_READHF   (unsigned)0b00001011    // 0Bh Read Array High Freq

// Erase Commands
#define EEPROM_CMD_ERASE4K  (unsigned)0b00200000    // 20h Block Erase (4K bytess)
#define EEPROM_CMD_ERASE32K (unsigned)0b01010010    // 52h Block Erase (32K bytes)
#define EEPROM_CMD_ERASE64K (unsigned)0b11011000    // D8h Block Erase (64K bytes)
#define EEPROM_CMD_ERASECHP (unsigned)0b01100000    // 60h Chip Erase

// Program Commands
#define EEPROM_CMD_WRITE    (unsigned)0b00000010    // 02h Byte/Page Program (1 to 256 bytes)
#define EEPROM_CMD_PGMSEQ1  (unsigned)0b10101101    // ADh Sequential Programming Mode (1)
#define EEPROM_CMD_PGMSEQ2  (unsigned)0b10101111    // AFh Sequential Programming Mode (2)

// Protection Commands
#define EEPROM_CMD_WREN     (unsigned)0b00000110    // 06h Write Enable
#define EEPROM_CMD_WRDI     (unsigned)0b00000100    // 04h Write Disable

#define EEPROM_CMD_PS       (unsigned)0b00110110    // 36h Protect Sector
#define EEPROM_CMD_US       (unsigned)0b00111001    // 39h Unprotect Sector
#define EEPROM_CMD_PSREG    (unsigned)0b00111100    // 3Ch Read Sector Protection Registers

// Status Register Commands
#define EEPROM_CMD_RDSR     (unsigned)0b00000101    // 05h Read Status Register
#define EEPROM_CMD_WRSR     (unsigned)0b00000001    // 01h Write Status Register

// Misc Commands
#define EEPROM_CMD_RDDEVID  (unsigned)0b10011111    // 9Fh Read Manufacturer and Device ID
#define EEPROM_CMD_POWERDN  (unsigned)0b10111001    // B9h Deep Power-down
#define EEPROM_CMD_POWERUP  (unsigned)0b10101011    // ABh Resume from Deep Power-down


/************************************************************************
* Aliases for IOs registers related to SPI connected to EEPROM          *
*                                                                       *
************************************************************************/

#define EEPROM_SS_TRIS      tris_UTIL1_nCS          // mrk - updated pins for pk3
#define EEPROM_SS_PORT      pin_UTIL1_nCS
#define EEPROM_SCK_TRIS     tris_UTIL_SDO
#define EEPROM_SDO_TRIS     tris_UTIL_SCK
#define EEPROM_SDI_TRIS     tris_UTIL_SDI


/************************************************************************
* Structure STATREG and union _EEPROMStatus_                            *
*                                                                       *
* Overview: Provide a bits and byte access to EEPROM status value.      *
*                                                                       *
************************************************************************/
struct  STATREG{
    unsigned    WIP:1;          // bit 0 Write-In-Process, Ready/Busy Status
    unsigned    WEL:1;          // bit 1 Write Enable Latch Status
    unsigned    BP0:1;          // bit 2 Block Protection, Software Protection Status
    unsigned    BP1:1;          // bit 3 Block Protection, Software Protection Status
    unsigned    WPP:1;          // bit 4 Write Protect (WP#) Pin Status
    unsigned    EPE:1;          // bit 5 Erase/Program Error
    unsigned    SPM:1;          // bit 6 Sequential Program Mode Status
    unsigned    WPEN:1;         // bit 7 Write Protect Enable / Sector Protection Registers Locked
};

typedef union _EEPROMStatus_{
    struct STATREG          Bits;
    unsigned char           Char;
}EEPROMSTATUS,* pEEPROMSTATUS;

#endif // do_once
