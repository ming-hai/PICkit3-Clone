/*********************************************************************
 *
 * Main Header File
 *
 *********************************************************************
 * FileName:            pickit3.h
 * Dependencies:        please check the INCLUDES section
 * Processor:           PIC24
 * Assembler/Compiler:  MPLAB XC16 1.00
 * Linker:              MPLAB XC16 1.00
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

#ifndef PICKIT3_DOT_H
#define PICKIT3_DOT_H

//********************************************************************
// INCLUDES
//********************************************************************
#include "processor.h"

#include <stdlib.h>
#include <stddef.h>
#include <string.h>

#include "types.h"
#include "micro.h"

#include "powerPK3.h"
#include "usb_pk3.h"
#include "spi_serial_flash.h"

#include "commands.h"
#include "system.h"

//********************************************************************
// Defines
//********************************************************************
// If BLEEP_ON_INTS is defined, sets DEBUG_PIN inside the interrupt
//#define DO_BLEEP_ON_INTS      1
#undef  DO_BLEEP_ON_INTS

#ifdef DO_BLEEP_ON_INTS
    #define BLEEP_ON_INTS       pin_DEBUG2 = 1;
    #define EBLEEP_ON_INTS      pin_DEBUG2 = 0;
#else
    #define BLEEP_ON_INTS
    #define EBLEEP_ON_INTS
#endif

#define OS_MAGICKEY         0x1A50
#define OS_VER              0x00,0x02,0x00,0x05

#define INTERNAL_SRAM_BUFFER_SIZE       0x800

#define SCRIPTING

#define STORE_OS_PSV(v)     \
        int saved_PSVPAG; \
        saved_PSVPAG = PSVPAG; \
        PSVPAG = __builtin_psvpage(v);

#define RESTORE_OS_PSV      {PSVPAG = saved_PSVPAG;}

//********************************************************************
// Types
//********************************************************************

typedef struct
{
    WORD    MagicKey;
    SYS_VER  Version;
    WORD    Format;
}OS_HEADER;

typedef struct
{
    WORD            SetVDDVoltage;  // What VDD is supposed to be  set to (incase we implement this)
    WORD            SetVPPVoltage;  // What VPP is supposed to be  set to
    WORD            MaxVDDVoltage;  // Maximum VDD voltage ever
}typSetVoltages;

//********************************************************************
// Externs
//********************************************************************

extern SYS_VER gblSysVersion;

#endif      // PICKIT3_DOT_H
