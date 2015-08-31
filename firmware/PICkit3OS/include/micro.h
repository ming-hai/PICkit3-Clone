/*********************************************************************
 *
 * Microcontroller Code Functions Header File
 *
 *********************************************************************
 * FileName:            micro.h
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

#ifndef MICRO_DOT_H
#define MICRO_DOT_H

//********************************************************************
// Defines
//********************************************************************

#define SERIAL_NUM_MAX_SIZE         0x20
#define TOOL_NAME_MAX_SIZE          0x20
#define TIMER_EVENT_PGMWAIT             (VOID_FCN_PVOID_CAST)0x1

#define ROWDATASIZE 192      // Number of bytes per row
//Self-write NVMCON opcodes 
#define PM_PAGE_ERASE       0x4042  //NVM page erase opcode
#define PM_ROW_WRITE        0x4001  //NVM row write opcode
#define PK3_BL_SWITCH_ADDRESS 0x2A000

#define BOOTLOADER_START    0x0F000
#define BOOTLOADER_END      0x29FF0
#define BOOTLOADERKEY       0xA99A
#define MIN_BL_VER          {0x00, 0x01, 0x04, 0x14}
#define PK3_JUMP_TO_BOOT         0xBEEF

/***********************************t**********************************
 * MACROS                                                            *
 *********************************************************************/

// depending on the mode of operation is how long to wait
#define SLEEP(x)        Delayus((long)x*63L)

/*********************************************************************
 * Typdefs
 ********************************************************************/

typedef enum _TIMERMETHOD{
    
    ONESHOT,        
    CONTINUOUS}     
    
    TIMERMETHOD;


//********************************************************************
// External Functions
//********************************************************************

extern void Delayus(dword delay);
extern void SetupTimer(dword useconds, TIMERMETHOD method ,void (*fcn)(void *), void * attribs);
extern void GoTimer(void);
extern void StopTimer(void);
extern void StartPeriodicTimer(void);
extern void SystemReset(void);
extern void RetrieveJAMVer(void);
extern void SetupPushButtonIRQ(void);
extern void SetHaltPBAware(bool state);
extern void SetResetPBAware(bool state);
extern void SetSpecialInternalVDD(bool state);
extern bool LoadSerialNumber(void);
extern bool LoadToolName(void);
extern void SetupPeriodicTimer(void);
extern void WriteAPBootloaderSwitch();
void SwitchToBootloader(void);
bool TestBootloader(void);
bool TestBLFooter(void);
//********************************************************************
// End
//********************************************************************

#endif      // MICRO_DOT_H
