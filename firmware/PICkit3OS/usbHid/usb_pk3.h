/*********************************************************************
 *
 * PICkit 3 USB stack header file
 *
 *********************************************************************
 * FileName:            usb_pk3.h
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

#ifndef USB_PK3_DOT_H
#define USB_PK3_DOT_H

//********************************************************************
// 
//********************************************************************

#define USB_BUS_SENSE   1

#define self_power      0


// =================================================
//    U S B   P R O D U C T   I D s   ( P I D s )
// =================================================

// These are the USB Product IDs we have chosen:

#define PK3_PID             0x900A

//********************************************************************
// 
//********************************************************************

//#include "processor.h"
//#include "types.h"
#include "..\include\pickit3.h"
#include "compiler.h"


#include "usb_ch9.h"
#include "usb_config.h"
#include "usb_device.h"
#include "usb_function_hid.h"
#include "usb_hal.h"
#include "usb_hal_pic24.h"
//#include "usb_pkob.h"

//********************************************************************
// 
//********************************************************************

#ifndef HASSERT
#ifdef __DEBUG
#define HASSERT(x) {if(!x){_HALT();}}
#else
#define HASSERT(x)  
#endif
#endif


//********************************************************************
// 
//********************************************************************

byte HIDRxHasData(void);
byte HIDTxIsDone (void);
void HIDGetBlock (byte *buffer, dword size);
void HIDPutBlock (byte *buffer, word size);

BOOL USBDataReady(void);
BYTE *USBGet(BYTE *buffer, DWORD size);
BYTE USBGetChar(void);
bool USBInitialize(void);
bool USBStart(void);
void USBPutCont(BYTE *buffer, DWORD size, bool cont);
void ResetUSBBuffers(void);
void USBFASTPut(BYTE *buffer, DWORD size);
void SetupSSSShipment(bool Start);
void USBStreamSRAM(bool SRAMA, DWORD size);
void USBflush(void);
void USBReset(void);
bool Check4USBLateStart(void *NoArgs);
void USBStreamSRAM(bool SRAMA, DWORD size);
void ResetStreamPort(void);
BOOL USBBUFFERReady(void);
BOOL USBSTREAMBUFFERReady(void);

//********************************************************************
// 
//********************************************************************

#define USBSRAMSPACE        0x00 

#define FIRSTUSBDATAMEMBER  0x0

#define USBPut(b,s)         USBPutCont((b),(s),FALSE)

#define USBPutChar(b)       USBPutCont((&b),1,FALSE)        

#define USBPutAppend(b,s)   USBPutCont((b),(s),FALSE)   

#define USBGetVar(Var)      USBGet((BYTE *)&Var,sizeof(Var));    

#define USBPutVar(Var)      USBPut((BYTE *)&Var,sizeof(Var));   

//#define USB_SERIAL_NUMBER_LENGTH_IN_WORDS   12

// Increased the length for the Tool Name/Unit ID/Friendly Name
#define USB_SERIAL_NUMBER_LENGTH_IN_WORDS   14

#define USB_SERIAL_DEFAULT_VALUES {'M','T','I','0','0','0','0','0','0','0','0','0'}

//********************************************************************
// 
//********************************************************************

// USB_SERIAL_NUMBER_DESCRIPTOR
//
/*
 * USB Serial Number Descriptor
 *
 */
typedef struct _USB_SERIAL_NUMBER_DESCRIPTOR
{
    byte bLength;       
    byte bDscType;      
    word string[USB_SERIAL_NUMBER_LENGTH_IN_WORDS]; 
    
}   USB_SERIAL_NUMBER_DESCRIPTOR;

// USB_RX_INFO
//
/*
 * USB transmitter information
 *
 */
typedef struct _USB_RX_INFO
{
    BYTE waitingForFirstReport; // If we are waiting for a new message
    DWORD total;                // Length as found in the first 2 bytes of frist report in a message
    DWORD readByUser;           // How many bytes of a message have been read by the user
    BYTE available;             // Number of bytes to be read from reportCopy[]
    BYTE offsetIntoCopy;        // Where we are reading from in reportCopy[]
    BYTE rxPending;             //If we the ISR has received another report that needs to be copied
                                // onto reportCopy

    BYTE reportCopyAvailable;   // We can copy into reportCopy because 
                                //  we have process all the data it had.

    BYTE __attribute__ ((aligned (2))) reportCopy[HID_INT_OUT_EP_SIZE]; // where we put the report we are processing
                                                                        //  We align it on a word since we will be accesing the length of the message
                                                                        //  with a word pointer
} USB_RX_INFO;

#ifdef SCRIPTING
#define FIRST_REPORT_LEN_OF_LEN         0
#else
#define FIRST_REPORT_LEN_OF_LEN         4
#endif

#define FIRST_REPORT_AVAILABLE_SPACE    (HID_INT_OUT_EP_SIZE - FIRST_REPORT_LEN_OF_LEN)

#define FIRST_REPORT_LEN_IS_AT_OFFSET   (HID_INT_OUT_EP_SIZE - FIRST_REPORT_LEN_OF_LEN)


/*
 * LANGID
 * USB 'LANGID' (Language ID) code: English (United States)
 */
#define LANGID 0x0409 // English (United States)

/*
 * DESCRIPTOR_TYPE_STRING
 * String descriptor type
 */
#define DESCRIPTOR_TYPE_STRING 0x03 // String

#define SRAM_SIZE               0x600

// USB_TX_INFO
//
/*
 * For tx, we need to buffer up all the data the user puts via 
 * USBPutCont(...,FALSE) until the user calls USBPutCont(...,TRUE) or 
 * calls USBFlush()
 */
typedef struct _USB_TX_INFO
{
    WORD total;                 // how much data to send
    WORD offset;                // where to put the next char when USBPutCont() is called
    volatile BOOL SENDSSS;       
    volatile BOOL BufferRdy;     
    volatile BOOL FastPutInProgress;    // Possible to get back to back interrupts
    volatile BYTE SSSPktsSent;           
    BYTE __attribute__ ((aligned (4))) buffer[SRAM_SIZE+4+4+2]; 
                                // We align it on a dword since we will be accesing the length of the message
                                // with a dword pointer                                
                                // +4 for the length +4 for the normal responses +2 for the checksum
} USB_TX_INFO;

//********************************************************************
// End
//******************************************************************** 

#endif          // USB_PK3_DOT_H
