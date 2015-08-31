/*********************************************************************
 *
 * PICkit 3 USB stack
 *
 *********************************************************************
 * FileName:            usb_pk3_hid.c
 * Dependencies:        usb_pk3.h
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

// This file adds to the MCHP HID framework by allowing the send/receive
// of any lenght packet. 

#include "usb_pk3.h"

// Protocol notes: HID wants to send information in reports. We chose a report size of 64 bytes.
// If the user sends 67 bytes, we receive it using two 64 byte reports. Then the upper layer
// code can read the 67 bytes. That's the end of the story if the upper layer code would
// know exactly how many bytes are comming and would ask for that exact number. This is how
// PK2 and the starter kits work. However, thatis not the case for Real ICE/ICD3/PK3. 
// The upper layer SW might read the 67 bytes one byte at a time (or in any possible combinations
// of reads that yields 67). In that case,
// this driver cannot know how much of the two 64 byte reports is data and how much is padding.
// So, after reading the 67 bytes, we receive another report. Now the upper layer asks us for the
// next byte. The next byte is not byte number 3 (counting from 0) in report number 2 (the byte just
// after the last byte of useful information in the previous read). Instead the byte the upper layer
// wants is byte number 0 in report number 3!. To avoid this problem, we create the concept of a message. A message
// is a set of reports that contains all the data the upper layer will actually ask for. The
// upper layer and the software in the PC host must comply with this. In the Real ICE/ICD3/PK3 firmware
// the length of the data is always know or in the case of a variable length transaction, then the command itself
// will contain the length of the data being sent. We could conceivable use that length (by peeking into the
// command structure). However, this will mix this communication layer with the upper one. So we do not
// do this. We simply sacrifice the first four bytes of the first report and put in them the total length
// of the message. This allows us to use this HID API to transport anything and could thus be useful in the future
// for other products (not only the ones that use the Real ICE/ICD3/PK3 command set).
// So for the example above when the PC is sending 67 bytes (0x43 in hex:
//
//     +----------+
//     | 0x43     | byte 0     <- length of data in this message (little endian)
//     | 0x00     | byte 1
//     | 0x00     | byte 2
//     | 0x00     | byte 3
//     | data0    | byte 4     <-- report 1
//     :          :
//     | data61   | byte 63
//     +----------+
//     | data62   | byte 0
//     | data63   |             <-- report 2
//     | data64   |             
//     | data65   |             
//     | data66   |             
//     | padding  |             
//     | padding  |             
//     :          :
//     | padding  | byte 63
//     +----------+
//
// NOTE: This is true when the PC sends and we receive. When we send and the PC receives, we never,
// ever send more than the size of the SRAM. So, in that case, the header length is only 16 bits.

// Define so we can link to usbHid/test/main.c and be able to implement (in main.c) a ping pong test
// Whenever we see this define in this file we do not attempt to call functions that are part of ../ (RS)
//#define USB_HID_UNIT_TEST   1

void ControlUSBPower(BOOL on);
void USBDeviceTasks(void);
void USBSetSerialNumberDescriptorInRam(void);
void USBSetSerialNumber(word *sn);
word HIDRxGetCount(void);
void USBHandleOneRxReport(void);

extern USB_VOLATILE BYTE USTATcopy;

#ifndef USB_HID_UNIT_TEST    
    extern BYTE gblMainSer[];
#endif

USB_RX_INFO gRx;
USB_TX_INFO gTx;

USB_HANDLE USBRxHandle = NULL;  // for OUT (w/r/t host) endpoint
USB_HANDLE USBTxHandle = NULL;  // for IN  (w/r/t host) endpoint

// Used to set the USB timer to the correct state
#define USBTimerON(x)           T1CONbits.TON = x
#define SSS_THRESHOLD_COUNT     10
#define SET_TMR1_DELAY(x)       PR1 = 0.001*(__FCY/mUL(256))*x

// Windows 2000 has a problem with recieving data too quickly
extern bool     gblWINDOWS_2000_HACK;

//********************************************************************
// Functions
//********************************************************************

// Functions expected by the upper level Real ICE/ICD3/PICkit3 software
// These functions are thin shells that simply adjust calling conventions
// and use the primitives HIDXXXX() functions.
// The comments in the function headers are the comments from the 
// same functions in the PLX driver used by Real ICE and ICD3.
//

// USBDataReady
// 
/*
 * Returns true if there is data ready to be read
 * 
 *
 */
BOOL USBDataReady(void)
{
    return HIDRxGetCount() ? 1: 0;
}


// USBGet
// 
/*
 * Get a stream of length size from the USB port and place it 
 * into the buffer
 * if SCRIPTING is defined, it can only be used to get one report
 * 
 * return BYTE RETURNS the end of the buffer+1
 *
 */
BYTE *USBGet(BYTE *buffer, DWORD size)
{
    HIDGetBlock(buffer, size);
    return buffer+(word)size;
}


// USBGetChar
// 
BYTE USBGetChar(void)
{
    BYTE res;
    HIDGetBlock(&res, sizeof(res));
    return res;
}


// USBInitRxVars
// 
void USBInitRxVars(void)
{
    gRx.waitingForFirstReport = TRUE;
    gRx.total                 = 0;
    gRx.available             = 0;
    gRx.offsetIntoCopy        = 0;
    gRx.rxPending             = FALSE;
    gRx.reportCopyAvailable   = TRUE;
    gRx.readByUser            = 0;
}


// USBInitTxVars
// 
void USBInitTxVars(void)
{
    gTx.total = 0;
    gTx.offset = FIRST_REPORT_LEN_OF_LEN; // skip the length
}


// USBInitialize
// 
/*
 * Gets the ports, interrupts, DMA, etc ready to be startup. 
 * Must call startup to make it enumerate.
 * 
 */
bool USBInitialize(void)
{
    // Initialize reception variables
    memset((BYTE *)&gTx,0x0,sizeof(gTx));
    
    USBInitRxVars();
    USBInitTxVars();

#ifdef USB_HID_UNIT_TEST    
    USBSetSerialNumberDescriptorInRam();    // Copy the one from ROM
#else
    WORD serialNumberInWords[USB_SERIAL_NUMBER_LENGTH_IN_WORDS];   
    int i;
    extern BYTE gblMainSer[];
    extern BYTE gblToolName[];
    
    // Assume that gblMainSer contains the serial number
    for (i = 0 ; i < USB_SERIAL_NUMBER_LENGTH_IN_WORDS; i++)
    {
        
        // Form the UNICODE string (serialNumberInWords[]) from the ASCIIZ string (gblMainSer[])
        // (USB enumeration needs the string in UNICODE format.)
		if(gblToolName[0] == 0x23)
	       	serialNumberInWords[i] = gblToolName[i+1];
		else
        	serialNumberInWords[i] = gblMainSer[i];
    }    
    
    USBSetSerialNumberDescriptorInRam();        // Copy the one from ROM
    USBSetSerialNumber(serialNumberInWords);    // Override the serial number section only
                                                // with what is stored in gblMainSer
#endif
    USBDeviceInit();
    
    // Set the buffer ready
    gTx.BufferRdy=1;
    
    return TRUE;
}


// USBStartTimedOut
// 
void USBStartTimedOut(void *p)
{
    *(BYTE *)p = TRUE;
}


// USBStart
// 
/*
 * Actually initializes all the USB endpoints and makes it 
 * enumerate to the Host returns false if it fails 
 * 
 */
bool USBStart(void)
{
    BYTE done    = FALSE;
    BYTE expired = FALSE;
    bool res     = FALSE; // assume we will fail

    // Wait up to 3 seconds, it really does not matter anymore since we now 
    // are interrupt based so it can reconnect anytime. The only thing wrong is it
    // does not request the 500ma before it grabs it
    SetupTimer(3000000, ONESHOT, USBStartTimedOut, &expired);
    GoTimer();  // and fire!
    
    while(!done)
    {
        // USBDeviceTasks is usually called in the interrupt.
        // Here we have not started the interrupt yet. So, simply
        // poll it.
        USBDeviceTasks();

        // After 10 seconds now we continue on anyway if the pgm2go is active or its a PKOB based system
        if ((USBDeviceState == CONFIGURED_STATE) || (expired ))
        {
            done = TRUE;
            res  = TRUE;
            
            // Now that we are configure, we can turn on
            // the power to the rest of the system (we have been
            // granted 500ma by the host).
            #ifndef USB_HID_UNIT_TEST
                ControlUSBPower(TRUE);
            #endif
        }
    }
    if (res)
    {
        // After this we handle the USBDeviceTasks at interrupt time
        _USB1IE = 0;
        _USB1IF = 0;
        _USB1IP = 6; // same as in Real ICE and ICD3
        _USB1IE = 1;
    }
    return TRUE;
}


// USBPutCont
// 
/*
 * When cont is TRUE, we flush the tx buffer after copying the data.
 * 
 */
void USBPutCont(BYTE *buffer, DWORD size, bool cont)
{
    //HASSERT(((WORD)size + gTx.total) < SRAM_SIZE);
    // We now need to send a response   
    
    // Since it is interrupted based and using the transmit buffer we can over run one in progress
    // wait until we are ready
    while(!USBBUFFERReady());    
    
    // should we transmit out of RAM or sudo SRAM
    if(buffer == USBSRAMSPACE)
    {
        memcpy(&gTx.buffer[gTx.offset], *gblCurIntRamLoc, (WORD)size);
        *gblCurIntRamLoc += size; // now point to the next location
    }   
    else
        memcpy(&gTx.buffer[gTx.offset], buffer, (WORD)size);
    gTx.offset += (WORD)size;
    gTx.total  += (WORD)size;
    if (cont)
        USBflush();
}


// ResetUSBBuffers
// 
/*
 * Reset USB Buffers
 * 
 */
void ResetUSBBuffers(void)
{
    // TODO: Do we need to really do something here?
}


// USBFASTPut
// 
/*
 * When cont is TRUE, we flush the tx buffer after copying the data.
 * NOTE: SHOULD ONLY be called within the interrupt context
 * 
 */
void USBFASTPut(BYTE *buffer, DWORD size)
{   
    WORD i;

    // Copy the data 
    memcpy(&gTx.buffer[gTx.offset], buffer, (WORD)size);
    gTx.total += size;
    
    // First report contains length of message at the end of the first report
    // Move the data from gTx.buffer[4]->gTx.buffer[64] to gTx.buffer[0]->gTx.buffer[59]
    for (i = 0; i < FIRST_REPORT_AVAILABLE_SPACE; ++i)
        gTx.buffer[i] = gTx.buffer[i+FIRST_REPORT_LEN_OF_LEN];
    
#ifndef SCRIPTING
    *((DWORD *)&gTx.buffer[FIRST_REPORT_LEN_IS_AT_OFFSET]) = gTx.total;
#endif

    gTx.total += FIRST_REPORT_LEN_OF_LEN; // to include the length of the message stored in the last 4 bytes

    // move the pointers
    gTx.offset += (WORD)size;
    gTx.total  += (WORD)size;

    // Ship it      
    USBTxHandle = HIDTxPacket(HID_EP,gTx.buffer,HID_INT_IN_EP_SIZE);
    
    USBInitTxVars();   // reset the place holders
}

extern BYTE MARKTEMP;


// USBflush
// 
/*
 * Transmit whatever is in gTx.buffer
 * 
 */
void USBflush(void)
{
    byte *sendFromHere = gTx.buffer;
    byte howMuchToCopy;
    word i;

    // Make sure we have data in the buffer before we send it
    if (gTx.total!=0)
    {     
        // If we are windows 2000 give the PC some time to gobble up the previous packet, this may be too short for slower machines  
        if(gblWINDOWS_2000_HACK)
            Delayus(5000);

        // First report contains length of message at the end of the first report
        // Move the data from gTx.buffer[4]->gTx.buffer[64] to gTx.buffer[0]->gTx.buffer[59]
        for (i = 0; i < FIRST_REPORT_AVAILABLE_SPACE; ++i)
            gTx.buffer[i] = gTx.buffer[i+FIRST_REPORT_LEN_OF_LEN];
        
#ifndef SCRIPTING
        *((DWORD *)&gTx.buffer[FIRST_REPORT_LEN_IS_AT_OFFSET]) = gTx.total;
#endif
        gTx.total += FIRST_REPORT_LEN_OF_LEN; // to include the length of the message stored in the last 4 bytes
    
        while(gTx.total)
        {           
            // Wait for last report to be out only if have sent at least one
            if (USBTxHandle)                        
            {                               // the first time we call this function we do not           
                while(!HIDTxIsDone())       // not need to wait. Otherwise wait for
                    ;                       // the last response to be done
            }   
     
            if (gTx.total > HID_INT_IN_EP_SIZE)
                howMuchToCopy = HID_INT_IN_EP_SIZE;
            else
                howMuchToCopy = gTx.total;
    
            gTx.total -= howMuchToCopy;
            
            // Ship it      
            USBTxHandle = HIDTxPacket(HID_EP,sendFromHere,HID_INT_IN_EP_SIZE);
            sendFromHere += howMuchToCopy;
            
            // Since now interrupt based, we need this flag
            gTx.BufferRdy = FALSE;
        }
        
        USBInitTxVars();  
    }    
}


// USBReset
// 
void USBReset(void)
{
    // Wait for all data to be out or in, then wait for 100 msecs
    // and finally disable the USB peripheral.
    while( HIDRxHandleBusy(USBRxHandle) )       // wait for any rx to be done
        ;
    while( HIDTxHandleBusy(USBTxHandle) )       // wait for any tx to be done
        ;
    #ifndef USB_HID_UNIT_TEST    
        Delayus(100000);
    #endif

    // Disable module & detach from bus
    U1CON = 0;             

    // Mask all USB interrupts              
    U1IE = 0;          

    //Move to the detached state                  
    USBDeviceState = DETACHED_STATE;
}


// Check4USBLateStart
// 
bool Check4USBLateStart(void *NoArgs)
{
    return USBDeviceState == CONFIGURED_STATE;
}


// USBStreamSRAM
// 
/*
 * No streaming supported in PK3
 * 
 */
void USBStreamSRAM(bool SRAMA, DWORD size) 
{
    HASSERT(0);
}
void ResetStreamPort(void) 
{
    // Just use this as a prototype
    //HASSERT(0);
}


// USBBUFFERReady
// 
BOOL USBBUFFERReady(void)   
{ 
    return(gTx.BufferRdy);
}


// USBSTREAMBUFFERReady
// 
BOOL USBSTREAMBUFFERReady(void)
{
    HASSERT(0);
    return TRUE;
}


// HIDRxHasData
// 
/*
 * Primitives to receive/transmit
 * 
 */
byte HIDRxHasData(void)
{
    byte res;

    DISABLE_SYSTEM_INTERRUPT
    res = gRx.available ? TRUE : FALSE;
    RESTORE_SYSTEM_INTERRUPT 

    return res;
}   


// HIDTxIsDone
// 
byte HIDTxIsDone(void)
{
    return !HIDTxHandleBusy(USBTxHandle);
}   


// HIDRxGetCount
// 
/*
 * Return how many bytes are available
 * 
 */
word HIDRxGetCount(void)
{
    word res;
    DISABLE_SYSTEM_INTERRUPT
    res = gRx.available;
    RESTORE_SYSTEM_INTERRUPT
    return res;
}

 
// HIDGetBlock
// 
/*
 * Read from gRx.reportCopy[] as data becomes available
 * 
 */
void HIDGetBlock(byte *buffer, dword size)
{
    byte done;
    byte available;
    dword read = 0;

    for (done = FALSE; !done ; )
    {
        available = HIDRxGetCount();
        if (available)
        {
            DISABLE_SYSTEM_INTERRUPT
            if (available + read >= size)
            {
                // We can finish this call
                if(buffer == USBSRAMSPACE)
                {
                    memcpy(*gblCurIntRamLoc, &gRx.reportCopy[gRx.offsetIntoCopy], size-read);
                    *gblCurIntRamLoc += (size-read); // now point to the next location
                }   
                else
                    memcpy(buffer, &gRx.reportCopy[gRx.offsetIntoCopy], size-read);
                    
                gRx.offsetIntoCopy += (size-read);
                gRx.available      -= (size-read);
                gRx.readByUser     += (size-read);
                done = TRUE;
            } 
            else
            {
                // Copy as much as we have available in gRx.reportCopy[]
                if(buffer == USBSRAMSPACE)
                {
                    memcpy(*gblCurIntRamLoc, &gRx.reportCopy[gRx.offsetIntoCopy], available);
                    *gblCurIntRamLoc += available; // now point to the next location
                }   
                else                
                {
                    memcpy(buffer, &gRx.reportCopy[gRx.offsetIntoCopy], available);
                    buffer             += available;
                }           
                    
                gRx.offsetIntoCopy += available;
                gRx.available      -= available;
                gRx.readByUser     += available;
                read               += available;
            }
            // determine if we have read all data from reportCopy
            if (gRx.total == gRx.readByUser)
            {
                // we are done with message!
                gRx.total                 = 0;
                gRx.available             = 0;
                gRx.offsetIntoCopy        = 0;
                gRx.readByUser            = 0;
                gRx.reportCopyAvailable   = TRUE;
                USBRxHandle = HIDRxPacket(HID_EP, gRx.reportCopy, sizeof(gRx.reportCopy));

            }
            else
            {
                // We know if we are done with this report when offsetIntoCopy
                // is the size of the repor
                if (gRx.offsetIntoCopy == sizeof(gRx.reportCopy))
                {
                    gRx.offsetIntoCopy = 0;
                    gRx.available      = 0;
                    gRx.reportCopyAvailable = TRUE;
                    USBRxHandle = HIDRxPacket(HID_EP, gRx.reportCopy, sizeof(gRx.reportCopy));
                }
            }
            RESTORE_SYSTEM_INTERRUPT
        }
        DISABLE_SYSTEM_INTERRUPT
        if (gRx.rxPending && gRx.reportCopyAvailable)
            USBHandleOneRxReport();
        RESTORE_SYSTEM_INTERRUPT
    }
}


// HIDTxEmptyCount
// 
/*
 * Return how many bytes can be still put in gTx.buffer. 
 * In other words how much space is available in the transmission buffer.
 * 
 */
word HIDTxEmptyCount(void)
{
    word res=0;
    DISABLE_SYSTEM_INTERRUPT
    RESTORE_SYSTEM_INTERRUPT
    return res;
}


// USBHandleOneRxReport
// 
/*
 * This function assumes a report is available to be read
 * 
 */
void USBHandleOneRxReport(void)
{
    WORD i;
    // After that call, the USB driver can receive more data on this endpoint. So it does
    // not have to stall.
    if (gRx.waitingForFirstReport)
    {
        // The last four bytes contain the total length of this message
        // for scripting: length is not stored at the end of the packet
#ifdef SCRIPTING
        gRx.total = 64;
#else
        gRx.total = *( (DWORD *)&gRx.reportCopy[FIRST_REPORT_LEN_IS_AT_OFFSET] );
#endif

        // The original code expected the length at the beginning (not at the end) of the
        // first report in a message. Fool the code by moving the data around. We already saved
        // the last for bytes (the len) so we can safely overwrite them.
        i = FIRST_REPORT_LEN_IS_AT_OFFSET;
        while(i--)
            gRx.reportCopy[i+FIRST_REPORT_LEN_OF_LEN] = gRx.reportCopy[i];
            
        gRx.offsetIntoCopy        = FIRST_REPORT_LEN_OF_LEN;  // skip the length
        if (gRx.total > FIRST_REPORT_AVAILABLE_SPACE)
        {
            // This is the first report of a multilpe-report message 
            gRx.available = FIRST_REPORT_AVAILABLE_SPACE;
            gRx.waitingForFirstReport = FALSE;
        }
        else
        {
            // This message fits in a single report
            gRx.available = gRx.total;
            gRx.waitingForFirstReport = TRUE;
        }
    }
    else
    {
        gRx.offsetIntoCopy        = 0;
        // See if this is the last report in the message
        if (gRx.readByUser + sizeof(gRx.reportCopy) >= gRx.total)
        {
            // Last report in this message
            gRx.available = gRx.total - gRx.readByUser;
            gRx.waitingForFirstReport = TRUE;
        } 
        else
        {
            // Just another report in this message. Not the last one
            gRx.available = sizeof(gRx.reportCopy);
            gRx.waitingForFirstReport = FALSE;
        }
        
    }
}

extern bool isCheckRunTimeCmd(void);


// USBEPService
// 
void USBEPService(void)
{
    //If the last packet was a EP1 IN packet
    if((USTATcopy & USTAT_EP1_PP_MASK) == USTAT_EP1_IN)
    {
        // We just transmitted a report. See if there is more data to be transmitted
        
        gTx.FastPutInProgress=FALSE;    // Done sending the fast put, even if that is not what we just sent
                        
        // Otherwise just tell the system the buffer is ready
        if(!gTx.SENDSSS)    
            gTx.BufferRdy=TRUE; // now make the buffer ready
    }
    else if((USTATcopy & USTAT_EP1_PP_MASK) == USTAT_EP1_OUT_EVEN)
    {
        // We just received a report
        if (gRx.reportCopyAvailable)
        {
            USBHandleOneRxReport();            
        }    
        else
            // Remember we have to deal with this later on
            // The USB driver will stall the endpoint so no more data can come through it
            // until we get around to servicing the data in USBGet()
            gRx.rxPending             = TRUE;
    }
}

const BYTE u_block[] = {0x0};

void __attribute__((__interrupt__,__no_auto_psv__)) _T1Interrupt(void)
{
    BLEEP_ON_INTS
    DISABLE_SYSTEM_INTERRUPT
    STORE_OS_PSV(u_block)
        
    _T1IF = 0;
        
    RESTORE_OS_PSV
    RESTORE_SYSTEM_INTERRUPT
    EBLEEP_ON_INTS
}    
