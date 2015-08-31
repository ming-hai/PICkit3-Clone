/*********************************************************************
 *
 * PICkit 3 USB stack header file
 *
 *********************************************************************
 * FileName:            usb_device.h
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

#ifndef USBDEVICE_H
#define USBDEVICE_H
//DOM-IGNORE-END


#include "usb_ch9.h"
#include "usb_hal.h"
#include "usb_config.h" //This file needs to be included after the 
                        //  usb_hal.h file to insure that the user
                        //  options are selected after the defines
                        //  are created.
/** DEFINITIONS ****************************************************/

/********************************************************************
USB Endpoint Definitions
USB Standard EP Address Format: DIR:X:X:X:EP3:EP2:EP1:EP0
This is used in the descriptors. See usbcfg.c

NOTE: Do not use these values for checking against USTAT.
To check against USTAT, use values defined in usbd.h.
********************************************************************/
#define _EP_IN      0x80
#define _EP_OUT     0x00
#define _EP01_OUT   0x01
#define _EP01_IN    0x81
#define _EP02_OUT   0x02
#define _EP02_IN    0x82
#define _EP03_OUT   0x03
#define _EP03_IN    0x83
#define _EP04_OUT   0x04
#define _EP04_IN    0x84
#define _EP05_OUT   0x05
#define _EP05_IN    0x85
#define _EP06_OUT   0x06
#define _EP06_IN    0x86
#define _EP07_OUT   0x07
#define _EP07_IN    0x87
#define _EP08_OUT   0x08
#define _EP08_IN    0x88
#define _EP09_OUT   0x09
#define _EP09_IN    0x89
#define _EP10_OUT   0x0A
#define _EP10_IN    0x8A
#define _EP11_OUT   0x0B
#define _EP11_IN    0x8B
#define _EP12_OUT   0x0C
#define _EP12_IN    0x8C
#define _EP13_OUT   0x0D
#define _EP13_IN    0x8D
#define _EP14_OUT   0x0E
#define _EP14_IN    0x8E
#define _EP15_OUT   0x0F
#define _EP15_IN    0x8F

/* Configuration Attributes */
#define _DEFAULT    (0x01<<7)       //Default Value (Bit 7 is set)
#define _SELF       (0x01<<6)       //Self-powered (Supports if set)
#define _RWU        (0x01<<5)       //Remote Wakeup (Supports if set)

/* Endpoint Transfer Type */
#define _CTRL       0x00            //Control Transfer
#define _ISO        0x01            //Isochronous Transfer
#define _BULK       0x02            //Bulk Transfer
#define _INT        0x03            //Interrupt Transfer

/* Isochronous Endpoint Synchronization Type */
#define _NS         (0x00<<2)       //No Synchronization
#define _AS         (0x01<<2)       //Asynchronous
#define _AD         (0x02<<2)       //Adaptive
#define _SY         (0x03<<2)       //Synchronous

/* Isochronous Endpoint Usage Type */
#define _DE         (0x00<<4)       //Data endpoint
#define _FE         (0x01<<4)       //Feedback endpoint
#define _IE         (0x02<<4)       //Implicit feedback Data endpoint

#define _ROM        USB_INPIPES_ROM
#define _RAM        USB_INPIPES_RAM

//These are the directional indicators used for the USBTransferOnePacket()
//  function.
#define OUT_FROM_HOST 0
#define IN_TO_HOST 1

/********************************************************************
 * CTRL_TRF_SETUP: Every setup packet has 8 bytes.  This structure
 * allows direct access to the various members of the control
 * transfer.
 *******************************************************************/
typedef union _CTRL_TRF_SETUP
{
    /** Standard Device Requests ***********************************/
    struct
    {
        BYTE bmRequestType; //from table 9-2 of USB2.0 spec
        BYTE bRequest; //from table 9-2 of USB2.0 spec
        WORD wValue; //from table 9-2 of USB2.0 spec
        WORD wIndex; //from table 9-2 of USB2.0 spec
        WORD wLength; //from table 9-2 of USB2.0 spec
    };
    struct
    {
        unsigned :8;
        unsigned :8;
        WORD_VAL W_Value; //from table 9-2 of USB2.0 spec, allows byte/bitwise access
        WORD_VAL W_Index; //from table 9-2 of USB2.0 spec, allows byte/bitwise access
        WORD_VAL W_Length; //from table 9-2 of USB2.0 spec, allows byte/bitwise access
    };
    struct
    {
        unsigned Recipient:5;   //Device,Interface,Endpoint,Other
        unsigned RequestType:2; //Standard,Class,Vendor,Reserved
        unsigned DataDir:1;     //Host-to-device,Device-to-host
        unsigned :8;
        BYTE bFeature;          //DEVICE_REMOTE_WAKEUP,ENDPOINT_HALT
        unsigned :8;
        unsigned :8;
        unsigned :8;
        unsigned :8;
        unsigned :8;
    };
    struct
    {
        unsigned :8;
        unsigned :8;
        BYTE bDscIndex;         //For Configuration and String DSC Only
        BYTE bDescriptorType;          //Device,Configuration,String
        WORD wLangID;           //Language ID
        unsigned :8;
        unsigned :8;
    };
    struct
    {
        unsigned :8;
        unsigned :8;
        BYTE_VAL bDevADR;       //Device Address 0-127
        BYTE bDevADRH;          //Must equal zero
        unsigned :8;
        unsigned :8;
        unsigned :8;
        unsigned :8;
    };
    struct
    {
        unsigned :8;
        unsigned :8;
        BYTE bConfigurationValue;         //Configuration Value 0-255
        BYTE bCfgRSD;           //Must equal zero (Reserved)
        unsigned :8;
        unsigned :8;
        unsigned :8;
        unsigned :8;
    };
    struct
    {
        unsigned :8;
        unsigned :8;
        BYTE bAltID;            //Alternate Setting Value 0-255
        BYTE bAltID_H;          //Must equal zero
        BYTE bIntfID;           //Interface Number Value 0-255
        BYTE bIntfID_H;         //Must equal zero
        unsigned :8;
        unsigned :8;
    };
    struct
    {
        unsigned :8;
        unsigned :8;
        unsigned :8;
        unsigned :8;
        BYTE bEPID;             //Endpoint ID (Number & Direction)
        BYTE bEPID_H;           //Must equal zero
        unsigned :8;
        unsigned :8;
    };
    struct
    {
        unsigned :8;
        unsigned :8;
        unsigned :8;
        unsigned :8;
        unsigned EPNum:4;       //Endpoint Number 0-15
        unsigned :3;
        unsigned EPDir:1;       //Endpoint Direction: 0-OUT, 1-IN
        unsigned :8;
        unsigned :8;
        unsigned :8;
    };

    /** End: Standard Device Requests ******************************/

} CTRL_TRF_SETUP;

// Defintion of the PIPE structure
//  This structure is used to keep track of data that is sent out
//  of the stack automatically.
typedef struct
{
    union
    {
        //Various options of pointers that are available to
        // get the data from
        BYTE *bRam;
        ROM BYTE *bRom;
        WORD *wRam;
        ROM WORD *wRom;
    }pSrc;
    union
    {
        struct
        {
            //is this transfer from RAM or ROM?
            BYTE ctrl_trf_mem          :1;
            BYTE reserved              :5;
            //include a zero length packet after
            //data is done if data_size%ep_size = 0?
            BYTE includeZero           :1;
            //is this PIPE currently in use
            BYTE busy                  :1;
        }bits;
        BYTE Val;
    }info;
    WORD_VAL wCount;
}IN_PIPE;

#define CTRL_TRF_RETURN void
#define CTRL_TRF_PARAMS void
typedef struct
{
    union
    {
        //Various options of pointers that are available to
        // get the data from
        BYTE *bRam;
        WORD *wRam;
    }pDst;
    union
    {
        struct
        {
            BYTE reserved              :7;
            //is this PIPE currently in use
            BYTE busy                  :1;
        }bits;
        BYTE Val;
    }info;
    WORD_VAL wCount;
    CTRL_TRF_RETURN (*pFunc)(CTRL_TRF_PARAMS);
}OUT_PIPE;

//Various options for setting the PIPES
#define USB_INPIPES_ROM            0x00     //Data comes from RAM
#define USB_INPIPES_RAM            0x01     //Data comes from ROM
#define USB_INPIPES_BUSY           0x80     //The PIPE is busy
#define USB_INPIPES_INCLUDE_ZERO   0x40     //include a trailing zero packet
#define USB_INPIPES_NO_DATA        0x00     //no data to send
#define USB_INPIPES_NO_OPTIONS     0x00     //no options set

#define USB_EP0_ROM            USB_INPIPES_ROM
#define USB_EP0_RAM            USB_INPIPES_RAM
#define USB_EP0_BUSY           USB_INPIPES_BUSY
#define USB_EP0_INCLUDE_ZERO   USB_INPIPES_INCLUDE_ZERO
#define USB_EP0_NO_DATA        USB_INPIPES_NO_DATA
#define USB_EP0_NO_OPTIONS     USB_INPIPES_NO_OPTIONS

/********************************************************************
 * Standard Request Codes
 * USB 2.0 Spec Ref Table 9-4
 *******************************************************************/
#define GET_STATUS  0
#define CLR_FEATURE 1
#define SET_FEATURE 3
#define SET_ADR     5
#define GET_DSC     6
#define SET_DSC     7
#define GET_CFG     8
#define SET_CFG     9
#define GET_INTF    10
#define SET_INTF    11
#define SYNCH_FRAME 12

/* Standard Feature Selectors */
#define DEVICE_REMOTE_WAKEUP    0x01
#define ENDPOINT_HALT           0x00

/* USB Device States - To be used with [BYTE usb_device_state] */
#define DETACHED_STATE          0x00
#define ATTACHED_STATE          0x01
#define POWERED_STATE           0x02
#define DEFAULT_STATE           0x04
#define ADR_PENDING_STATE       0x08
#define ADDRESS_STATE           0x10
#define CONFIGURED_STATE        0x20

/* UCFG Initialization Parameters */
#if defined(__18CXX)
    #if defined(UCFG_VAL)
        //This has been depricated in v2.2 - it will be removed in future releases
        #define SetConfigurationOptions() {U1CNFG1 = UCFG_VAL;} //UCFG_VAL defined in usb_config.h
    #else
        #define SetConfigurationOptions()   {\
                                                U1CNFG1 = USB_PULLUP_OPTION | USB_TRANSCEIVER_OPTION | USB_SPEED_OPTION | USB_PING_PONG_MODE;\
                                            }  
    #endif      

#elif defined(__C30__)
    
    //TODO: need to use the PULLUP_ENABLE definition and enable OTG mode with all resistors disabled

    #if defined(UCFG_VAL)
        //This has been depricated in v2.2 - it will be removed in future releases
        #define SetConfigurationOptions() {U1CNFG1 = UCFG_VAL;} //UCFG_VAL defined in usb_config.h
    #else
        #define SetConfigurationOptions()   {\
                                                U1CNFG1 = USB_PING_PONG_MODE;\
                                                U1CNFG2 = USB_TRANSCEIVER_OPTION | USB_SPEED_OPTION | USB_PULLUP_OPTION;\
                                            } 
    #endif

//    #if (USB_SPEED_OPTION ==  USB_LOW_SPEED)
//        #error "Low speed operation in device mode is not currently supported in the PIC24F family devices."
//    #endif
#endif
//#define _UTEYE      0x80            // Use Eye-Pattern test

/* UEPn Initialization Parameters */
#if defined (__18CXX)
    #define EP_CTRL     0x06            // Cfg Control pipe for this ep
    #define EP_OUT      0x0C            // Cfg OUT only pipe for this ep
    #define EP_IN       0x0A            // Cfg IN only pipe for this ep
    #define EP_OUT_IN   0x0E            // Cfg both OUT & IN pipes for this ep
                                    // Handshake should be disable for isoch

    #define USB_HANDSHAKE_ENABLED   0x10
    #define USB_HANDSHAKE_DISABLED  0x00

    #define USB_OUT_ENABLED         0x04
    #define USB_OUT_DISABLED        0x00

    #define USB_IN_ENABLED          0x02
    #define USB_IN_DISABLED         0x00

    #define USB_ALLOW_SETUP         0x00
    #define USB_DISALLOW_SETUP      0x08

    #define USB_STALL_ENDPOINT      0x01
#elif defined(__C30__)
    #define EP_CTRL     0x0C            // Cfg Control pipe for this ep
    #define EP_OUT      0x18            // Cfg OUT only pipe for this ep
    #define EP_IN       0x14            // Cfg IN only pipe for this ep
    #define EP_OUT_IN   0x1C            // Cfg both OUT & IN pipes for this ep
    #define HSHK_EN     0x01            // Enable handshake packet
                                    // Handshake should be disable for isoch

    #define USB_HANDSHAKE_ENABLED   0x01
    #define USB_HANDSHAKE_DISABLED  0x00

    #define USB_OUT_ENABLED         0x08
    #define USB_OUT_DISABLED        0x00

    #define USB_IN_ENABLED          0x04
    #define USB_IN_DISABLED         0x00

    #define USB_ALLOW_SETUP         0x00
    #define USB_DISALLOW_SETUP      0x10

    #define USB_STALL_ENDPOINT      0x02
#endif

#define USB_HANDLE volatile BDT_ENTRY*

#if !defined(USBDEVICE_C)
/** EXTERNS ********************************************************/
    //Definitions for the BDT
    #if (USB_PING_PONG_MODE == USB_PING_PONG__NO_PING_PONG)
        extern volatile BDT_ENTRY BDT[(USB_MAX_EP_NUMBER + 1) * 2];
    #elif (USB_PING_PONG_MODE == USB_PING_PONG__EP0_OUT_ONLY)
        extern volatile BDT_ENTRY BDT[((USB_MAX_EP_NUMBER+1) * 2)+1];
    #elif (USB_PING_PONG_MODE == USB_PING_PONG__FULL_PING_PONG)
        extern volatile BDT_ENTRY BDT[(USB_MAX_EP_NUMBER + 1) * 4];
    #elif (USB_PING_PONG_MODE == USB_PING_PONG__ALL_BUT_EP0)
        extern volatile BDT_ENTRY BDT[((USB_MAX_EP_NUMBER + 1) * 4)-2];
    #else
        #error "No ping pong mode defined."
    #endif

    //Depricated in v2.2 - will be removed in a future revision
    #if !defined(USB_USER_DEVICE_DESCRIPTOR)
        //Device descriptor
        extern ROM USB_DEVICE_DESCRIPTOR device_dsc;
    #else
        USB_USER_DEVICE_DESCRIPTOR_INCLUDE;
    #endif

    //Configuration descriptor
    extern ROM BYTE configDescriptor1[];

    #if !defined(USB_USER_CONFIG_DESCRIPTOR)
        //Array of configuration descriptors
        extern ROM BYTE *ROM USB_CD_Ptr[];
    #else
        USB_USER_CONFIG_DESCRIPTOR_INCLUDE;
    #endif

    //Array of string descriptors
    extern ROM BYTE *ROM USB_SD_Ptr[];

    #if defined(USB_USE_HID)
    //Class specific - HID report descriptor
    #if !defined(__USB_DESCRIPTORS_C)
        extern ROM struct{BYTE report[HID_RPT01_SIZE];} hid_rpt01;
    #endif
    #endif

    //Buffer for control transfers
    extern volatile CTRL_TRF_SETUP SetupPkt;           // 8-byte only
    //Buffer for control transfer data
    extern volatile BYTE CtrlTrfData[USB_EP0_BUFF_SIZE];

    #if defined(USB_USE_HID)
    //class specific data buffers
    extern volatile unsigned char hid_report_out[HID_INT_OUT_EP_SIZE];
    extern volatile unsigned char hid_report_in[HID_INT_IN_EP_SIZE];
    #endif


#endif

/* Control Transfer States */
#define WAIT_SETUP          0
#define CTRL_TRF_TX         1
#define CTRL_TRF_RX         2

/* v2.1 fix - Short Packet States - Used by Control Transfer Read  - CTRL_TRF_TX */
#define SHORT_PKT_NOT_USED  0
#define SHORT_PKT_PENDING   1
#define SHORT_PKT_SENT      2

/* USB PID: Token Types - See chapter 8 in the USB specification */
#define SETUP_TOKEN         0x0D    // 0b00001101
#define OUT_TOKEN           0x01    // 0b00000001
#define IN_TOKEN            0x09    // 0b00001001

/* bmRequestType Definitions */
#define HOST_TO_DEV         0
#define DEV_TO_HOST         1

#define STANDARD            0x00
#define CLASS               0x01
#define VENDOR              0x02

#define RCPT_DEV            0
#define RCPT_INTF           1
#define RCPT_EP             2
#define RCPT_OTH            3

/** EXTERNS ********************************************************/
#if !defined(USBDEVICE_C)
    extern BYTE USBDeviceState;
    extern BYTE USBActiveConfiguration;
    extern USB_VOLATILE IN_PIPE inPipes[1];
    extern USB_VOLATILE OUT_PIPE outPipes[1];
    extern volatile BDT_ENTRY *pBDTEntryIn[USB_MAX_EP_NUMBER+1];
#endif

/** PUBLIC PROTOTYPES **********************************************/

/****************************************************************************
  Function:
    void USBDeviceTasks(void)

  Description:
    This function is the main state machine of the 
    USB device side stack.  This function should be
    called periodically to receive and transmit
    packets through the stack.  This function should
    be called  preferably once every 100us 
    during the enumeration process.  After the
    enumeration process this function still needs to
    be called periodically to respond to various
    situations on the bus but is more relaxed in its
    time requirements.  This function should also
    be called at least as fast as the OUT data
    expected from the PC.

  Precondition:
    None

  Parameters:
    None

  Return Values:
    None

  Remarks:
    None
  ***************************************************************************/
void USBDeviceTasks(void);

/**************************************************************************
  Function:
            void USBDeviceInit(void)
    
  Description:
    This function initializes the device stack it in the default state. The
    USB module will be completely reset including all of the internal
    variables, registers, and interrupt flags.
    
    
    
    This function must be called before any of the other USB Device
    functions can be called, including USBDeviceTasks().
  Remarks:
    \ \ 
  Conditions:
    \ \                                                                    
  **************************************************************************/
void USBDeviceInit(void);

void USBRemoteWakeup(void);
void USBSoftDetach(void);
void USBCtrlEPService(void);
void USBCtrlTrfSetupHandler(void);
void USBCtrlTrfInHandler(void);
void USBCheckStdRequest(void);
void USBStdGetDscHandler(void);
void USBCtrlEPServiceComplete(void);
void USBCtrlTrfTxService(void);
void USBPrepareForNextSetupTrf(void);
void USBCtrlTrfRxService(void);
void USBStdSetCfgHandler(void);
void USBStdGetStatusHandler(void);
void USBStdFeatureReqHandler(void);
void USBCtrlTrfOutHandler(void);
BOOL USBIsTxBusy(BYTE EPNumber);
void USBPut(BYTE EPNum, BYTE Data);
void USBEPService(void);
void USBConfigureEndpoint(BYTE EPNum, BYTE direction);
word HIDRxGetCount(void);

void USBProtocolResetHandler(void);
void USBWakeFromSuspend(void);
void USBSuspend(void);
void USBStallHandler(void);
volatile USB_HANDLE USBTransferOnePacket(BYTE ep, BYTE dir, BYTE* data, BYTE len);
void USBEnableEndpoint(BYTE ep,BYTE options);

#if defined(USB_DYNAMIC_EP_CONFIG)
    void USBInitEP(BYTE ROM* pConfig);
#else
    #define USBInitEP(a)
#endif

#if defined(ENABLE_EP0_DATA_RECEIVED_CALLBACK)
    void USBCBEP0DataReceived(void);
    #define USBCB_EP0_DATA_RECEIVED() USBCBEP0DataReceived()
#else
    #define USBCB_EP0_DATA_RECEIVED()
#endif

/** CALLBACKS ******************************************************/
void USBCBSuspend(void);
void USBCBWakeFromSuspend(void);
void USBCB_SOF_Handler(void);
void USBCBErrorHandler(void);
void USBCBCheckOtherReq(void);
void USBCBInitDevClass(void);
void USBCBStdSetDscHandler(void);
void USBCBSendResume(void);
void USBCBInitEP(void);



/** MACROS ******************************************************/

#define DESC_CONFIG_BYTE(a) (a)
#define DESC_CONFIG_WORD(a) (a&0xFF),((a>>8)&0xFF)
#define DESC_CONFIG_DWORD(a) (a&0xFF),((a>>8)&0xFF),((a>>16)&0xFF),((a>>24)&0xFF)

/********************************************************************
 * Function:        BYTE USBHandleBusy(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          BYTE - 1 if the USB module owns the buffer
 *                       - 0 if the CPU owns the module
 *
 * Side Effects:    None
 *
 * Overview:        Retreives the status of the buffer ownership
 *
 * Note:
 *******************************************************************/
#define USBHandleBusy(handle) (handle==0?0:handle->STAT.UOWN)

/********************************************************************
 * Function:        WORD USBHandleGetLength(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          WORD - length of the current buffer settings
 *
 * Side Effects:    None
 *
 * Overview:        Retreives the length of the buffer
 *
 * Note:
 *******************************************************************/
#define USBHandleGetLength(handle) (handle->CNT)

/********************************************************************
 * Function:        WORD USBHandleGetAddr(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          WORD - address of the current buffer
 *
 * Side Effects:    None
 *
 * Overview:        Retreives the address of the buffer
 *
 * Note:
 *******************************************************************/
#define USBHandleGetAddr(handle) (handle->ADR)

/********************************************************************
 * Function:        void USBEP0SetSourceRAM(BYTE* src)
 *
 * PreCondition:    None
 *
 * Input:           src - address of the data to send
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Sets the address of the data to send over the
 *                  control endpoint
 *
 * Note:
 *******************************************************************/
#define USBEP0SetSourceRAM(src) inPipes[0].pSrc.bRam = src

/********************************************************************
 * Function:        void USBEP0SetSourceROM(BYTE* src)
 *
 * PreCondition:    None
 *
 * Input:           src - address of the data to send
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Sets the address of the data to send over the
 *                  control endpoint
 *
 * Note:
 *******************************************************************/
#define USBEP0SetSourceROM(src) inPipes[0].pSrc.bRom = src

/********************************************************************
 * Function:        void USBEP0Transmit(BYTE options)
 *
 * PreCondition:    None
 *
 * Input:           options - the various options that you want
 *                  when sending the control data. Options are:
 *                      USB_INPIPES_ROM
 *                      USB_INPIPES_RAM
 *                      USB_INPIPES_BUSY
 *                      USB_INPIPES_INCLUDE_ZERO
 *                      USB_INPIPES_NO_DATA
 *                      USB_INPIPES_NO_OPTIONS
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Sets the address of the data to send over the
 *                  control endpoint
 *
 * Note:
 *******************************************************************/
#define USBEP0Transmit(options) inPipes[0].info.Val = options | USB_INPIPES_BUSY

/********************************************************************
 * Function:        void USBEP0SetSize(WORD size)
 *
 * PreCondition:    None
 *
 * Input:           size - the size of the data needing to be transmitted
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Sets the size of the data to send over the
 *                  control endpoint
 *
 * Note:
 *******************************************************************/
#define USBEP0SetSize(size) inPipes[0].wCount.Val = size

/********************************************************************
 * Function:        void USBEP0SendRAMPtr(BYTE* src, WORD size, BYTE Options)
 *
 * PreCondition:    None
 *
 * Input:           src - address of the data to send
 *                  size - the size of the data needing to be transmitted
 *                  options - the various options that you want
 *                      when sending the control data. Options are:
 *                      USB_INPIPES_ROM
 *                      USB_INPIPES_RAM
 *                      USB_INPIPES_BUSY
 *                      USB_INPIPES_INCLUDE_ZERO
 *                      USB_INPIPES_NO_DATA
 *                      USB_INPIPES_NO_OPTIONS
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Sets the source, size, and options of the data
 *                  you wish to send from a RAM source
 *
 * Note:
 *******************************************************************/
#define USBEP0SendRAMPtr(src,size,options)  {USBEP0SetSourceRAM(src);USBEP0SetSize(size);USBEP0Transmit(options | USB_EP0_RAM);}

/********************************************************************
 * Function:        void USBEP0SendROMPtr(BYTE* src, WORD size, BYTE Options)
 *
 * PreCondition:    None
 *
 * Input:           src - address of the data to send
 *                  size - the size of the data needing to be transmitted
 *                  options - the various options that you want
 *                      when sending the control data. Options are:
 *                      USB_INPIPES_ROM
 *                      USB_INPIPES_RAM
 *                      USB_INPIPES_BUSY
 *                      USB_INPIPES_INCLUDE_ZERO
 *                      USB_INPIPES_NO_DATA
 *                      USB_INPIPES_NO_OPTIONS
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Sets the source, size, and options of the data
 *                  you wish to send from a ROM source
 *
 * Note:
 *******************************************************************/
#define USBEP0SendROMPtr(src,size,options)  {USBEP0SetSourceROM(src);USBEP0SetSize(size);USBEP0Transmit(options | USB_EP0_ROM);}

/********************************************************************
 * Function:        void USBTxOnePacket(BYTE ep, BYTE* data, WORD len)
 *
 * PreCondition:    None
 *
 * Input:           ep - the endpoint you want to send the data out of
 *                  data - the data that you wish to send
 *                  len - the length of the data that you wish to send
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        sends the specified data out the specified endpoint
 *
 * Note:
 *******************************************************************/
#define USBTxOnePacket(ep,data,len)     USBTransferOnePacket(ep,IN_TO_HOST,data,len)

/********************************************************************
 * Function:        void USBRxOnePacket(BYTE ep, BYTE* data, WORD len)
 *
 * PreCondition:    None
 *
 * Input:           ep - the endpoint you want to receive the data into
 *                  data - where the data will go when it arrives
 *                  len - the length of the data that you wish to receive
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        receives the specified data out the specified endpoint
 *
 * Note:
 *******************************************************************/
#define USBRxOnePacket(ep,data,len)      USBTransferOnePacket(ep,OUT_FROM_HOST,data,len)

/********************************************************************
 * Function:        void USBClearInterruptFlag(WORD reg, BYTE flag)
 *
 * PreCondition:    None
 *
 * Input:
 *   WORD reg - the register name holding the interrupt flag
 *   BYTE flag - the bit number needing to be cleared
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        clears the specified interrupt flag.
 *
 * Note:
 *******************************************************************/
void USBClearInterruptFlag(BYTE* reg, BYTE flag);

/********************************************************************
 * Function:        void USBClearInterruptRegister(WORD reg)
 *
 * PreCondition:    None
 *
 * Input:
 *   WORD reg - the register name that needs to be cleared
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        clears the specified interrupt register.
 *
 * Note:
 *******************************************************************/
#if defined(__18CXX)
    #define USBClearInterruptRegister(reg) reg = 0;
#elif  defined(__C30__)
    #define USBClearInterruptRegister(reg) reg = 0xFF;
#endif

/********************************************************************
 * Function:        void USBStallEndpoint(BYTE ep, BYTE dir)
 *
 * PreCondition:    None
 *
 * Input:
 *   BYTE ep - the endpoint the data will be transmitted on
 *   BYTE dir - the direction of the transfer
 *
 * Output:          None
 *
 * Side Effects:    Endpoint is STALLed
 *
 * Overview:        STALLs the specified endpoint
 *
 * Note:            None
 *******************************************************************/
void USBStallEndpoint(BYTE ep, BYTE dir);


#if (USB_PING_PONG_MODE == USB_PING_PONG__NO_PING_PONG)
    #define USB_NEXT_EP0_OUT_PING_PONG 0x0000   // Used in USB Device Mode only
    #define USB_NEXT_EP0_IN_PING_PONG 0x0000    // Used in USB Device Mode only
    #define USB_NEXT_PING_PONG 0x0000           // Used in USB Device Mode only
    #define EP0_OUT_EVEN    0                   // Used in USB Device Mode only
    #define EP0_OUT_ODD     0                   // Used in USB Device Mode only
    #define EP0_IN_EVEN     1                   // Used in USB Device Mode only
    #define EP0_IN_ODD      1                   // Used in USB Device Mode only
    #define EP1_OUT_EVEN    2                   // Used in USB Device Mode only
    #define EP1_OUT_ODD     2                   // Used in USB Device Mode only
    #define EP1_IN_EVEN     3                   // Used in USB Device Mode only
    #define EP1_IN_ODD      3                   // Used in USB Device Mode only
    #define EP2_OUT_EVEN    4                   // Used in USB Device Mode only
    #define EP2_OUT_ODD     4                   // Used in USB Device Mode only
    #define EP2_IN_EVEN     5                   // Used in USB Device Mode only
    #define EP2_IN_ODD      5                   // Used in USB Device Mode only
    #define EP3_OUT_EVEN    6                   // Used in USB Device Mode only
    #define EP3_OUT_ODD     6                   // Used in USB Device Mode only
    #define EP3_IN_EVEN     7                   // Used in USB Device Mode only
    #define EP3_IN_ODD      7                   // Used in USB Device Mode only
    #define EP4_OUT_EVEN    8                   // Used in USB Device Mode only
    #define EP4_OUT_ODD     8                   // Used in USB Device Mode only
    #define EP4_IN_EVEN     9                   // Used in USB Device Mode only
    #define EP4_IN_ODD      9                   // Used in USB Device Mode only
    #define EP5_OUT_EVEN    10                  // Used in USB Device Mode only
    #define EP5_OUT_ODD     10                  // Used in USB Device Mode only
    #define EP5_IN_EVEN     11                  // Used in USB Device Mode only
    #define EP5_IN_ODD      11                  // Used in USB Device Mode only
    #define EP6_OUT_EVEN    12                  // Used in USB Device Mode only
    #define EP6_OUT_ODD     12                  // Used in USB Device Mode only
    #define EP6_IN_EVEN     13                  // Used in USB Device Mode only
    #define EP6_IN_ODD      13                  // Used in USB Device Mode only
    #define EP7_OUT_EVEN    14                  // Used in USB Device Mode only
    #define EP7_OUT_ODD     14                  // Used in USB Device Mode only
    #define EP7_IN_EVEN     15                  // Used in USB Device Mode only
    #define EP7_IN_ODD      15                  // Used in USB Device Mode only
    #define EP8_OUT_EVEN    16                  // Used in USB Device Mode only
    #define EP8_OUT_ODD     16                  // Used in USB Device Mode only
    #define EP8_IN_EVEN     17                  // Used in USB Device Mode only
    #define EP8_IN_ODD      17                  // Used in USB Device Mode only
    #define EP9_OUT_EVEN    18                  // Used in USB Device Mode only
    #define EP9_OUT_ODD     18                  // Used in USB Device Mode only
    #define EP9_IN_EVEN     19                  // Used in USB Device Mode only
    #define EP9_IN_ODD      19                  // Used in USB Device Mode only
    #define EP10_OUT_EVEN   20                  // Used in USB Device Mode only
    #define EP10_OUT_ODD    20                  // Used in USB Device Mode only
    #define EP10_IN_EVEN    21                  // Used in USB Device Mode only
    #define EP10_IN_ODD     21                  // Used in USB Device Mode only
    #define EP11_OUT_EVEN   22                  // Used in USB Device Mode only
    #define EP11_OUT_ODD    22                  // Used in USB Device Mode only
    #define EP11_IN_EVEN    23                  // Used in USB Device Mode only
    #define EP11_IN_ODD     23                  // Used in USB Device Mode only
    #define EP12_OUT_EVEN   24                  // Used in USB Device Mode only
    #define EP12_OUT_ODD    24                  // Used in USB Device Mode only
    #define EP12_IN_EVEN    25                  // Used in USB Device Mode only
    #define EP12_IN_ODD     25                  // Used in USB Device Mode only
    #define EP13_OUT_EVEN   26                  // Used in USB Device Mode only
    #define EP13_OUT_ODD    26                  // Used in USB Device Mode only
    #define EP13_IN_EVEN    27                  // Used in USB Device Mode only
    #define EP13_IN_ODD     27                  // Used in USB Device Mode only
    #define EP14_OUT_EVEN   28                  // Used in USB Device Mode only
    #define EP14_OUT_ODD    28                  // Used in USB Device Mode only
    #define EP14_IN_EVEN    29                  // Used in USB Device Mode only
    #define EP14_IN_ODD     29                  // Used in USB Device Mode only
    #define EP15_OUT_EVEN   30                  // Used in USB Device Mode only
    #define EP15_OUT_ODD    30                  // Used in USB Device Mode only
    #define EP15_IN_EVEN    31                  // Used in USB Device Mode only
    #define EP15_IN_ODD     31                  // Used in USB Device Mode only

    #define EP(ep,dir,pp) (2*ep+dir)            // Used in USB Device Mode only

    #define BD(ep,dir,pp)   ((8 * ep) + (4 * dir))      // Used in USB Device Mode only

#elif (USB_PING_PONG_MODE == USB_PING_PONG__EP0_OUT_ONLY)
    #define USB_NEXT_EP0_OUT_PING_PONG 0x0004
    #define USB_NEXT_EP0_IN_PING_PONG 0x0000
    #define USB_NEXT_PING_PONG 0x0000
    #define EP0_OUT_EVEN    0
    #define EP0_OUT_ODD     1
    #define EP0_IN_EVEN     2
    #define EP0_IN_ODD      2
    #define EP1_OUT_EVEN    3
    #define EP1_OUT_ODD     3
    #define EP1_IN_EVEN     4
    #define EP1_IN_ODD      4
    #define EP2_OUT_EVEN    5
    #define EP2_OUT_ODD     5
    #define EP2_IN_EVEN     6
    #define EP2_IN_ODD      6
    #define EP3_OUT_EVEN    7
    #define EP3_OUT_ODD     7
    #define EP3_IN_EVEN     8
    #define EP3_IN_ODD      8
    #define EP4_OUT_EVEN    9
    #define EP4_OUT_ODD     9
    #define EP4_IN_EVEN     10
    #define EP4_IN_ODD      10
    #define EP5_OUT_EVEN    11
    #define EP5_OUT_ODD     11
    #define EP5_IN_EVEN     12
    #define EP5_IN_ODD      12
    #define EP6_OUT_EVEN    13
    #define EP6_OUT_ODD     13
    #define EP6_IN_EVEN     14
    #define EP6_IN_ODD      14
    #define EP7_OUT_EVEN    15
    #define EP7_OUT_ODD     15
    #define EP7_IN_EVEN     16
    #define EP7_IN_ODD      16
    #define EP8_OUT_EVEN    17
    #define EP8_OUT_ODD     17
    #define EP8_IN_EVEN     18
    #define EP8_IN_ODD      18
    #define EP9_OUT_EVEN    19
    #define EP9_OUT_ODD     19
    #define EP9_IN_EVEN     20
    #define EP9_IN_ODD      20
    #define EP10_OUT_EVEN   21
    #define EP10_OUT_ODD    21
    #define EP10_IN_EVEN    22
    #define EP10_IN_ODD     22
    #define EP11_OUT_EVEN   23
    #define EP11_OUT_ODD    23
    #define EP11_IN_EVEN    24
    #define EP11_IN_ODD     24
    #define EP12_OUT_EVEN   25
    #define EP12_OUT_ODD    25
    #define EP12_IN_EVEN    26
    #define EP12_IN_ODD     26
    #define EP13_OUT_EVEN   27
    #define EP13_OUT_ODD    27
    #define EP13_IN_EVEN    28
    #define EP13_IN_ODD     28
    #define EP14_OUT_EVEN   29
    #define EP14_OUT_ODD    29
    #define EP14_IN_EVEN    30
    #define EP14_IN_ODD     30
    #define EP15_OUT_EVEN   31
    #define EP15_OUT_ODD    31
    #define EP15_IN_EVEN    32
    #define EP15_IN_ODD     32

    #define EP(ep,dir,pp) (2*ep+dir+(((ep==0)&&(dir==0))?pp:2))
    #define BD(ep,dir,pp) (4*(ep+dir+(((ep==0)&&(dir==0))?pp:2)))

#elif (USB_PING_PONG_MODE == USB_PING_PONG__FULL_PING_PONG)
    #define USB_NEXT_EP0_OUT_PING_PONG 0x0004
    #define USB_NEXT_EP0_IN_PING_PONG 0x0004
    #define USB_NEXT_PING_PONG 0x0004
    #define EP0_OUT_EVEN    0
    #define EP0_OUT_ODD     1
    #define EP0_IN_EVEN     2
    #define EP0_IN_ODD      3
    #define EP1_OUT_EVEN    4
    #define EP1_OUT_ODD     5
    #define EP1_IN_EVEN     6
    #define EP1_IN_ODD      7
    #define EP2_OUT_EVEN    8
    #define EP2_OUT_ODD     9
    #define EP2_IN_EVEN     10
    #define EP2_IN_ODD      11
    #define EP3_OUT_EVEN    12
    #define EP3_OUT_ODD     13
    #define EP3_IN_EVEN     14
    #define EP3_IN_ODD      15
    #define EP4_OUT_EVEN    16
    #define EP4_OUT_ODD     17
    #define EP4_IN_EVEN     18
    #define EP4_IN_ODD      19
    #define EP5_OUT_EVEN    20
    #define EP5_OUT_ODD     21
    #define EP5_IN_EVEN     22
    #define EP5_IN_ODD      23
    #define EP6_OUT_EVEN    24
    #define EP6_OUT_ODD     25
    #define EP6_IN_EVEN     26
    #define EP6_IN_ODD      27
    #define EP7_OUT_EVEN    28
    #define EP7_OUT_ODD     29
    #define EP7_IN_EVEN     30
    #define EP7_IN_ODD      31
    #define EP8_OUT_EVEN    32
    #define EP8_OUT_ODD     33
    #define EP8_IN_EVEN     34
    #define EP8_IN_ODD      35
    #define EP9_OUT_EVEN    36
    #define EP9_OUT_ODD     37
    #define EP9_IN_EVEN     38
    #define EP9_IN_ODD      39
    #define EP10_OUT_EVEN   40
    #define EP10_OUT_ODD    41
    #define EP10_IN_EVEN    42
    #define EP10_IN_ODD     43
    #define EP11_OUT_EVEN   44
    #define EP11_OUT_ODD    45
    #define EP11_IN_EVEN    46
    #define EP11_IN_ODD     47
    #define EP12_OUT_EVEN   48
    #define EP12_OUT_ODD    49
    #define EP12_IN_EVEN    50
    #define EP12_IN_ODD     51
    #define EP13_OUT_EVEN   52
    #define EP13_OUT_ODD    53
    #define EP13_IN_EVEN    54
    #define EP13_IN_ODD     55
    #define EP14_OUT_EVEN   56
    #define EP14_OUT_ODD    57
    #define EP14_IN_EVEN    58
    #define EP14_IN_ODD     59
    #define EP15_OUT_EVEN   60
    #define EP15_OUT_ODD    61
    #define EP15_IN_EVEN    62
    #define EP15_IN_ODD     63

    #define EP(ep,dir,pp) (4*ep+2*dir+pp)

    #define BD(ep,dir,pp) (4*(4*ep+2*dir+pp))

#elif (USB_PING_PONG_MODE == USB_PING_PONG__ALL_BUT_EP0)
    #define USB_NEXT_EP0_OUT_PING_PONG 0x0000
    #define USB_NEXT_EP0_IN_PING_PONG 0x0000
    #define USB_NEXT_PING_PONG 0x0004
    #define EP0_OUT_EVEN    0
    #define EP0_OUT_ODD     0
    #define EP0_IN_EVEN     1
    #define EP0_IN_ODD      1
    #define EP1_OUT_EVEN    2
    #define EP1_OUT_ODD     3
    #define EP1_IN_EVEN     4
    #define EP1_IN_ODD      5
    #define EP2_OUT_EVEN    6
    #define EP2_OUT_ODD     7
    #define EP2_IN_EVEN     8
    #define EP2_IN_ODD      9
    #define EP3_OUT_EVEN    10
    #define EP3_OUT_ODD     11
    #define EP3_IN_EVEN     12
    #define EP3_IN_ODD      13
    #define EP4_OUT_EVEN    14
    #define EP4_OUT_ODD     15
    #define EP4_IN_EVEN     16
    #define EP4_IN_ODD      17
    #define EP5_OUT_EVEN    18
    #define EP5_OUT_ODD     19
    #define EP5_IN_EVEN     20
    #define EP5_IN_ODD      21
    #define EP6_OUT_EVEN    22
    #define EP6_OUT_ODD     23
    #define EP6_IN_EVEN     24
    #define EP6_IN_ODD      25
    #define EP7_OUT_EVEN    26
    #define EP7_OUT_ODD     27
    #define EP7_IN_EVEN     28
    #define EP7_IN_ODD      29
    #define EP8_OUT_EVEN    30
    #define EP8_OUT_ODD     31
    #define EP8_IN_EVEN     32
    #define EP8_IN_ODD      33
    #define EP9_OUT_EVEN    34
    #define EP9_OUT_ODD     35
    #define EP9_IN_EVEN     36
    #define EP9_IN_ODD      37
    #define EP10_OUT_EVEN   38
    #define EP10_OUT_ODD    39
    #define EP10_IN_EVEN    40
    #define EP10_IN_ODD     41
    #define EP11_OUT_EVEN   42
    #define EP11_OUT_ODD    43
    #define EP11_IN_EVEN    44
    #define EP11_IN_ODD     45
    #define EP12_OUT_EVEN   46
    #define EP12_OUT_ODD    47
    #define EP12_IN_EVEN    48
    #define EP12_IN_ODD     49
    #define EP13_OUT_EVEN   50
    #define EP13_OUT_ODD    51
    #define EP13_IN_EVEN    52
    #define EP13_IN_ODD     53
    #define EP14_OUT_EVEN   54
    #define EP14_OUT_ODD    55
    #define EP14_IN_EVEN    56
    #define EP14_IN_ODD     57
    #define EP15_OUT_EVEN   58
    #define EP15_OUT_ODD    59
    #define EP15_IN_EVEN    60
    #define EP15_IN_ODD     61

    #define EP(ep,dir,pp) (4*ep+2*dir+((ep==0)?0:(pp-2)))
    #define BD(ep,dir,pp) (4*(4*ep+2*dir+((ep==0)?0:(pp-2))))

#else
    #error "No ping pong mode defined."
#endif


#endif //USBD_H
