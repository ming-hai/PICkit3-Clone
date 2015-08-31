/*********************************************************************
 *
 * System Specific Variables
 *
 *********************************************************************
 * FileName:            system.h
 * Dependencies:        processor.h
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

#ifndef SYSTEM_DOT_H
#define SYSTEM_DOT_H

#include "processor.h"


/*********************************************************************
 * Defines
 *********************************************************************/

// Clock Specifics
#define __FCY                   16000000L           // (confirmed)

#define __NORM_US_FCY           (__FCY/10000)  // normalized for a us conversion

#define DEFAULT_PRIORITY_LEVEL      0

#define Reset() {__asm__ volatile ("reset");}
#define NOP() {__asm__ volatile ("nop");}

#ifdef __DEBUG
    #define _HALT() {__asm__ volatile (".pword 0xDA4000");}
#else
    #define _HALT()
#endif

#define SAFETY_DOWNLOAD_KEY 0xA512345A  

/************************************************************************
  Requests that can be made to the interrupt controller connected to the
  IRQ pins: This is currently for completely disabling on turning on IRQs
  ************************************************************************/
typedef enum _INTERRUPTCONTROL
{
    SYS_INTERRUPT_DISABLE,
    SYS_INTERRUPT_ENABLE,
    SYS_INTERRUPT_STATUS,
    SYS_INTERRUPT_UNKNOWN
} INTERRUPTCONTROL;

// Prototype
INTERRUPTCONTROL System_InterruptController(INTERRUPTCONTROL InterruptRequest);

extern INTERRUPTCONTROL* InterruptStatus;

// These macros actually store off the current IPL and save it
#define     INTERRUPT_DISABLE_VALUE         6

#define RE_DISABLE_SYSTEM_INTERRUPT SET_AND_SAVE_CPU_IPL(___current_cpu_ipl, INTERRUPT_DISABLE_VALUE); \
                                    *InterruptStatus = SYS_INTERRUPT_DISABLE; 
                                    

#define DISABLE_SYSTEM_INTERRUPT    volatile int    ___current_cpu_ipl;\
                                    RE_DISABLE_SYSTEM_INTERRUPT 

#define RE_RESTORE_SYSTEM_INTERRUPT *InterruptStatus = SYS_INTERRUPT_UNKNOWN; \
                                    RESTORE_CPU_IPL(___current_cpu_ipl);                    
                                        
// Just restoring the IPL. Don't know if its enabled or disabled at this point                                      
#define RESTORE_SYSTEM_INTERRUPT    RE_RESTORE_SYSTEM_INTERRUPT

/*********************************************************************
 * PIN DEFINITIONS
 *********************************************************************
 * Interrupts
 * INT1 - MBUFFER_RDY
 * INT2 - MTIMER_RDY
 * INT3 - USB Interrupt
 *********************************************************************/

//--------------------------------------------------------------------
// Driver Board pins
//--------------------------------------------------------------------
#define     pin_CLK_EN          _LATE0
#define     tris_CLK_EN         _TRISE0

#define     SetCLKDIR_DEBUG     
#define     SetCLKDIR_PGM       

#define     CLK_DIR(r)          {pin_CLK_EN = r; tris_MSCK = !r;}

#define     pin_DATA_EN         _LATE1
#define     tris_DATA_EN        _TRISE1

#define     DATA_DIR(r)         pin_DATA_EN = r

#define     LVP_DIR(r)          {pin_LVP_EN = r; tris_LVP = !r;}

#define     OUT_DIR         1U
#define     IN_DIR          0U

// auxillary pins to the driver board
#define     pin_AUX1_EN         _LATD6      // TP1 on schematics
#define     tris_AUX1_EN        _TRISD6

#define     pin_AUX2_EN         _LATB1      // TP2 on schematics
#define     tris_AUX2_EN        _TRISB1     

#define     pin_AUX3_EN         _LATD10     // TP3 on schematics
#define     tris_AUX3_EN        _TRISD10

#define     pin_AUX4_EN         _LATD9      // TP4 on schematics
#define     tris_AUX4_EN        _TRISD9

#define     pin_AUX5_EN         _LATB14      // DB0 schematics
#define     tris_AUX5_EN        _TRISB14

#define     pin_AUX6_EN         _LATD1      // DB1 schematics
#define     tris_AUX6_EN        _TRISD1

#define     pin_LVP             _LATF3
#define     pin_LVP_in          _RF3
#define     tris_LVP            _TRISF3

#define     pin_LVP_EN          _LATE2
#define     tris_LVP_EN         _TRISE2

#define     pin_5V_USB_ENABLE   _LATF1      // enable VBUS to be propagated to 5V_USB
#define     tris_5V_USB_ENABLE  _TRISF1

#define     pin_5V_USB_GOOD     _RE4
#define     tris_5V_USB_GOOD    _TRISE4

#define     pin_POWER_GOOD      _RB12
#define     tris_POWER_GOOD     _TRISB12

#define     pin_MSDO            _LATD4
#define     tris_MSDO           _TRISD4

#define     pin_MSDI            _RD3
#define     tris_MSDI           _TRISD3

#define     pin_MSCK            _RD2
#define     tris_MSCK           _TRISD2
//--------------------------------------------------------------------
// Power Supply controls
//--------------------------------------------------------------------

#define     pin_INT_NVDD_GND    _LATE7
#define     tris_INT_NVDD_GND   _TRISE7
#define     pin_SUPPLY_PWR      _LATE3  // this needs to be an open drain
#define     tris_SUPPLY_PWR     _TRISE3 // configuration to be able to
#define     odc_SUPPLY_PWR      _ODE3   // drive it all the way to 5 volts

#define     INIT_TO_DEBUGGER_SOURCE     {  \
    pin_INT_NVDD_GND    = 1;               \
    pin_SUPPLY_PWR      = 0;               \
    odc_SUPPLY_PWR      = 1;               \
    tris_INT_NVDD_GND   = OUTPUT;          \
    tris_SUPPLY_PWR     = OUTPUT;          \
                                        }
#define     SELECT_DEBUGGER_SOURCE      {  \
    pin_INT_NVDD_GND    = 1;               \
    pin_SUPPLY_PWR      = 0;               \
                                        }
#define     SELECT_TARGET_SOURCE        {  \
    pin_INT_NVDD_GND    = 1;               \
    pin_SUPPLY_PWR      = 1;               \
                                        }
#define     SELECT_DRAIN_VDD_FAST       {  \
    pin_INT_NVDD_GND    = 0;               \
    pin_SUPPLY_PWR      = 1;               \
                                        }

#define     pin_VPP_ON          _LATE5
#define     tris_VPP_ON         _TRISE5

#define     pin_VPP_GROUND      _LATE6      // making this one, drives VPP to gnd (to quickly discharge it).
#define     tris_VPP_GROUND     _TRISE6

#define     pin_VPP_PUMP        _LATB5
#define     tris_VPP_PUMP       _TRISB5

#define     pin_VDD_PUMP        _LATB4
#define     tris_VDD_PUMP       _TRISB4

#define     pin_AN3_VPP_FB      _LATB3
#define     tris_AN3_VPP_FB     _TRISB3

#define     pin_AN2_VDD_FB      _LATB2
#define     tris_AN2_VDD_FB     _TRISB2

#define     pin_LED_ACTIVE      _LATB9
#define     tris_LED_ACTIVE     _TRISB9

#define     pin_LED_RESET       _LATB10
#define     tris_LED_RESET      _TRISB10

#define     pin_LED_HALT        _LATB11
#define     tris_LED_HALT       _TRISB11

#define     pin_BUTTON          _RB8
#define     tris_BUTTON         _TRISB8

#define     Vpp_ON_pin          _RE5
#define     VPP_Gnd_pin         _RE6
#define     Vdd_TGT_P_pin       _RE3 // TODO: open drain?
#define     NVdd_TGT_N_pin      _RE7

//--------------------------------------------------------------------
// Debug
//--------------------------------------------------------------------
#define     pin_DEBUG           _RD6            // TP1
#define     tris_DEBUG          tris_AUX1_EN
#define     DEBUG_PORT          PORTD
#define     DEBUG_PIN           6

#define     pin_DEBUG2          pin_AUX2_EN     // TP2
#define     tris_DEBUG2         tris_AUX2_EN

#define     pin_DEBUG3          pin_AUX3_EN     // TP3
#define     tris_DEBUG3         tris_AUX3_EN

#define     pin_DEBUG4          pin_AUX4_EN     // TP4
#define     tris_DEBUG4         tris_AUX4_EN

#define     pin_DEBUG5          pin_AUX5_EN     // DB0
#define     tris_DEBUG5         tris_AUX5_EN

#define     pin_DEBUG6          pin_AUX6_EN     // DB1
#define     tris_DEBUG6         tris_AUX6_EN

#define     pinD                pin_DEBUG5
#define     pinD2               pin_DEBUG6

// Unique to PICkit3
// It has 2 serial EEs, one for the serial number (1) and another for code image (2)
//
// UTIL lines are common to both serial EEs
#define pin_UTIL_SCK            _RD8        
#define tris_UTIL_SCK           _TRISD8

#define pin_UTIL_SDI            _RD11        
#define tris_UTIL_SDI           _TRISD11

#define pin_UTIL_SDO            _LATD0        
#define tris_UTIL_SDO           _TRISD0

// UTIL1 is SPI connection to serial EE with code image

#define pin_UTIL1_nCS           _LATD5        
#define tris_UTIL1_nCS          _TRISD5

#define pin_UTIL1_nWP           _LATD7        
#define tris_UTIL1_nWP          _TRISD7

// UTIL2 is SPI connection to serial EE with code image

#define pin_UTIL2_nCS           _LATG9       
#define tris_UTIL2_nCS          _TRISG9

#define pin_UTIL2_nWP           _LATF0        
#define tris_UTIL2_nWP          _TRISF0

// Lines used to talk to the PIC16F636
#define pin_KC                  _LATC14 
#define tris_KC                 _TRISC14

#define pin_KD                  _LATC13 
#define tris_KD                 _TRISC13

// UART (connected to TP5 and TP6)
#define pic_DEBUG_UART_RX       _RF5
#define tris_DEBUG_UART_RX      _TRISF5
#define pic_DEBUG_UART_TX       _RF4
#define tris_DEBUG_UART_TX      _TRISF4    

#define pin_VREF_2_5V           _RB0
#define tris_VREG_2_5V          _TRISB0

#define pin_REFCLKO             _LATB15
#define tris_REFCLKO            _TRISB15

/*********************************************************************
 *********************************************************************
 * MACROS
 *********************************************************************
 *********************************************************************/


// LEDS 
//   -IGNORE THE NAMES of the pins, use these MACROS
#define     SetLED_BUSY()       pin_LED_RESET = 1; pin_LED_HALT = 1; 
#define     SetLED_GOOD()       pin_LED_RESET = 0; pin_LED_HALT = 1;
#define     SetLED_ERROR()      pin_LED_RESET = 1; pin_LED_HALT = 0;
#define     SetLED_IDLE()       pin_LED_RESET = 0; pin_LED_HALT = 0;

#define     SetLED_ACTIVE(r)      pin_LED_ACTIVE = r

void TurnVPP(byte state);
#define PK3_BL_OS_SWITCH_ADDRESS 0x2A000    // right after the BL 
// If PK3_BL_OS_SWITCH_ADDRESS contains PK3_JUMP_TO_BOOT, we jump to boot, else
// we jump to OS
#define PK3_JUMP_TO_BOOT         0xBEEF

// not compilable, just wrote for my refrence
// #define _ADDR_BUS            LATD[0..6],LATG6
// #define _DATA_BUS            LATB[8..15]

#define defAddrBusMask          0x007F   
#define defDataBusMask          0xFF00

// A2D Defines
#define A2DSAMPLES      8

#define VDD_CHN_2       0x0002
#define VPP_CHN_3       0x0003
#define TGT_CHN_6       0x0006

#define SAFETY_DOWNLOAD_KEY 0xA512345A

extern BYTE **gblCurIntRamLoc;

#endif// end of h file declaration
