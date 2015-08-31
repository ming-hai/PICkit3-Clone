/*********************************************************************
 *
 * Peripheral Pin Select setup routines
 *
 *********************************************************************
 * FileName:            xbparpk3.c
 * Dependencies:        types.h system.h
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

#include "types.h"
#include "system.h"

void UNLOCK_PPS(void);
void LOCK_PPS(void);
        
void UNLOCK_PPS(void)
{
asm volatile ( "MOV #OSCCON, w1     \n"
                "MOV #0x46, w2      \n"
                "MOV #0x57, w3      \n"
                "MOV.b w2, [w1]     \n"
                "MOV.b w3, [w1]     \n"
                "BCLR OSCCON,#6");
}

void LOCK_PPS(void)
{
asm volatile ( "MOV #OSCCON, w1     \n"
                "MOV #0x46, w2      \n"
                "MOV #0x57, w3      \n"
                "MOV.b w2, [w1]     \n"
                "MOV.b w3, [w1]     \n"
                "BSET OSCCON,#6");
}

//    PIN NAMEs (as shown in data sheet)              Name in systems.h          
//    ----------------------------------------+----------------------------------
//    C3INB/CN15/RD6                                  pin_AUX1_EN                
//    PMRD/RP20/CN14/RD5                              pin_UTIL1_nCS              
//    PMWR/RP25/CN13/RD4                              pin_MSDO                   
//    RP22/PMBE/CN52/RD3                              pin_MSDI                   
//    DPH/RP23/CN51/RD2                               pin_MSCK                   
//    RP24/VCPCON/CN50/RD1                            pin_AUX6_EN                
//    PMD4/CN62/RE4                                   pin_5V_USB_GOOD            
//    PMD3/CN61/RE3                                   pin_SUPPLY_PWR             
//    PMD2/CN60/RE2                                   pin_LVP_EN                 
//    PMD1/CN59/RE1                                   pin_DATA_EN                
//    VBUSST/VCMPST1/CN68/RF0                         pin_UTIL2_nWP              
//    VCAP/VDDCORE                                    VDDCORE                    
//    SOSCI/C3IND/CN1/RC13                            pin_KD                     
//    RP11/DMH/CN49/INT0/RD0                          pin_UTIL_SDO               
//    RP3/SCL1/PMCS2/CN55/RD10                        pin_AUX3_EN                
//    RP4/DPLN/SDA1/CN54/RD9                          pin_AUX4_EN                
//    RP2/DMLN/RTCC/CN53/RD8                          pin_UTIL_SCK               
//    RP12/PMCS1/CN56/RD11                            pin_UTIL_SDI               
//    OSCO/CLKO/CN22/RC15                             OSC2                       
//    OSCI/CLKI/CN23/RC12                             OSC1                       
//    VDD                                             ------------               
//    D+/RG2                                          ------------               
//    VUSB                                            ------------               
//    VBUS                                            ------------               
//    RP16/USBID/CN71/RF3                             pin_LVP                    
//    D-/RG3                                          ------------               
//    RPI37/SOSCO/C3INC/TICK/                         pin_KC                     
//    AVDD                                            ------------               
//    RP8/AN8/CN26/RB8                                pin_BUTTON                 
//    PMA7/RP9/AN9/CN27/RB9                           pin_LED_ACTIVE             
//    TMS/PMA13/AN10/CVREF/CN28/RB10                  pin_LED_RESET              
//    TDO/AN11/PMA12/CN29/RB11                        pin_LED_HALT               
//    VDD                                             ------------               
//    PGEC2/AN6/RP6/CN24/RB6                          ICSP PGC                   
//    PGED2/RCV/RP7/AN7/CN25/RB7                      ICSP PGD                   
//    PMA8/RP17/SCL2/CN18/RF5                         pin_DEBUG_UART_RX          
//    PMA9/RP10/SDA2/CN17/RF4                         pin_DEBUG_UART_TX          
//    PMD5/CN63/RE5                                   pin_VPP_ON                 
//    PMD6/SCL3/CN64/RE6                              pin_VPP_GROUND             
//    PMD7/SDA3/CN65/RE7                              pin_INT_NVDD_GND           
//    PMA5/RP21/C1IND/CN8/RG6                         pin_JTAG_TCK               
//    VDD                                             ------------               
//    PGEC3/RP18/VBUSON/C1INA/AN5/CN7/RB5             pin_VPP_PUMP               
//    PGED3/RP28/USBOEN/C1INB/AN4/CN6/RB4             pin_VDD_PUMP               
//    VPIO/C2INA/AN3/CN5/RB3                          tris_AN3_VPP_FB            
//    VMIO/RP13/C2INB/AN2/CN4/RB2                     tris_AN2_VDD_FB            
//    RP26/PMA4/C1INC/CN9/RG7                         pin_JTAG_TDO               
//    PMA3/RP19/C2IND/CN10/RG8                        pin_JTAG_TDI               
//    PGEC1/RP1/VREF-/AN1/CN3/RB1                     pin_AUX2_EN                
//    PGED1/RP0/PMA6/VREF+/AN0/CN2/RB0                tris_VREG_2_5V             
//    RP27/PMA2/C2INC/CN11/RG9                        pin_UTIL2_nCS              
//    MCLR                                            ------------               
//    TCK/PMA11/AN12/CTED2/CN30/RB12                  tris_POWER_GOOD            
//    TDI/PMA10/AN13/CTED1/CN31/RB13                  pin_JTAG_TMS               
//    CTPLS/RP14/PMA1/AN14/CN32/RB14                  pin_AUX5_EN                
//    RP29/PMA0/AN15/REFO/CN12/RB15                   tris_REFCLKO               
//    PMD0/CN58/RE0                                   pin_CLK_EN                 
//    VCMPST2/CN69/RF1                                pin_5V_USB_ENABLE          
//    C3INA/CN16/RD7                                  pin_UTIL1_nWP
//    VSS                                             ------------               
//    VSS                                             ------------               
//    VSS                                             ------------               
//    ENVREG                                          ------------               
//    AVSS                                            ------------               
//    CN0/RC14
//    

// Could not find macros representing values in TABLE 9-2 in DS39897B-page 125
#define PK3_UART1_TX           3 
#define PK3_SDO1               7
#define PK3_SCK1OUT            8
#define PK3_SDO2              10
#define PK3_SCK2OUT           11
#define PK3_OC1               18
#define PK3_OC2               19

void InitializePeripheralPinSelect(void)
{
    // Peripheral assignement:
    //  SPI1    --> ICSP 
    //  SPI2    --> serial EEs (both)
    //  OC1     --> VPP_PUMP
    //  OC2     --> VDD_PUMP
    
    //
    // Output pin selection
    // Pin number               Function            Comments
    // ------------------+--------------------+------------------------
    // RP0                       ----             AN0 is used to read ref 2.5
    // RP1                       ----             pin_AUX2_EN
    // RP2                       SPI2             pin_UTIL_SCK
    // RP3                       ----             pin_AUX3_EN
    // RP4                       ----             pin_AUX4_EN
    // RP6                       ----             ICSP PGC
    // RP7                       ----             ICSP PGD
    // RP8                       ----             pin_BUTTON
    // RP9                       ----             pin_LED_ACTIVE
    // RP10                      UART1            pin_DEBUG_UART_TX
    // RP11                      SPI2             pin_UTIL_SDO (usb serial number EE)
    // RP12                      SPI2             pin_UTIL_SDI (usb serial number EE)
    // RP13                      ----             tris_AN2_VDD_FB
    // RP14                      ----             pin_AUX5_EN
    // RP16                      ----             pin_LVP
    // RP17                      UART1            pin_DEBUG_UART_RX
    // RP18                      OC1              pin_VPP_PUMP
    // RP19                      ----             pin_JTAG_TDI
    // RP20                      ----             pin_UTIL1_nCS
    // RP21                      ----             pin_JTAG_TCK
    // RP22                      SPI1             pin_MSDI
    // RP23                      SPI1             pin_MSCK
    // RP24                      ----             pin_AUX6_EN
    // RP25                      SPI1             pin_MSDO
    // RP26                      ----             pin_JTAG_TDO  
    // RP27                      ----             pin_UTIL2_nCS
    // RP28                      OC2              pin_VDD_PUMP
    // RP29                      ----             tris_REFCLKO
    // RPI37                     ----             pin_KC
    
    // RPORX: (SFR named after pin #)
    //      +------------------------------------+
    //      |    FUNCTION N+1 |   FUNCTION N     |
    //      +------------------------------------+
    //
    //
    // RPINRX: (SFR named after function)
    //      +------------------------------------+
    //      |    PIN N+1      |   PIN N          |
    //      +------------------------------------+
    //
    UNLOCK_PPS();
    
    // UART 1 (used for debugging only [connected to test points on board])
    _RP10R  = PK3_UART1_TX;     // pin_DEBUG_UART_TX
    _U1RXR = 17;               // pin_DEBUG_UART_RX
    
    // SPI1 (used to talk to target ICSP interface)
    _RP25R  = PK3_SDO1;         // pin_MSDO
    _RP23R  = PK3_SCK1OUT;      // pin_MSCK (output)
    _SDI1R  = 22;               // pin_MSDI
    _SCK1R  = 23;               // pin_MSCK (input)

    // SPI2 (used to talk to serial number EE and to code image EE,
    //       they share the same SPI bus)
    _RP11R  = PK3_SDO2;         // pin_UTIL_SDO
    _RP2R  = PK3_SCK2OUT;       // pin_UTIL_SCK
    _SDI2R  = 12;               // pin_UTIL_SDI
    
    // OC1 (used to modulate VPP)
    _RP18R  = PK3_OC1;         // pin_VPP_PUMP

    // OC2 (used to modulate VDD)
    _RP28R  = PK3_OC2;         // pin_VDD_PUMP

    LOCK_PPS();
}


