/*********************************************************************
 *
 * Main Execution Code
 *
 *********************************************************************
 * FileName:            main.c
 * Dependencies:        pickit3.h pk3_scripting.h
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

#include "pickit3.h"
#include "pk3_scripting.h"

//********************************************************************
// Globals
//********************************************************************

SYS_VER gblSysVersion;
BYTE    **gblCurIntRamLoc;
bool    gblWINDOWS_2000_HACK = FALSE;

BYTE __attribute__ ((section ("buffer_space"))) _gblIntRamStorage[INTERNAL_SRAM_BUFFER_SIZE];

const OS_HEADER HEADER __attribute__ ((section(".header"), space(prog))) = {OS_MAGICKEY,{OS_VER}};

INTERRUPTCONTROL    _InterruptStatus;   
INTERRUPTCONTROL    *InterruptStatus; 

//********************************************************************
// Local Functions
//********************************************************************

bool Initialize(void);
void InitPorts(void);
void InitVersions(void);
void ProcessCommand(void);

//********************************************************************
// External Functions
//********************************************************************

extern void InitPowerTrain(void);
extern bool USBStart(void);
extern bool USBInitialize(void);
extern void InitializePeripheralPinSelect(void);
extern void SetVDDPowerSense(typVddSense state);
extern void InitScripting();
extern void ScriptingHandler();

/******************************************************************************
 * Function:        int main ( void )
 *
 * Overview:        Main program entry point.
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/

int main ( void )
{
    bool Init;

    // startup the system
    Init = Initialize();

    // Start periodic timer
    //StartPeriodicTimer();

    // Loop Forever
    FOREVER
    {
        if (Init)
            ProcessCommand();
        else
        {
            SetLED_ERROR();
            SLEEP(250);
            SetLED_IDLE();
            SLEEP(250);
        }
    }
}

/******************************************************************************
 * Function:        bool Initialize(void)
 *
 * Overview:        Initializes the system. Starts up USB.
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          Success/Fail
 *
 * Side Effects:    
 *
 * Note:            None
 *****************************************************************************/

bool Initialize(void)
{
    bool status = TRUE;
    
    // setup the runtime level
    SET_CPU_IPL(DEFAULT_PRIORITY_LEVEL); 
    
    InterruptStatus = &_InterruptStatus;

    InitializePeripheralPinSelect();

    // Set speed to internal PLL 96MHz (from external 12 MHz crystal)
    OSCCON = 0x3302;    // Enable secondary oscillator, use HS oscillator
    CLKDIV = 0x0000;    // (not needed - FRC post-scaler (1:1)), USB postscaler (1:1), CPU postscaler (1:1)
    
    // Initialize all the port Pins
    InitPorts();

    InitVersions();

    SLEEP(170);
            
    SetupPushButtonIRQ();
        
    InitSEEs();
        
    // load the serial number up. If its wrong that is it! End of game
    LoadSerialNumber();
    
    // load up the Unit ID/Friendly Name if it has one.
	LoadToolName();
    
    // Configure TIMER2, enable interrupt
    SetupPeriodicTimer();

    // Set it to internal power
    SetVDDPowerSense(TARGETSENSE);  
        
    //setup the USB subsystem
    if(!USBInitialize())
    {
        status = FALSE;
    }

    // Now tell the USB to start up
    if(!USBStart())
    {
        status = FALSE;
    }

    // We cannot init the power train until we are granted the power by the host.
    // USBStart turns the power on to the rest of the board (from the Vbus)
    // Initialize the power train
    InitPowerTrain();

    if(status)
    {
        SetLED_IDLE();
    }

    // Initialize the main scripting engine
    InitScripting();

    return(status);
}

/******************************************************************************
 * Function:        void InitPorts(void)
 *
 * Overview:        Sets ports to expected states
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
void InitPorts(void)
{   
    // Specify the Analog Pins
    //      AN0 (ref 2.5), AN2 (VDD_FBACK), AN3 (VPP_FBACK)
    AD1PCFG = 0xFFF2;

    //--------------------------------------------------------------------
    // User interface, PB and LEDs
    //--------------------------------------------------------------------
    tris_LED_ACTIVE     = OUTPUT;
    tris_LED_HALT       = OUTPUT;
    tris_LED_RESET      = OUTPUT;
    
    // Turn on the LEDs
    SetLED_ACTIVE(1);
    SetLED_BUSY();
    
    #ifdef _HAS_LVP_
        // New LVP pin, make it HIGH-Z
        tris_LVP = OUTPUT;
        tris_LVP_EN = OUTPUT;
        pin_LVP = 0;
        LVP_DIR(IN_DIR);
    #endif
    
    CLK_DIR(OUT_DIR);
    tris_CLK_EN         = OUTPUT;
    
    DATA_DIR(OUT_DIR);
    tris_DATA_EN        = OUTPUT;
        
    pin_AUX2_EN         = 0;
    pin_AUX1_EN         = 0;
    tris_AUX2_EN        = OUTPUT;
    tris_AUX1_EN        = OUTPUT;
    
    tris_MSDI           = INPUT;
    tris_MSDO           = OUTPUT;
    tris_MSCK           = INPUT;
    
    pin_MSDO            = 0;
    pin_MSCK            = 0;
    
    pin_VPP_ON          = 0;
    pin_VPP_PUMP        = 0;
    pin_VDD_PUMP        = 0;
    tris_VPP_ON         = OUTPUT;
    tris_VPP_GROUND     = OUTPUT;
    tris_VPP_PUMP       = OUTPUT;
    tris_VDD_PUMP       = OUTPUT;
            
    // Set for debugger sourced
    INIT_TO_DEBUGGER_SOURCE

    // Analog input pins
    tris_AN3_VPP_FB     = INPUT;
    tris_AN2_VDD_FB     = INPUT;
    
    // Debug PIN
    tris_DEBUG = OUTPUT;
    tris_DEBUG2 = OUTPUT;
    tris_DEBUG3 = OUTPUT;
    tris_DEBUG4 = OUTPUT;
    tris_DEBUG5 = OUTPUT;
    tris_DEBUG6 = OUTPUT;       
}

/******************************************************************************
 * Function:       void InitVersions(void)
 *
 * Overview:        Fill in the gblSysVersion structure
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    gblSysVersion is modified
 *
 * Note:            None
 *****************************************************************************/
void InitVersions(void)
{
    // set it to all zeros
    memset((SYS_VER *)&gblSysVersion,0x0,sizeof(SYS_VER));
        
    // Set the main Version
    FILLVERVAR(gblSysVersion, OS_VER);  
}

/******************************************************************************
 * Function:        void ProcessCommand(void)
 *
 * Overview:        Responds to USB commands from the host and runs the script
 *                  handler to handle them
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
void ProcessCommand(void)
{
    // Get all the commands while they are being sent
    while(USBDataReady())
    {
        ScriptingHandler();

        // Flush the transfer buffer
        USBflush();
    }
}// end process command
