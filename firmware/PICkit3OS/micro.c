/*********************************************************************
 *
 * Microcontroller functions code
 *
 *********************************************************************
 * FileName:            micro.c
 * Dependencies:        pickit3.h
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

//********************************************************************
// Defines
//********************************************************************
#define DELAYOFFSET         18          // It takes around 17.7us for the delay function setup

//********************************************************************
// Globals
//********************************************************************
void                (*gblTimerEvent)(void *);
TIMERMETHOD         gblTimerMethod;
void                *gblTimerEventAttribs;
BYTE                gblMainSer[SERIAL_NUM_MAX_SIZE];
BYTE                gblToolName[TOOL_NAME_MAX_SIZE];
BYTE                gOneRow[ROWDATASIZE];
SYS_VER gblBootloaderVersion;

//********************************************************************
// Local Functions
//********************************************************************

bool GetSerialNumber(char *Serial);
bool GetToolName(char *Name);
void EraseSection(DWORD address);
void ProgramOneRow(unsigned char *buffer, DWORD address);
bool VerTest(SYS_VER ThisVersion, SYS_VER TestVersion);
bool TestBLFooter(void);
bool TestBootloader(void);
void WriteAPBootloaderSwitch(void);
void ProgramOneRow(BYTE *buffer, DWORD address);
void WriteRow(void);
void WriteLatch(WORD addrLo, WORD dataHi, WORD dataLo);

/******************************************************************************
 * Function:        void SetupPushButtonIRQ(void)
 *
 * Overview:        Sets up the change notification interrupt for detecting
 *                  push button presses
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

void SetupPushButtonIRQ(void)
{   
    // Just pins 9, 10 and 11
    CNEN1 = 0x0E00;
    CNEN2 = 0x0000;
    
    _CNIF = 0x0;
    
    // low priority
    _CNIP = 1;              // INTERRUPT PRIORITY LEVEL
    
    _CN26IE  = 1;           // turn on the change notification
    
    // now enable it
    _CNIE = 1;  
}

/******************************************************************************
 * Function:        void SetVDDPowerSense(typVddSense state)
 *
 * Overview:        Changes power source
 *
 * PreCondition:    None
 *
 * Input:           state - TARGETSENSE / INTERNALSENSE
 *
 * Output:          None
 *
 * Side Effects:    Changes to the power circuit
 *
 * Note:            None
 *****************************************************************************/

void SetVDDPowerSense(typVddSense state)
{
    // Ok set the voltage to the target sense
    if(state == TARGETSENSE)
    {
        SELECT_TARGET_SOURCE
    }
    else
    {
        SELECT_DEBUGGER_SOURCE                  
    }
}

/******************************************************************************
 * Function:        void SetupTimer(dword useconds, TIMERMETHOD method ,void (*fcn)(void *), void * attribs)
 *
 * Overview:        Sets up a timer given a timebase in useconds and optionally
 *                  a function to call when the timer expires
 *
 * PreCondition:    None
 *
 * Input:           usconds - timebase in usconds
 *                  method - ONESHOT or CONTINUOUS
 *                  fcn - pointer to function to call or NULL otherwise
 *                  attribs - pointer used as a function argument
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/

void SetupTimer(dword useconds, TIMERMETHOD method ,void (*fcn)(void *), void * attribs)
{   
    // Turn timers off
    T5CON = 0;  
    T4CON = 0;

    // Set it in 32bit mode
    T4CONbits.T32 = 1;
        
    // setup the function
    gblTimerEvent = fcn;
    gblTimerEventAttribs = attribs;
    gblTimerMethod = method;

    // Convert it to cycles
    if(useconds > 1000000)
        useconds = useconds/100*__NORM_US_FCY;      
    else
        useconds = useconds*__NORM_US_FCY/100;      
    
    PR5 = ((word *)(&useconds))[1]; // high byte
    PR4 = ((word *)(&useconds))[0]; // low byte
    
    // clear out the interrupt status bit
    _T5IF = 0;
    
    // lowest priority
    _T5IP = 0x01;                       // INTERRUPT PRIORITY LEVEL
    
    // Enable the interrupt
    _T5IE = 1;
}

/******************************************************************************
 * Function:        void SetupPeriodicTimer(void)
 *
 * Overview:        Sets up Timer2 for any periodic bookkeeping tasks
 *
 * PreCondition:    None
 *
 * Input:           None.
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/

void SetupPeriodicTimer(void)
{   
    // Turn TIMER2 off, set with 256 prescaler
    T2CON = 0x0030;
            
    // Clear TIMER2 interrupt flag
    _T2IF = 0;
    
    // Setup TIMER2 to 333ms
    PR2 = 0.333*(__FCY/mUL(256));       
    
    // Set TIMER2 interrupt priority level
    _T2IP = 0x02;
    
    // Enable TIMER2 interrupt
    _T2IE = 1;
}

/******************************************************************************
 * Function:        void StartPeriodicTimer(void)
 *
 * Overview:        Start Timer 2.
 *
 * PreCondition:    Timer has been setup with SetupPeriodicTimer()
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    Timer interrupt _T2Interrupt
 *
 * Note:            None
 *****************************************************************************/

void StartPeriodicTimer(void)
{
    T2CONbits.TON = 1;
}


/******************************************************************************
 * Function:        void GoTimer(void)
 *
 * Overview:        Start period timer.
 *
 * PreCondition:    Timer has been setup with SetupTimer()
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    Timer interrupt _T5Interrupt
 *
 * Note:            None
 *****************************************************************************/
void GoTimer(void)
{
    TMR5 = 0;
    TMR4 = 0;   
    _T5IF = 0;
    T4CONbits.TON = 1;      
}

/******************************************************************************
 * Function:        void StopTimer(void)
 *
 * Overview:        Stop period timer.
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
void StopTimer(void)
{
    T4CONbits.TON = 0;  
}

/******************************************************************************
 * Function:        void Delayus(dword delay)
 *
 * Overview:        Delay for the specified number of useconds
 *
 * PreCondition:    None
 *
 * Input:           delay - useconds
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
void Delayus(dword delay)
{   
    if(delay > 0)
    {
        StopTimer(); // just incase
        
        if(delay > DELAYOFFSET)
            delay -= DELAYOFFSET;
            
        SetupTimer(delay, ONESHOT, NULL, NULL);
        
        // stop the interupt
        _T5IE = 0;
        
        // run it
        GoTimer();
        
        while(!_T5IF){;} // waiting loop here
        
        _T5IF = 0;
    }
}

    const char DEFAULT_SERIAL_NUMBER[]         = "DEFAULT_PK3 ";

/******************************************************************************
 * Function:        bool LoadSerialNumber(void)
 *
 * Overview:        Loads serial number stored in Serial EE
 *
 * PreCondition:    Serial number has been stored during production
 *
 * Input:           None
 *
 * Output:          Success/Fail
 *
 * Side Effects:    gblMainSer will contain the serial number
 *
 * Note:            None
 *****************************************************************************/
bool LoadSerialNumber(void)
{
    bool status;
    
    memset(gblMainSer,0x0,sizeof(gblMainSer));
    
    status = GetSerialNumber((char *)gblMainSer);           
    
    return(status); 
}

/******************************************************************************
 * Function:        bool GetSerialNumber(char *Serial)
 *
 * Overview:        Loads serial number stored in Serial EE
 *
 * PreCondition:    Serial number has been stored during production
 *
 * Input:           Serial - pointer to serial location to fill
 *
 * Output:          fills Serial with serial number
 *
 * Side Effects:    gblMainSer will contain the serial number
 *
 * Note:            None
 *****************************************************************************/
bool GetSerialNumber(char *Serial)
{
    bool status = TRUE;
    unsigned char signconversion;
    
    status = SeeStructRead(mbr.Parameters.SerialNumber,Serial);
    
    signconversion = (unsigned char)Serial[0];
    
    // Make sure its good           
    if(signconversion == 0xFF || signconversion == 0 || !status)
    {
        memcpy(gblMainSer,DEFAULT_SERIAL_NUMBER,sizeof(DEFAULT_SERIAL_NUMBER));                     
    }   
                
    return(status);
}


/******************************************************************************
 * Function:        bool LoadToolName(void)
 *
 * Overview:        Loads Unit ID/Friendly Name stored in Serial EE
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          Success/Fail
 *
 * Side Effects:    gblToolName will contain the tool name
 *
 * Note:            None
 *****************************************************************************/
bool LoadToolName(void)
{
	bool status;
	
	memset(gblToolName,0x0,sizeof(gblToolName));
	
	status = GetToolName((char *)gblToolName);           
	
	return(status); 
}

/******************************************************************************
 * Function:        bool GetToolName(char *ToolName)
 *
 * Overview:        Loads Unit ID/Friendly Name stored in Serial EE
 *
 * PreCondition:    None
 *
 * Input:           ToolName - pointer to tool name location to fill
 *
 * Output:          Success/Fail
 *
 * Side Effects:    gblToolName will contain the tool name
 *
 * Note:            None
 *****************************************************************************/
bool GetToolName(char *Name)
{
	bool status = TRUE;
	unsigned char validateName;
	
	status = SeeStructRead(mbr.Parameters.ToolName,Name);
	
	validateName = (unsigned char)Name[0];
	
	// Make sure its got a valid tool name           
	if(validateName != 0x23 || !status) // '#' first byte is always ASCII pound to indicate valid Tool Name
	{
		status = FALSE;                     
	}   
	           
	return(status);
}


/******************************************************************************
 * Function:        void WriteLatch(WORD addrLo, WORD dataHi, WORD dataLo)
 *
 * Overview:        Writes programming latch using TBLWT
 *
 * PreCondition:    None
 *
 * Input:           addLo  - low Address of programming latch
 *                  dataLo - Data low
 *                  dataHi - Data low
 *
 * Output:          None
 *
 * Side Effects:    The latch specified by addr and TBLPAG is written
 *
 * Note:            None
 *****************************************************************************/
void WriteLatch(WORD addrLo, WORD dataHi, WORD dataLo)
{
    __builtin_tblwtl(addrLo,dataLo);
    __builtin_tblwth(addrLo,dataHi);
}   

/******************************************************************************
 * Function:        void WriteRow(void)
 *
 * Overview:        Initiates programming operation for one row
 *
 * PreCondition:    Programming latches for the row have been written
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    A row of flash memory is programmed
 *
 * Note:            None
 *****************************************************************************/
void WriteRow(void)
{
    NVMCON = PM_ROW_WRITE;

    __builtin_write_NVM();
    while(NVMCONbits.WR == 1)
        ;
        
    HASSERT(NVMCONbits.WRERR == 0);        
}   

/******************************************************************************
 * Function:        void EraseSection(DWORD address)
 *
 * Overview:        Initiates an Erase operation for a page 
 *
 * PreCondition:    None
 *
 * Input:           address - address of page to be erased
 *
 * Output:          None
 *
 * Side Effects:    A page of flash memory is erased
 *
 * Note:            None
 *****************************************************************************/
void EraseSection(DWORD address)
{
    WORD page;
    WORD addrLo;
    WORD temp;  
    
    DISABLE_SYSTEM_INTERRUPT

    page = address >> 16;
    addrLo = (WORD)address;

    temp = TBLPAG;
    TBLPAG = page;

    NVMCON = PM_PAGE_ERASE;

    __builtin_tblwtl(addrLo,addrLo);
    __builtin_write_NVM();
    while(NVMCONbits.WR == 1);

    HASSERT(NVMCONbits.WRERR == 0);

    TBLPAG = temp;
    
    RESTORE_SYSTEM_INTERRUPT
}

/******************************************************************************
 * Function:        void ProgramOneRow(unsigned char *buffer, DWORD address)
 *
 * Overview:        Programs a row of data stored in buffer starting from 
 *                  address
 *
 * PreCondition:    Data has been stored in memory block pointed to by buffer
 *
 * Input:           buffer  - pointer to start of data to be programmed
 *                  address - target address in flash memory to be programmed
 *
 * Output:          None
 *
 * Side Effects:    A row of flash memory is programmed
 *
 * Note:            None
 *****************************************************************************/
void ProgramOneRow(BYTE *buffer, DWORD address)
{
    WORD i;
    WORD temp;  
    WORD page;
    WORD addrLo;

    WORD dataHi;
    WORD dataLo;

    DISABLE_SYSTEM_INTERRUPT
    
    page   = address >> 16;
    addrLo = (WORD)address;

    temp = TBLPAG;
    TBLPAG = page;
    
    // 1 Load latches
    // The passed buffer MUST be ROWDATASIZE bytes long
    // We break it up into ROWDATASIZE/3 latch writes
    for (i = 0 ; i <  ROWDATASIZE/3 ; ++i)
    {
        // Write one latch
        dataLo =  *buffer++;
        dataLo |= ((WORD)(*buffer++) << 8);
        dataHi = *buffer++;
        WriteLatch(addrLo, dataHi, dataLo);
        addrLo += 2;
            
    }
    // 2 Write latches
    WriteRow();
    TBLPAG = temp;
    
    RESTORE_SYSTEM_INTERRUPT
}

/******************************************************************************
 * Function:        void WriteAPBootloaderSwitch(void)
 *
 * Overview:        Program a value in PK3 flash so that it jumps to bootloader
 *                  on the next reset. This is to allow MPLAB to connect to the
 *                  PICkit 3 and upgrade it.
 *
 * PreCondition:    An MPLAB compatible bootloader is programmed at 0xF000
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    A row of flash memory starting at 0x2A000 is programmed
 *
 * Note:            None
 *****************************************************************************/
void WriteAPBootloaderSwitch(void)
{
    EraseSection(PK3_BL_SWITCH_ADDRESS);

    // Mark the FLASH as to jump to BOOT after the next reset
    gOneRow[0] =  (unsigned char)PK3_JUMP_TO_BOOT;
    gOneRow[1] =  (unsigned char)(PK3_JUMP_TO_BOOT >> 8);
    gOneRow[2] =  0;
    ProgramOneRow(gOneRow,PK3_BL_OS_SWITCH_ADDRESS);
}

/******************************************************************************
 * Function:        bool TestBootloader(void)
 *
 * Overview:        Ensures that there is a valid bootloader programmed at 
 *                  0xF000 with the appropriate values in header and footer
 *
 * PreCondition:    Programming latches for the row have been written
 *
 * Input:           An MPLAB compatible bootloader is programmed at 0xF000
 *
 * Output:          Success/Fail
 *
 * Side Effects:    gblBootloaderVersion is written
 *
 * Note:            None
 *****************************************************************************/
bool TestBootloader(void)
{
    int     saved_PSVPAG;
    BL_HEADER   *BL_Header;
    WORD    test;   
    bool    status = FALSE;
    SYS_VER  testver = MIN_BL_VER;
    
    // first thing is to see if its out there
    saved_PSVPAG = PSVPAG;
    PSVPAG = mUL(BOOTLOADER_START) >> 15;
 
    BL_Header = (BL_HEADER*)(0x8000 + (mUL(BOOTLOADER_START) & 0x7FFF));
    test = BL_Header->MagicKey;

    // Check to see that the Bootloader is programmed in
    if(test == BOOTLOADERKEY)
    {
        // copy over the version
        memcpy(&gblBootloaderVersion, &BL_Header->Version,sizeof(SYS_VER));

        // Check to see that the bootloader is new enough
        if((status = VerTest(BL_Header->Version, testver)) == TRUE)
        {
            status = TestBLFooter();
        }
        else
            status = FALSE;
    }
    else
        status = FALSE;
        
    // Restore PSVPAG
    PSVPAG = saved_PSVPAG;
    
    return(status);
}

/******************************************************************************
 * Function:        bool TestBLFooter(void)
 *
 * Overview:        Part of bootloader validation.
 *
 * PreCondition:    An MPLAB compatible bootloader is programmed at 0xF000
 *
 * Input:           None
 *
 * Output:          Success/Fail
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
bool TestBLFooter(void)
{
    bool status = FALSE;
    WORD test;
    BL_FOOTER *BL_Footer;
    SYS_VER footversion;
    
    PSVPAG = mUL(BOOTLOADER_END) >> 15;
 
    BL_Footer = (BL_FOOTER *)(0x8000 + (mUL(BOOTLOADER_END) & 0x7FFF));
    test = BL_Footer->MagicKey;
    
    // Check to see that the Bootloader is programmed in
    if(test == BOOTLOADERKEY)
    {
        // now get the version
        memcpy(&footversion, &BL_Footer->Version,sizeof(SYS_VER));
        
        if((footversion.MAJOR == gblBootloaderVersion.MAJOR) && \
            (footversion.MINOR == gblBootloaderVersion.MINOR) && \
            (footversion.REV == gblBootloaderVersion.REV) && \
            (footversion.TYPE == gblBootloaderVersion.TYPE))
            status = TRUE;
    }
    
    return(status);
}

/******************************************************************************
 * Function:        bool VerTest(SYS_VER ThisVersion, SYS_VER TestVersion)
 *
 * Overview:        Compares OS versions
 *
 * PreCondition:    None
 *
 * Input:           ThisVersion - target version
 *                  TestVersion - test version
 *
 * Output:          True - ThisVersion is equal or later than TestVersion
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
bool VerTest(SYS_VER ThisVersion, SYS_VER TestVersion)
{
    bool status = FALSE;
    
    if((ThisVersion.MAJOR > TestVersion.MAJOR) || \
    ((ThisVersion.MAJOR == TestVersion.MAJOR) && (ThisVersion.MINOR > TestVersion.MINOR)) || \
    ((ThisVersion.MAJOR == TestVersion.MAJOR) && (ThisVersion.MINOR == TestVersion.MINOR) && (ThisVersion.REV >= TestVersion.REV)))
    {
        // get the version, 0x0 is a don't care
        if(ThisVersion.TYPE >= TestVersion.TYPE || TestVersion.TYPE == 0x0)
            status = TRUE;
    }

    return(status);
}
