/*********************************************************************
 *
 * Serial SPI functions
 *
 *********************************************************************
 * FileName:            spi_serial_flash.c
 * Dependencies:        pickit3.h spieeprom.h spi_serial_flash.h
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
#include "spieeprom.h"
#include "spi_serial_flash.h"


//********************************************************************
// Local Functions
//********************************************************************
BOOL SelectEEProm(BYTE DeviceAddress, BOOL Enable, BOOL EnableWrite);
BOOL EEPROMReadStatus(BYTE DeviceAddress, pEEPROMSTATUS eestatus );
BOOL WaitTilEEDone(BYTE DeviceAddress, DWORD timeout);
unsigned char writeSPI2( unsigned char i );
void SPI2INTInit(BOOL FastMode );

/******************************************************************************
 * Function:        void SPI2INTInit(BOOL FastMode )
 *
 * Overview:         This function is used for initializing the SPI module.
 *
 * PreCondition:    TRIS bit of Slave Chip Select pin (if any used) should
 *                  be made output.
 *
 * Input:           FastMode - TRUE=16 MHz, FALSE=4 MHz
 *
 * Output:          None
 *
 * Side Effects:    
 *
 * Note:            None
 *****************************************************************************/
void SPI2INTInit(BOOL FastMode )
{
    SPI2STAT = 0;  // turn it off
    IFS2bits.SPI2IF = 0;    // clear interrupt flag
    IEC2bits.SPI2IE = 0;    // disable interrupt
    
    // Enable the peripheral @ 16Mhz or 4Mhz
    if(FastMode)
        SPI2CON1 = SPI_MASTER;      // 16Mhz
    else    
        SPI2CON1 = SPI_MASTER_SLOW;  // 4Mhz

    SPI2STAT = SPI_ENABLE;      
} // SPI2INTInit

/******************************************************************************
 * Function:        unsigned char writeSPI2( unsigned char i )
 *
 * Overview:        send one byte of data and receive one back at the same time
 *
 * PreCondition:    SPI module has been initialized
 *
 * Input:           i - input byte to be sent
 *
 * Output:          received byte
 *
 * Side Effects:    
 *
 * Note:            None
 *****************************************************************************/
unsigned char writeSPI2( unsigned char i )
{
    SPI2BUF = i;                    // write to buffer for TX
    while(!SPI2STATbits.SPIRBF);    // wait for transfer to complete
    return SPI2BUF;                 // read the received value
}//writeSPI2

/******************************************************************************
 * Function:        void InitSEEs(void)
 *
 * Overview:        Initialize SEEs. Drive pins to proper state.
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
void InitSEEs(void)
{
    // Set CS direction control
    tris_UTIL1_nCS  = OUTPUT;
    pin_UTIL1_nCS   = 1;

    tris_UTIL2_nCS  = OUTPUT;
    pin_UTIL2_nCS   = 1;

    // Disable hardware write protect (make both writable)
    tris_UTIL1_nWP  = OUTPUT;
    pin_UTIL1_nWP   = 1;

    tris_UTIL2_nWP  = OUTPUT;
    pin_UTIL2_nWP   = 1;        // Disable hardware write protect
}

/******************************************************************************
 * Function:        BOOL ReadSerialEEBuffer(DWORD Address, BYTE *Buf, DWORD Size)
 *
 * Overview:        Reads a chunk of memory from Serial EE
 *
 * PreCondition:    SPI module must be configured to operate with EEPROM.
 *
 * Input:           Address - starting address (DWORD)
 *                  Buf - pointer to destination buffer (BYTE*)
 *                  Size - size in bytes (DWORD)
 *
 * Output:          Success/Fail
 *                  Buf is filled with data contents
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
BOOL ReadSerialEEBuffer(DWORD Address, BYTE *Buf, DWORD Size)
{
    BOOL    status = TRUE;
    BYTE    DeviceAddress;

    // correctly select the Serial EEprom
    if(Address >= CODE_SPACE_SEE_ADDRESS)
        DeviceAddress = CODE_SEE_ADDR;
    else
        DeviceAddress = PARMS_SEE_ADDR;     // Serial EEPROM
        
    // Select the correct SEE
    if((status = SelectEEProm(DeviceAddress, TRUE, FALSE)))
    {
        // Send 'Read Array' command
        writeSPI2(EEPROM_CMD_READ);

        // Send address
        // Code Image requires address bits A23:A0 (24 bits)
        // Serial EEPROM requires A15:A0 (16 bits)
        if(DeviceAddress == CODE_SEE_ADDR)
        {
            writeSPI2(upper(Address));      // Code Image only
        }
        writeSPI2(high(Address));           // Code Image & Serial EEPROM
        writeSPI2(low(Address));            // Code Image & Serial EEPROM

        //
        while(Size--)
            *Buf++ = writeSPI2(0x0);

        // Desselect EEPROMs
        SelectEEProm(DeviceAddress, FALSE, FALSE);
    }

    return status;
}

/******************************************************************************
 * Function:        BOOL WriteSerialEEBuffer(DWORD StartAddress, BYTE *Buf, DWORD Size)
 *
 * Overview:        Writes a chunk of memory from buffer to Serial EE
 *
 * PreCondition:    SPI module must be configured to operate with EEPROM.
 *
 * Input:           Address - starting address (DWORD)
 *                  Buf - pointer to source buffer (BYTE*)
 *                  Size - size in bytes (DWORD)
 *
 * Output:          Success/Fail
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
BOOL WriteSerialEEBuffer(DWORD StartAddress, BYTE *Buf, DWORD Size)
{
    BOOL    status=TRUE;
    WORD    PageSize;       // Page size
    WORD    BlockSize, index;
    BOOL    parmsSEE;
    BOOL    firstrun=TRUE;
    DWORD   timeout;
    DWORD   CurrentAddress;
    BYTE    DeviceAddress;

    // Select the parameters according to the device
    if(StartAddress >= CODE_SPACE_SEE_ADDRESS)
    {
        DeviceAddress = CODE_SEE_ADDR;
        PageSize=256;       // 256 byte page
        parmsSEE = FALSE;   // Code Image
        timeout = 1000;     // 10ms timeout
    }
    else
    {
        DeviceAddress = PARMS_SEE_ADDR;     // Serial EEPROM
        PageSize = 64;      // 64 byte page
        parmsSEE = TRUE;    // Serial EEPROM
        timeout = 1000;     // 10ms timeout
    }
    
    CurrentAddress=StartAddress;

    // Write until done
    while(status && Size)
    {
        // Adjust page size if needed
        if(Size >= PageSize)
            BlockSize = PageSize;
        else
            BlockSize = Size;

        // Now check for mod issues. Make sure we are on the same page
        // TODO: optimize - move to delay and poll
        if( ((CurrentAddress + BlockSize) / PageSize) != (CurrentAddress / PageSize))
        {
            // Adjust block size
            BlockSize -= ((CurrentAddress+BlockSize) % PageSize);
        }

        // Just a little optimization
        if(firstrun)
            firstrun=FALSE;     // Don't wait the first time
        else
        {
            // Poll for done from last operation, timesout after preset time
            status = WaitTilEEDone(DeviceAddress, timeout);
        }

        // Enable the correct device and turn on writes
        if(status && (status = SelectEEProm(DeviceAddress, TRUE, TRUE)))
        {
            // Send 'Byte/Page Program' command
            writeSPI2(EEPROM_CMD_WRITE);

            // Write the address
            if(!parmsSEE)
                writeSPI2(upper(CurrentAddress));   // Code Image only

            writeSPI2(high(CurrentAddress));        // Code Image & Serial EEPROM
            writeSPI2(low(CurrentAddress));         // Code Image & Serial EEPROM

            // Write the data for the block
            for(index=0; index<BlockSize; index++)
                writeSPI2(*Buf++);

            // Disabling starts the write, no worries, the nWP has already gotten latched
            SelectEEProm(DeviceAddress, FALSE, FALSE);

            // Adjust size and address for next loop
            Size -= BlockSize;
            CurrentAddress += BlockSize;
        }
    }

    // Last write
    if(status)
        status = WaitTilEEDone(DeviceAddress, timeout);

    return status;
}

/******************************************************************************
 * Function:        BOOL SelectEEProm(BYTE DeviceAddress, BOOL EnableDevice, BOOL EnableWrite)
 *
 * Overview:        Selects and sets up the correct SEE for the operation.
 *
 * PreCondition:    SPI module must be configured to operate with EEPROM.
 *
 * Input:           DeviceAddress - device address
 *                  EnableDevice - TRUE=Enable, FALSE=Disable
 *                  EnableWrite - TRUE=Enable, FALSE=Disable
 *
 * Output:          Success/Fail
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
BOOL SelectEEProm(BYTE DeviceAddress, BOOL EnableDevice, BOOL EnableWrite)
{
    BOOL status = TRUE;
    BOOL CodeSEE=FALSE;

    // Serial EEPROM or Code Image
    if(DeviceAddress == CODE_SEE_ADDR)
        CodeSEE = TRUE;
    else if(DeviceAddress == PARMS_SEE_ADDR)
        CodeSEE = FALSE;
    else
        status = FALSE;     // Fails if neither

    // Default state for Serial EEPROM
    pin_UTIL1_nCS = 1;      // CS disabled
    pin_UTIL1_nWP = 1;      // Disable hardware write protect

    // Default state for Code Image
    pin_UTIL2_nCS = 1;      // CS disabled
    pin_UTIL2_nWP = 1;      // Disable hardware write protect

    if(status)
    {
        // Chip select the correct one
        if(EnableDevice)
        {
            // Code Image
            if(CodeSEE)
            {
                SPI2INTInit(TRUE);      // Setup the SPI fast

                pin_UTIL2_nCS = 0;      // CS Active

                if(EnableWrite)
                    pin_UTIL2_nWP = 1;  // Disable hardware write protect
            }
            // Serial EEPROM
            else
            {
                SPI2INTInit(FALSE);     // Setup the SPI slow

                pin_UTIL1_nCS = 0;      // CS Active

                if(EnableWrite)
                    pin_UTIL1_nWP = 1;  // Disable hardware write protect
            }

            // Write the Write enable command if so
            if(EnableWrite)
            {
                // Send 'Write Enable' command
                writeSPI2(EEPROM_CMD_WREN);

                // Pulse CS to set WREN set
                if(CodeSEE)
                {
                    // wait atleast 50ns, instuctions will be long enough
                    pin_UTIL2_nCS = 1;  // disabled
                    // wait atleast 50ns, 2 NOPS @ 33ns our speed
                    NOP();
                    NOP();
                    pin_UTIL2_nCS = 0;  // Active
                }
                else
                {
                    // wait atleast 50ns, instuctions will be long enough
                    pin_UTIL1_nCS = 1;  // disabled
                    // wait atleast 50ns, 2 NOPS @ 33ns our speed
                    NOP();
                    NOP();
                    pin_UTIL1_nCS = 0;  // Active
                }
            } // enable writes

        } // enable device

    } // status check

    return(status);
}

/******************************************************************************
 * Function:        BOOL EEPROMReadStatus(BYTE DeviceAddress, pEEPROMSTATUS eestatus )
 *
 * Overview:        This function reads status register from EEPROM.
 *
 * PreCondition:    SPI module must be configured to operate with EEPROM.
 *
 * Input:           DeviceAddress - device address
 *                  pEEPROMSTATUS - pointer to status structure
 *
 * Output:          Success/Fail
 *                  fills status structure
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
BOOL EEPROMReadStatus(BYTE DeviceAddress, pEEPROMSTATUS eestatus )
{
    BOOL ret=FALSE;

    // select the correct Eeprom
    if(SelectEEProm(DeviceAddress, TRUE, FALSE))
    {
        // Send 'Read Status Register' command
        writeSPI2(EEPROM_CMD_RDSR);

        // Read status register
        eestatus->Char = writeSPI2(0);

        ret = TRUE;

        // Deselect device
        SelectEEProm(DeviceAddress, FALSE, FALSE);
    }

    return ret;
}

/******************************************************************************
 * Function:        BOOL WaitTilEEDone(BYTE DeviceAddress, DWORD timeout)
 *
 * Overview:        Wait till EEPROM is ready or timeout
 *
 * PreCondition:    SPI module must be configured to operate with EEPROM.
 *
 * Input:           DeviceAddress - device address
 *                  timeout - how many times to loop
 *
 * Output:          Success/Fail
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
BOOL WaitTilEEDone(BYTE DeviceAddress, DWORD timeout)
{
    BOOL            exit = FALSE;
    EEPROMSTATUS    eestatus;
    BOOL            status = TRUE;

    while(!exit && timeout-- && status)
    {
        // Read the Status register
        status = EEPROMReadStatus(DeviceAddress, &eestatus);

        // Check for Write-in-progress cleared
        if(!eestatus.Bits.WIP)
             exit = TRUE;
    }

    if(!timeout)
        status = FALSE;

    return status;
}
