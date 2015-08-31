/*********************************************************************
 *
 * Power Supply Code
 *
 *********************************************************************
 * FileName:            powerPK3.c
 * Dependencies:        pickit3.h string.h math.h
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
#include <string.h>
#include <math.h>

//********************************************************************
// Globals
//********************************************************************
POWER_INFO gPowerInfo;
const char  pp_block[] = {1};           // just for reseting the PSV
SYSVOLTAGES gblSysVoltages;

//********************************************************************
// To simplify understanding the code in this file, keep in mind that 
// there are two kinds of 'counts': 
//  - Values read from the ADC1BUF0,1 
//  - Values plugged into the OC1RS and OC2RS (which are the duty cycle 
//    values for the PWMs). 
//
// The macros defined to go from voltages to counts include the kind 
// of count: so, ADC_COUNT and PWM_COUNT is included in the name.
//********************************************************************

//********************************************************************
// Functions
//********************************************************************

/******************************************************************************
 * Function:        void ControlUSBPower(bool on)
 *
 * Overview:        Turns USB power On/Off
 *
 * PreCondition:    successfully enumerated and granted the use of enough power
 *
 * Input:           on/off
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
void ControlUSBPower(bool on)
{
    if (on)
        pin_5V_USB_ENABLE   = 1;
    else
        pin_5V_USB_ENABLE   = 0;
        
    tris_5V_USB_ENABLE  = OUTPUT;
}

/******************************************************************************
 * Function:        void DelayForSampleAndConversionTime(void)
 *
 * Overview:        Sampling and conversion delay
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
void DelayForSampleAndConversionTime(void)
{
    // Big Tad is 256Tcy.
    // We need to wait for 12 TADs
    // 256*12 = 3072 nops
    __asm__ volatile("repeat #3071");
    __asm__ volatile("nop");
}
    
/******************************************************************************
 * Function:        void CalibrateADC(void)
 *
 * Overview:        Calibrate the ADC. The PK3 has a MCP1525 on board. This 
 *                  chip provides a known 2.5 reference regardless of the 
 *                  variations on the 5V USB. In this function we calculate the
 *                  v/count value for the ADC. Here we KNOW that 2.5 volts is
 *                  present at AN0. We read it and convert the count into 
 *                  mvolts per count. 
 
 *
 * PreCondition:    AD1PCFG has been set with AN0 as analog. AVdd and AVss are 
 *                  connected to 3.3 (regulated) and ground. So, the max we can 
 *                  see using the ADC is 3.3 volts. All measurements are scaled
 *                  via voltage dividers to fit in within the 3.3 volt range.
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    Calling this function re-programs the ADC. So, make sure 
 *                  you call this function before the normal ADC setup to read 
 *                  Vpp and Vdd.
 *
 * Note:            None
 *****************************************************************************/

void CalibrateADC(void)
{
    // Set ADC to read VREF_25 (connected to AN0)
    int AverageADCValue;
    int i;
    _AD1IE = 0;
    _AD1IF = 0;
    
    AD1CON1 = 0x0000;   // SAMP bit = 0 ends sampling, select integer format
                        // and starts converting
    AD1CHS0 = 0x0000;   // Connect AN0 as CH0 input
    AD1CSSL = 0;
    AD1CON3 = 256;      // Manual Sample, Tad = 256 Tcy
    AD1CON2 = 0;        // AVdd and AVss used as Vr+ and Vr-
    AD1CON1bits.ADON = 1; // turn ADC ON
    
    // Delay for sample and conversion time.
    DelayForSampleAndConversionTime();   

    // Gather 16 points and find average
    for( i = 0, AverageADCValue = 0; i < 16 ; ++i)
    {
        AD1CON1bits.SAMP = 1;           // start sampling...

        // Delay for sample and conversion time.
        DelayForSampleAndConversionTime();
                                        // before starting conversion.
        AD1CON1bits.SAMP = 0;           // start Converting
        while (!AD1CON1bits.DONE);      // conversion done?
        AverageADCValue += ADC1BUF0;    // yes then get ADC value
    }    

    // 2.5Volts = contant * AverageADCValue.
    // contant is volts per count.
    if (AverageADCValue != 0)
        gPowerInfo.VoltsPerADCCount = 2.5/((float)AverageADCValue/16.0);
    // else TODO handle error!
    AD1CON1 = 0;        // Shut off ADC
}

/******************************************************************************
 * Function:        void SetupADCToReadVddAndVpp(void)
 *
 * Overview:        Setup Analog to Digital converter to read Vdd and Vpp.
 *                  The original PK2 code used an timer interrupt (every 
 *                  125usecs) to read Vdd (the ADC was set up to read Vdd 
 *                  before the interrupt and the ADC complete interrupt was 
 *                  diabled). So, when timer1 happened, the ADRESH contained 
 *                  the (left justified) 8 MSBs of the Vdd. Then at the end of 
 *                  the timer 1 interrupt we set the ADC to read Vpp and enabled
 *                  the interrupt on completion. When the ADC interrupted, the
 *                  Vpp value was available in ADRESH. So effectively, every 
 *                  125 usecs we got both a sample of Vdd and Vpp. 
 *
 *                  In PK3 to avoid using an extra timer for this we use the 
 *                  alternating input selection. We select Vdd as one channel 
 *                  and then the ADC will switch to measuring the second 
 *                  channel (Vpp). Also, to mimic the 125 usec interval in PK2,
 *                  we setup the ADC to give us an interrupt at the end of the 
 *                  sampling of Vdd and Vpp. Since the ADC is fast, we ask the
 *                  ADC to also sample more than once each voltage. 
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
void SetupADCToReadVddAndVpp(void)
{
    AD1CON1 = 0x0000;    // SAMP bit = 0 ends sampling, select integer format
                         // and starts converting
                         
    AD1CHS0 = 0x0302;    // AN2 (Vdd)->channel A, AN3 (Vpp)->channel B, use AVss as Vr-
    AD1CON2 = 0x0005;    // Select AVdd and AVss for Vr+ and Vr-, no channel scan on mux A,
                         // use a single 16 word buffer, interrupt after 2 conversions (one for
                         // Vdd and one for Vpp), use alternate sampling.

    // We want to convert 2 samples every 125 usecs
    // So, for each sample:
    // 62.5usecs = 12TAD + SAMP*TAD. SAMP is the AD1CON3<SAMC4:SAMC0> bits.
    // And we have that TAD = ADCS*TCY where ADCS is the AD1CON3<ADCS7:ADCS0> bits.
    // 62.5usecs = 12*ADCS*TCY + SAMP*ADCS*TCY
    // 62.5usecs = ADCS*TCY*(12 + SAMP)
    // If we fix the SAMP to say 16 (we let ADCS be the 'variable' since it has a bigger
    // range). And Tcy = 62.5nsecs (1/16MIPS)
    // 62.5usecs = ADCS*62.5nsecs*28
    // Then ADCS = 62.5e-6/(62.5e-9*28) = 36 = 0x24
    AD1CON3 = 0x1024;    // Autosample every 16 TADs and a TAD is 36*TCy
    AD1CON1 = 0x00E0;    // Integer format, use internal counter to finish sampling, 
    _AD1IF  = 0;
    _AD1IE  = 0;         // No interrupts until SetVdd or SetVpp are called
    _AD1IP  = 6;         // High priority to avoid damaging HW  // INTERRUPT PRIORITY LEVEL 
    AD1CON1bits.ASAM = 1; //automatically start sampling after the last conversion)                    
    AD1CON1bits.ADON = 1;// turn ADC ON
    
    // Give the A2D time to boot up
    Delayus(200);
}

/******************************************************************************
 * Function:        unsigned int SafelyGetVddCount(void)
 *
 * Overview:        Access gPowerInfo safely to get VDD count.
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
unsigned int SafelyGetVddCount(void)
{
    unsigned int res;
    _AD1IE = 0;     // stop A2D ints while we access gPowerInfo
    res = gPowerInfo.VddADCCounts;
    _AD1IE = 1;     // re-start A2D ints
    return res;
}

/******************************************************************************
 * Function:        unsigned int SafelyGetVppCount(void)
 *
 * Overview:        Access gPowerInfo safely to get VPP count.
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
unsigned int SafelyGetVppCount(void)
{
    unsigned int res;
    _AD1IE = 0;     // stop A2D ints while we access gPowerInfo
    res = gPowerInfo.VppADCCounts;
    _AD1IE = 1;     // re-start A2D ints
    return res;
}

/******************************************************************************
 * Function:        word GetClosestCount(float v)
 *
 * Overview:        Get as close to a 0.125 count as possible.
 *
 * PreCondition:    None
 *
 * Input:           v - voltage in volts
 *
 * Output:          tickslow
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
word GetClosestCount(float v)
{
    float diffToLow;
    word ticksLow = v /0.125;
    
    diffToLow = v - (ticksLow*0.125);
    if (diffToLow > (0.125/2))
        ticksLow++;    
        
    return ticksLow;
}

/******************************************************************************
 * Function:        float GetVddInVolts(void)
 *
 * Overview:        Retrieves VDD ADC reading and converts to volts.
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          res - ADC reading in volts
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
float GetVddInVolts(void)
{
    float res;
    res = FROM_ADC_COUNTS_TO_VDD_VOLTAGE(SafelyGetVddCount());
    return res;
}


/******************************************************************************
 * Function:        float GetVppInVolts(void)
 *
 * Overview:        Retrieves VPP ADC reading and converts to volts.
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          res - ADC reading in volts
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
float GetVppInVolts(void)
{
    float res;
    res = FROM_ADC_COUNTS_TO_VPP_VOLTAGE(SafelyGetVppCount());
    return res;
}


/******************************************************************************
 * Function:        word GetVddInCounts(void)
 *
 * Overview:        Retrieves VDD ADC reading and converts to counts.
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          reading in counts
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
word GetVddInCounts(void)
{
    return GetClosestCount(GetVddInVolts());
}


/******************************************************************************
 * Function:        word GetVppInCounts(void)
 *
 * Overview:        Retrieves VPP ADC reading and converts to counts.
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          reading in counts
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
word GetVppInCounts(void)
{
    return GetClosestCount(GetVppInVolts());
}

/******************************************************************************
 * Function:        void InitPowerTrain(void)
 *
 * Overview:        Initializes power train.
 *
 * PreCondition:    USB power is on
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
void InitPowerTrain(void)
{
    memset(&gPowerInfo, 0, sizeof(gPowerInfo));
    
    pin_LVP     = 0;
    pin_LVP_EN  = 0;
    tris_LVP    = OUTPUT;
    tris_LVP_EN = OUTPUT;
    
    // Set timer 3 for PWMs (Vdd and Vpp)
    // See comments on the definition of MAX_VALUE_PERIOD_FOR_PWM_TIMER
    T3CON = 0;
    PR3   = MAX_VALUE_PERIOD_FOR_PWM_TIMER;
    T3CONbits.TON = 1;
    
    // Calibrate analog to digital converter.
    CalibrateADC();
     
    // Setup Analog to Digital converter to read Vdd and Vpp
    SetupADCToReadVddAndVpp();
    gPowerInfo.Flag.TargetPowered = TRUE;
    gPowerInfo.CurPwrMode = NO_PWR_MODE;
    TurnVPP(OFF);
    SetVdd(DEFAULT_VDD);
    SetVpp(DEFAULT_VPP);
}

/******************************************************************************
 * Function:        void MySetPWMInCounts(unsigned char which, unsigned int counts)
 *
 * Overview:        Sets up the PWM for VDD or VPP.
 *
 * PreCondition:    None
 *
 * Input:           which - SELECT_VPP or SELECT_VDD
 *                  counts - desired voltage in PWM counts
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
void MySetPWMInCounts(unsigned char which, unsigned int counts)
{
    if (which == SELECT_VDD)
    {
        OC2CON1 = PWM_OFF;            // shut off peripheral
        OC2R    = 0;
        OC2RS   = counts;
        OC2CON2 = PWM_SET_FOR_TIMER3_INPUT;         
        OC2CON1 = PWM_SET_FOR_TIMER3_CENTER_PWM;
    }
    else
    {
        OC1CON1 = PWM_OFF;            // shut off peripheral
        OC1R    = 0;
        OC1RS   = counts;
        OC1CON2 = PWM_SET_FOR_TIMER3_INPUT;         
    }
}

/******************************************************************************
 * Function:        BOOL SetVpp(WORD tickvoltage)
 *
 * Overview:        Set VPP. Converts voltage to 0.125v steps and calls 
 *                  InternalSetVpp()
 *
 * PreCondition:    None
 *
 * Input:           tickvoltage - desired voltage in 0.125v counts or 0 to shut
 *                  down
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
BOOL SetVpp(WORD tickvoltage)
{
    BOOL res = FALSE;
    float vDesired;
    
    vDesired = tickvoltage*((float).125);
    
    // Internal Set Vpp
    InternalSetVpp(vDesired);

    res = TRUE;
            
    return res;
}

/******************************************************************************
 * Function:        BOOL InternalSetVpp(float voltage)
 *
 * Overview:        Set Vpp to desired voltage. This is the actual function 
 *                  that manipulates the power system. Called by SetVpp().
 *
 * PreCondition:    None
 *
 * Input:           voltage - desired voltage in volts
 *
 * Output:          Success/Fail
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
BOOL InternalSetVpp(float voltage)
{
    BOOL    res = TRUE;

    _AD1IE = 0;     // stop A2D ints while we access gPowerInfo
    _AD1IP  = 6;         // INTERRUPT PRIORITY LEVEL, very high
    
    gPowerInfo.Flag.VppIsOn = FALSE;
    gPowerInfo.Flag.VppIsSetup = FALSE;
    gPowerInfo.Flag.InternalMCLR = FALSE;
    
    if (voltage > MAX_VPP) 
    {
        voltage = MAX_VPP;
        res = FALSE;
    }
    
    if (voltage == 0.0)
    {
        OC1CON1 = PWM_OFF;
        
        if (gPowerInfo.Flag.VddIsOn)
        {
            // Re-start A/D ints
            _AD1IE = 1;     
        }
        
        // else return with ADC interrupts disabled
        return TRUE;
    }
    float VppErr;
    VppErr = voltage * 0.7;
    
    _AD1IE = 0;     // stop A2D ints
    
    
    // Calculate what values the ADC should be seeing at minimum
    gPowerInfo.VppADCCountsTooLow     = FROM_VPP_VOLTAGE_TO_ADC_COUNTS(VppErr);
    gPowerInfo.VppTooLowCounter       = 0;      
    
    // Do what PK2 does in user/pickit.c line 682 (caseSETVPP)
    
    // 1) Set PWM duty cycle (they call it set point) to %50
    gPowerInfo.VppPWMCountsSetPoint   = VPP_DUTY_CYCLE;
    
    // 2) Calculate what values the ADC should see at the most
    // The PK3 ignore the least 2 LSBs. It does a +1 on the counts. Since we do not
    // ignore the 2 LSBs, the equivalent here is + 4
    gPowerInfo.VppADCCountsUpperLimit = FROM_VPP_VOLTAGE_TO_ADC_COUNTS(voltage) + 4;
    
    // 3) Set the minumum value a bit lower than the max
    // The PK3 ignore the least 2 LSBs. It does a -2 on the counts. Since we do not
    // ignore the 2 LSBs, the equivalent here is - 8
    gPowerInfo.VppADCCountsLowerLimit  = gPowerInfo.VppADCCountsUpperLimit - 8;
    
    // We always set the duty cycle at %50 and let A2D interrupt (which is scheduled
    // by timer 2) control Vpp by shutting of the PWM or re-starting it.
    // This is the exact same way PK2 did it.
    
    // Set PWM in counts
    MySetPWMInCounts(SELECT_VPP, VPP_DUTY_CYCLE);
    //MyDelayMsecs(250);
    //gPowerInfo.Flag.VppIsOn = TRUE;
    gPowerInfo.Flag.VppIsSetup = TRUE;
    _AD1IE = 1;     // re-start A2D ints
    
    //Delayus(100000);      // what it takes for Vpp to rise
    return res;
    
}

/******************************************************************************
 * Function:        BOOL SetVdd(WORD tickvoltage)
 *
 * Overview:        Set VDD. Converts voltage to 0.125v steps and calls 
 *                  InternalSetVdd()
 *
 * PreCondition:    None
 *
 * Input:           tickvoltage - desired voltage in 0.125v counts or 0 to shut
 *                  down
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
BOOL SetVdd(WORD tickvoltage)
{
    BOOL res = FALSE;
    float vDesired, vRead;
    
    vDesired = tickvoltage*((float).125);
    
    // Internal Set Vdd
    InternalSetVdd(vDesired);
    vRead = GetVddInVolts();
    gblSysVoltages.VDD = gblSysVoltages.VDDTARGET = GetVddInCounts();

    if (fabsf(vRead-vDesired) < MAX_VDD_ERROR)
        res = TRUE;
    else
        Nop();
    return res;
}


/******************************************************************************
 * Function:        BOOL SetVddInVolts(float vDesired)
 *
 * Overview:        Set VDD to desired voltage
 *
 * PreCondition:    None
 *
 * Input:           vDesired - desired voltage in volts
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
BOOL SetVddInVolts(float vDesired)
{
    BOOL res = FALSE;
    float vRead;
    
    // Internal Set Vdd
    InternalSetVdd(vDesired);
    vRead = GetVddInVolts();
    if (fabsf(vRead-vDesired) < MAX_VDD_ERROR)
        res = TRUE;
    else
        Nop();
    return res;
}


/******************************************************************************
 * Function:        BOOL InternalSetVdd(float voltage)
 *
 * Overview:        Set Vpp to desired voltage. This is the actual function 
 *                  that manipulates the power system. Called by SetVdd().
 *
 * PreCondition:    None
 *
 * Input:           voltage - desired voltage in volts
 *
 * Output:          Success/Fail
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
BOOL InternalSetVdd(float voltage)
{
    BOOL    res = TRUE;
    
    // Stop A/D ints while we access gPowerInfo
    _AD1IE = 0;     
    gPowerInfo.Flag.VddIsOn = FALSE;
    
    if (voltage == 0.0)
    {
        OC2CON1 = PWM_OFF;
        return TRUE;        // return with ADC interrupts disabled
    }
    
    if (voltage > MAX_VDD)
    {
        voltage = MAX_VDD;
        res = FALSE;
    }
    // Determine VDD error threshold. This if is lifted from the PK2 DLL code that controls
    // the PK2 via a setVDD command. 
    float VddErr;
    VddErr = voltage * 0.85;
    if (VddErr > 4.12)
        VddErr = 4.12;      // limit high side threshold - actual voltage may be low due to diode drop.
   
    // Calculate what values the ADC should be seeing at minimum
    gPowerInfo.VddADCCountsTooLow = FROM_VDD_VOLTAGE_TO_ADC_COUNTS(VddErr);
    gPowerInfo.VddTooLowCounter = 0;;      
    
    // Set PWM in counts
    MySetPWMInCounts(SELECT_VDD, FROM_VDD_VOLTAGE_TO_PWM_COUNTS(voltage));
    
    gPowerInfo.Flag.VddIsOn = TRUE;
    _AD1IE = 1;     // re-start A2D ints
    Delayus(10000);      // what it takes for Vpp to rise
    return res;
    
}

//********************************************************************
// Interrupt Service Routine
//********************************************************************

// A2D will interrupt us every 125 usecs with 2 samples in buf0 and buf1. Each sample
// represents Vdd and Vpp
void __attribute__((__interrupt__,__no_auto_psv__)) _ADC1Interrupt(void)
{
    BLEEP_ON_INTS
    DISABLE_SYSTEM_INTERRUPT
    STORE_OS_PSV(pp_block)
        
    _AD1IF = 0;
    gPowerInfo.VddADCCounts = ADC1BUF0;
    gPowerInfo.VppADCCounts = ADC1BUF1;
    
    if (gPowerInfo.Flag.VppIsOn && !gPowerInfo.Flag.InternalMCLR)
    {
        if (gPowerInfo.VppADCCounts > gPowerInfo.VppADCCountsUpperLimit)
        {
            OC1CON1 = PWM_OFF;
        }
        if (gPowerInfo.VppADCCounts < gPowerInfo.VppADCCountsLowerLimit)
        {
            if (OC1CON1 == PWM_OFF)
                OC1CON1 = PWM_SET_FOR_TIMER3_CENTER_PWM;  // start PWM
        }
    }
    
    RESTORE_OS_PSV
    RESTORE_SYSTEM_INTERRUPT
    EBLEEP_ON_INTS
}

/******************************************************************************
 * Function:        void TurnVPP(byte state)
 *
 * Overview:        Turn VPP On/Off.
 *
 * PreCondition:    None
 *
 * Input:           state - TRUE=On FALSE=off
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
void TurnVPP(byte state)
{
    // Only do this if VPP is on
    if (state)
    {
            // See if we are going to high voltage or not
            if(gPowerInfo.Flag.VppIsSetup)
            {
                TURN_ON_VPP_SUPPLY
            }
            
            // Now turn it on 
            pin_VPP_GROUND = 0;
            pin_VPP_ON     = 1;
    }
    else
    {
        if(!gPowerInfo.Flag.InternalMCLR)
        {
            TURN_OFF_VPP_SUPPLY
            
            // Old Method
            pin_VPP_GROUND = 1; // clear out the CAP
            Delayus(7000);      // allow it to discharge to ~7v
            pin_VPP_ON     = 0; // then shut the supply off

//            // New Method                     
//          pin_VPP_ON     = 0; // FIRST shut the supply off
//          pin_VPP_GROUND = 1; // and drive it hard to ground
//          Delayus(7000);      // allow some time for any external caps on VPP to discharge 

        }
        else
        {
            pin_VPP_ON     = 0; // Shut the supply off 
            pin_VPP_GROUND = 1; // Now ground it
        }
    }
}   

/******************************************************************************
 * Function:        bool isVPPOn(void)
 *
 * Overview:        Returns the status of the VPP_ON pin.
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          1/0
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
bool isVPPOn(void)
{
    return pin_VPP_ON; 
}
