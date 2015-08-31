/*********************************************************************
 *
 * PICkit 3 Power Supply Code header file
 *
 *********************************************************************
 * FileName:            powerPK3.h
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

#ifndef POWERPK3_DOT_H
#define POWERPK3_DOT_H

//********************************************************************
// Local Functions
//********************************************************************

bool ChangePowerMode(PWRMODE Mode, PWRMODE *OldMode);
void InitPowerTrain(void);
void ResetEmulatorPowerStruct(void);
BOOL SetVdd(WORD tickvoltage);
BOOL SetVpp(WORD tickvoltage);

void CalibrateADC(void);  
void ControlUSBPower(bool on);
void DelayForSampleAndConversionTime(void);
word GetClosestCount(float v);
word GetVppInCounts(void);
float GetVddInVolts(void);
word GetVddInCounts(void);
float GetVppInVolts(void);
BOOL InternalSetVdd(float voltage);
bool InternalSetVpp(float voltage);
void MySetPWMInCounts(unsigned char which, unsigned int counts);
void PK3ExternalVoltageDetection(void);
unsigned int SafelyGetVppCount(void);
unsigned int SafelyGetVddCount(void);
void SetupADCToReadVddAndVpp(void);
void TargetPowerLogic(WORD ADCValue);
void TurnVPP(byte state);

//********************************************************************
// Defines
//********************************************************************

//********************************************************************
// Relationship between Vpp and Vdd and how many counts they represent 
// in the ADC result registers (ADC1BUF0,1)  
//********************************************************************

/*
 * VDD_VOLTAGE_DIVISOR
 *
 *  Vdd Voltage Divisor
 *
 * Vdd is connected through a voltage divisor into AN2. 
 * The divisor is 2.2/(2.2+3.9) = 0.3606557
 * this means that the voltage seen by AN2 is %36 of Vdd
 *
 */
#define VDD_VOLTAGE_DIVISOR 0.3606557

/*
 * FROM_VDD_VOLTAGE_TO_ADC_COUNTS
 *
 * From Vdd Voltage to A/D Counts
 * 
 * Given a voltage at Vdd, calculate how many counts the ADC would see.
 *
 */
#define FROM_VDD_VOLTAGE_TO_ADC_COUNTS(_v_) \
        (((_v_) * VDD_VOLTAGE_DIVISOR )/gPowerInfo.VoltsPerADCCount)
        
/*
 * FROM_ADC_COUNTS_TO_VDD_VOLTAGE
 *
 * From A/D Counts to Vdd Voltage
 *
 * Given a number of counts in the ADC, calculate the corresponding 
 * voltage in Vdd.
 *
 */
#define FROM_ADC_COUNTS_TO_VDD_VOLTAGE(_c_) \
        ((((float)_c_)*gPowerInfo.VoltsPerADCCount)/VDD_VOLTAGE_DIVISOR)

/*
 * VPP_VOLTAGE_DIVISOR
 *
 * Vpp Voltage Divisor
 *
 * Vpp is connected through a voltage divisor into AN3. 
 * The divisor is 2.2/(2.2+10.0) = 0.1803279 
 * this means that the voltage seen by AN3 is %18 of Vpp. 
 *
 */
#define VPP_VOLTAGE_DIVISOR 0.1803279


/*
 * FROM_VPP_VOLTAGE_TO_ADC_COUNTS
 *
 * From Vpp Voltage to A/D Counts
 * 
 * Given a voltage at Vpp, calculate how many counts the ADC would see.
 *
 */
#define FROM_VPP_VOLTAGE_TO_ADC_COUNTS(_v_) \
        (((_v_) * VPP_VOLTAGE_DIVISOR )/gPowerInfo.VoltsPerADCCount)

/*
 * FROM_ADC_COUNTS_TO_VPP_VOLTAGE
 *
 * From A/D Counts to Vpp Voltage
 *
 * Given a number of counts in the ADC, calculate the corresponding 
 * voltage in Vpp.
 *
 */
#define FROM_ADC_COUNTS_TO_VPP_VOLTAGE(_c_) \
        ((((float)_c_)*gPowerInfo.VoltsPerADCCount)/VPP_VOLTAGE_DIVISOR)
        

/*
 * TURN_ON_VPP_SUPPLY
 *
 * Turn on Vpp Supply.
 *
 * We now turn on the Vpp supply seperately from the set Vpp.
 * Note: what it takes 25ms for Vpp to rise
 *
 */
#define TURN_ON_VPP_SUPPLY  { \
                                OC1CON1 = PWM_SET_FOR_TIMER3_CENTER_PWM; \
                                gPowerInfo.Flag.VppIsOn = TRUE;  \
                                Delayus(25000);      \
                            }
                            
/*
 * TURN_OFF_VPP_SUPPLY
 *
 * Turn off Vpp supply
 *
 */
#define TURN_OFF_VPP_SUPPLY { \
                                gPowerInfo.Flag.VppIsOn = FALSE; \
                                OC1CON1 = PWM_OFF;   \
                            }               

//********************************************************************
// Relationship between PWM duty cycle (counts in OC2RS) and the 
// produced Vdd
//********************************************************************

/*
 * VDD_VOLTS_PER_PWM_DUTY_CYCLE_COUNT
 *
 * Vdd volts-per-PWM cycle count
 *
 * I calculated how the counts placed in the PWM duty cycle register 
 * (OC2RS) relate to the voltage that appears in VDD. 
 *
 * Vdd is perfectly linear in relation to the count that goes in the 
 * duty cycle register. I checked in ranges 1->5V (no need to worry 
 * about < 1 volt). 
 *
 * I found that each increment in duty cycle from >1 volt to 4.96. 
 * produce 6.2mvolts. this is true to 4.96 (count = 80). With a count 
 * of 81 we get 5.0 (so it's only 4mvolts). 
 *
 * In summary, this is how many volts per count. 
 *
 *
 */
#define VDD_VOLTS_PER_PWM_DUTY_CYCLE_COUNT 0.060 


/*
 * FROM_VDD_VOLTAGE_TO_PWM_COUNTS
 *
 * Vdd voltage to PWM counts.
 *
 * This is how we know what to plug in the PWM duty cycle (OC2RS) to 
 * produce a given value of Vdd. The 0.5 is to round up. It is needed 
 * to make sure that for 5.0 volts the counts round up to 81 (not 80 
 * which is 4.96).
 *
 *
 */
#define FROM_VDD_VOLTAGE_TO_PWM_COUNTS(_v_) \
    (unsigned int)(((_v_)/VDD_VOLTS_PER_PWM_DUTY_CYCLE_COUNT) + 0.5)



//********************************************************************
// Vpp is controlled in a different way: the duty cycle of its PWM 
// (OC1RS) is kept constant at %50. Then the PWM is switched on and off 
// as needed. So, there is no need for a FROM_VPP_VOLTAGE_TO_PWM_COUNTS. 
// Instead new fields in the gPowerInfo structure are needed: 
// VppADCCountsUpperLimit, VppADCCountsLowerLimit and VppPWMCountsSetPoint.
//********************************************************************

/*
 * DEFAULT_VDD
 * Default Vdd (1.5 volts)
 */
#define DEFAULT_VDD         0x0C

/*
 * DEFAULT_VPP
 * Default Vpp (1.5 volts)
 */
#define DEFAULT_VPP         0x0C

/*
 * MAX_VDD_ERROR
 * Maximum Vdd error
 */
#define MAX_VDD_ERROR       0.16

/*
 * VOLTAGE_DETECTED_THRES
 *
 * Voltage detect threshold
 *
 * See excel spreedsheet, 0x188 = roughly 1.45 volts
 *
 */
//#define VOLTAGE_DETECTED_THRES    0x188
#define VOLTAGE_DETECTED_THRES  0x98    

/*
 * DEBOUNCE_THRES
 * Debounce Threshold
 */
#define DEBOUNCE_THRES          5

/*
 * VDD_WHEN_TARGET_IS_SOURCING
 * Vdd when target is sourcing
*/
#define VDD_WHEN_TARGET_IS_SOURCING 4.5


/*
 * VDD_TOO_MANY_LOW
 * at 125usec, this is 2 msecs)
 */
#define VDD_TOO_MANY_LOW    16

/*
 * VPP_TOO_MANY_LOW
 * (at 125usec, this is 2 msecs)
 */
#define VPP_TOO_MANY_LOW    16


// PWRFLAGS
//
typedef union
{
    struct
    {
        unsigned ManualConversionMutex:1;   // Manual conversion
        unsigned TargetPowered:1;           // Set to use Target Power, clear to use emulator power
        unsigned TgtPwrMutex:1;             // NEVER USED
        unsigned EmuPwrStayOn:1;            // If set, power stays on after a programming operation
        unsigned MCLRHELD:1;                // If set, MCLR is held even when we are done
        unsigned InternalMCLR:1;            // Internal MCLR
        unsigned VddIsOn:1;                 // Vdd is ON
        unsigned VppIsOn:1;                 // Vpp is ON
        unsigned VppIsSetup:1;              // Vpp is setup
        unsigned FirstTime:1;               // First time for internal power supply
    };
    WORD    All;
}PWRFLAGS;
 
 
// POWER_INFO
//
/*
 * Power information
 *
 * Vdd is not controlled in a loop as is Vpp. Vdd is simply monitor to 
 * make sure it does not go below the desired threshold. 
 *
 * Vpp is also monitored to make sure it does not go too low. However, 
 * Vpp is indeed controlled by turning on and shutting off the PWM (OC1)          
 *
 */
typedef struct _POWER_INFO
{
    float           VoltsPerADCCount;       // Fill in by CalibrateADC()

   
    unsigned int    VddADCCounts;           // the last value read from ADC
    unsigned int    VddADCCountsTooLow;     // shut off the whole thing if we see counts less than 
    unsigned int    VddTooLowCounter;       // VddTooLow for VDD_TOO_MANY_LOW times. 
                             
    unsigned int    VppADCCounts;           // the last value read from ADC
    unsigned int    VppADCCountsTooLow;     // shut off the whole thing if we see counts less than 
    unsigned int    VppTooLowCounter;       // VppADCCountsTooLow for VPP_TOO_MANY_LOW times. 

    unsigned int    VppADCCountsUpperLimit; // To control Vpp, shut off PWM (OC1) if we see more than this
    unsigned int    VppADCCountsLowerLimit; // To control Vpp, re-start PWM (OC1) if we see less than this
    unsigned int    VppPWMCountsSetPoint;   // PWM (OC1) duty cycle. PK2 makes it always %50! So do we.

    PWRMODE         CurPwrMode;             // Current power mode
    BYTE            Debounce;               // Debounce
    PWRFLAGS        Flag;                   // Power flags
    
}   POWER_INFO;


/*
 * MAX_VALUE_PERIOD_FOR_PWM_TIMER
 * Maximim value period for PWM timer
 *
 * We use 150KHz PWM freq (same as PK2). 
 * 
 * From DS39897B-page 163; 
 *      PWM Freq = 1/ ( [(PRy) + 1]*Tcy*(Timer Prescale Value) ) 
 *      Tcy = 1/Fcy = 1/16MHz = 62.5 nsecs, PWM Freq = 150e3 Hz  
 *      150e3 = 1/((PRy + 1)*62.5e-9*TimerPrescaleVale)  
 *  Solving for PRy:  
 *      PRy =  (106.6 / *TimerPrescaleValue) - 1  
 *      For TimerPrescalValue = 1, PRy = 105  
 * 
 * Resolution is (from the same page of the data sheet): 
 * 
 * Resolution in bits = log10( Fcy/(Fpwm*TimerPrescaleValue) )/ Log10(2) 
 * Resolution in bits = log10( 16e6/150e3 )/ Log10(2) 
 * Resolution in bits = 7 bits  
 *
 */
#define MAX_VALUE_PERIOD_FOR_PWM_TIMER    105


/*
 * VPP_DUTY_CYCLE
 * Vpp Duty Cycle
 *
 * PK2 sets its Vpp duty cycle at 64/79.  
 * For us that would be 105*(64/79)  
 *
 */
#define VPP_DUTY_CYCLE                    85

/*
 * MAX_VDD
 * Maximum Vdd
 */
#define MAX_VDD                           5.0


/*
 * PWM_OFF
 * PWM Off (disabled)
 */
#define PWM_OFF                         0x0000

/*
 * PWM_SET_FOR_TIMER3_CENTER_PWM
 * PWM set for TIMER3 center PWM
 *
 * synch to timer3
 *
 */
#define PWM_SET_FOR_TIMER3_CENTER_PWM   0x0407

/*
 * PWM_SET_FOR_TIMER3_INPUT
 * PWM set for TIMER3 Input
 *
 * Count from timer 3 and center align
 *
 */
#define PWM_SET_FOR_TIMER3_INPUT        0x000D


enum {SELECT_VDD, SELECT_VPP};


/*
 * MAX_VPP
 * Maximum Vpp
 */
#define MAX_VPP     14.0

//********************************************************************
// Enum
//******************************************************************** 

// typVddSense
//
typedef enum _typVddSense{
    TARGETSENSE,        // Target Vdd sense
    INTERNALSENSE}      // Internal Vdd sense
    
    typVddSense; 

//********************************************************************
// End
//******************************************************************** 
 
#endif      // end of POWERPK3_DOT_H 
