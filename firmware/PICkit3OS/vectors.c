/*********************************************************************
 *
 * Interrupt vectors routines
 *
 *********************************************************************
 * FileName:            vectors.c
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

extern void             (*gblTimerEvent)(void *);
extern void             *gblTimerEventAttribs;
extern TIMERMETHOD      gblTimerMethod;
const char              v_block[] = {1};            // just for reseting the PSV

bool _ButtonChanged=0;
WORD ptg_err_code;
WORD cnt = 0;

//---------------------------------------------------------------------
// Timer 2 Periodic timer interrupt 
//---------------------------------------------------------------------
// Used for any operation to be run periodically
//
void __attribute__((__interrupt__,__no_auto_psv__)) _T2Interrupt(void)
{
    BLEEP_ON_INTS
    DISABLE_SYSTEM_INTERRUPT
    STORE_OS_PSV(v_block)
        
    _T2IF = 0;
    
    pin_LED_ACTIVE =1;

    RESTORE_OS_PSV
    RESTORE_SYSTEM_INTERRUPT
    EBLEEP_ON_INTS
}

extern void USBDeviceTasks(void);

//---------------------------------------------------------------------
// USB interrupt
// 
void __attribute__((__interrupt__,__no_auto_psv__)) _USB1Interrupt(void)
{
    BLEEP_ON_INTS
    {
        DISABLE_SYSTEM_INTERRUPT
        STORE_OS_PSV(v_block)
       
        USBDeviceTasks();
        _USB1IF = 0;
        _USB1IE = 1;
        
        RESTORE_OS_PSV
        RESTORE_SYSTEM_INTERRUPT
    }
    EBLEEP_ON_INTS
}

//---------------------------------------------------------------------
//  Timer5 Interrupt - Generic Programmable timer
//--------------------------------------------------------------------- 
// Used for system wide delay and programming functions
//  
void __attribute__((__interrupt__,__no_auto_psv__)) _T5Interrupt(void)
{
    BLEEP_ON_INTS
    _T5IF = 0;
    
    // Turn if off if its a oneshot
    if(gblTimerMethod == ONESHOT)
        _T5IE = 0;  
        
    if(gblTimerEvent != NULL && gblTimerEvent != TIMER_EVENT_PGMWAIT)   
    {   
        // run request operation
        (*gblTimerEvent)(gblTimerEventAttribs);
    }
        
    EBLEEP_ON_INTS
}

void __attribute__((__interrupt__,__no_auto_psv__)) _CNInterrupt(void)
{
    BLEEP_ON_INTS
    STORE_OS_PSV(v_block) 
            
    _CNIF = 0x0;

    // Notification on RP8/CN26
    _ButtonChanged = 1;
        
    RESTORE_OS_PSV  
    EBLEEP_ON_INTS
}   
    
//-------------------------------------------------------------------------------------------
// System_InterruptController
// -- Shows the current status of the interrupts. Can turn the interrupts on or off if needed
//    Using the DISABLE_SYSTEM_INTERRUPT and RESTORE_SYSTEM_INTERRUPT macros is probably the best
//
INTERRUPTCONTROL System_InterruptController(INTERRUPTCONTROL InterruptRequest)
{   
    volatile int    ___current_cpu_ipl;
    // Basic interrupt controller controls (enable, disable, status) 
    SET_AND_SAVE_CPU_IPL(___current_cpu_ipl, 7);
    
    // If interrupt enable is changing, this is the interrupt status *before* the change:
    INTERRUPTCONTROL PreChangeStatus = *InterruptStatus;
    
    if(*InterruptStatus == SYS_INTERRUPT_UNKNOWN)
    {
        // Make it black or white (interrupt is marked as on if its priority level is less than 7)
        if((___current_cpu_ipl & 0x00E0)  == 0x00E0)
            *InterruptStatus = SYS_INTERRUPT_DISABLE;
        else                
            *InterruptStatus = SYS_INTERRUPT_ENABLE;
        
        PreChangeStatus = *InterruptStatus;
    }
    
    // Making sure it is set correctly      
    if(InterruptRequest !=  *InterruptStatus)
    {           
        switch (InterruptRequest)
        {
        case SYS_INTERRUPT_DISABLE:
            // Disable interrupts
            //  - RDK: If an interrupt event occurs while interrupts are disabled, 
            //    the interrupt loop should wait on this Interrupt Enable event 
            //    before allowing the interrupt handler to run. (None of this 
            //    applies to a real embedded system; the interrupt controller
            //    hardware logic does the work!)        
            // Set current IPL to level 7 (only trap errors)
            ___current_cpu_ipl = 7; 
            
            *InterruptStatus = InterruptRequest;
            break;
    
        case SYS_INTERRUPT_ENABLE:
            // Enable interrupts
            
            // Set IPL to handle all requests
            ___current_cpu_ipl = DEFAULT_PRIORITY_LEVEL; 
            
            *InterruptStatus = InterruptRequest;
            break;
    
        case SYS_INTERRUPT_STATUS:          
        default:
        
        
            break;
        }
    }
    
    SET_CPU_IPL(___current_cpu_ipl);
    
    return PreChangeStatus;    
}

/********* END OF INTERRUPT SERVICE ROUTINES ********/
