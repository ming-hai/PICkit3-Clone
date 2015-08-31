;*********************************************************************
;
; Interrupt Table
;
;
;*********************************************************************
; FileName:             os_ivt_pk3.s
; Dependencies:
; Processor:            PIC24
; Assembler/Compiler:   MPLAB C30 3.xx
; Linker:               MPLAB C30 3.xx
; Company:              Microchip Technology, Inc.
;
; Copyright (c) 2012 Microchip Technology Inc. All rights reserved.
; 
; Software License Agreement
;
; The software supplied herewith by Microchip Technology Incorporated
; (the “Company”) for its PICmicro® Microcontroller is intended and
; supplied to you, the Company’s customer, for use solely and
; exclusively on Microchip PICmicro Microcontroller products. The
; software is owned by the Company and/or its supplier, and is
; protected under applicable copyright laws. All rights are reserved.
; Any use in violation of the foregoing restrictions may subject the
; user to criminal sanctions under applicable laws, as well as to
; civil liability for the breach of the terms and conditions of this
; license.
;
; THIS SOFTWARE IS PROVIDED IN AN “AS IS” CONDITION. NO WARRANTIES,
; WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
; TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
; PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
; IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
; CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
;
;*********************************************************************
; Change log
;
; Initial Release
;
;*********************************************************************
    ; For the revectoring of interrupts. Need to place any new interrupts into this table. 
    ; The compiler will not automatically place them here 
    
.section ivt_vectors, code
    ;__ReservedTrap0
    goto __DefaultInterrupt
    
    ;__OscillatorFail
    goto __OscillatorFail

    ;__AddressError
    goto __AddressError

    ;__StackError
    goto __StackError

    ;__MathError   
    goto __MathError

    ;__ReservedTrap5
    goto __DefaultInterrupt
    
    ;__ReservedTrap6
    goto __DefaultInterrupt
    
    ;__ReservedTrap7
    goto __DefaultInterrupt
    
    ;__INT0Interrupt
    goto __DefaultInterrupt
    
    ;__IC1Interrupt
    goto __DefaultInterrupt
    
    ;__OC1Interrupt
    goto __DefaultInterrupt
    
    ;__T1Interrupt
    goto __T1Interrupt
    
    ;__Interrupt4
    goto __DefaultInterrupt
    
    ;__IC2Interrupt

    goto __DefaultInterrupt
    
    ;__OC2Interrupt
    goto __DefaultInterrupt
    
    ;__T2Interrupt
    goto __T2Interrupt

    ;__T3Interrupt
    goto __DefaultInterrupt
    
    ;__SPI1ErrInterrupt
    goto __DefaultInterrupt 
    
    ;__SPI1Interrupt
    goto __DefaultInterrupt 

    ;__U1RXInterrupt
    goto __DefaultInterrupt
    
    ;__U1TXInterrupt
    goto __DefaultInterrupt
    
    ;__ADC1Interrupt

    goto __ADC1Interrupt
    
    ;__Interrupt14
    goto __DefaultInterrupt
    
    ;__Interrupt15
    goto __DefaultInterrupt
    
    ;__SI2C1Interrupt
    goto __DefaultInterrupt
    
    ;__MI2C1Interrupt
    goto __DefaultInterrupt
    
    ;__CompInterrupt
    goto __DefaultInterrupt
    
    ;__CNInterrupt
    goto __CNInterrupt

    ;__INT1Interrupt
    goto __DefaultInterrupt

    ;__Interrupt21
    goto __DefaultInterrupt
    
    ;__IC7Interrupt
    goto __DefaultInterrupt
    
    ;__IC8Interrupt
    goto __DefaultInterrupt
    
    ;__Interrupt24
    goto __DefaultInterrupt
    
    ;__OC3Interrupt
    goto __DefaultInterrupt
    
    ;__OC4Interrupt
    goto __DefaultInterrupt
    
    ;__T4Interrupt
    goto __DefaultInterrupt
    
    ;__T5Interrupt
    goto __T5Interrupt

    ;__INT2Interrupt
    goto __DefaultInterrupt

    ;__U2RXInterrupt
    goto __DefaultInterrupt
    
    ;__U2TXInterrupt
    goto __DefaultInterrupt
    
    ;__SPI2ErrInterrupt
    goto __DefaultInterrupt
    
    ;__SPI2Interrupt
    goto __DefaultInterrupt
    
    ;__Interrupt34
    goto __DefaultInterrupt
    
    ;__Interrupt35
    goto __DefaultInterrupt
    
    ;__Interrupt36
    goto __DefaultInterrupt
    
    ;__IC3Interrupt

    goto __DefaultInterrupt
    
    ;__IC4Interrupt

    goto __DefaultInterrupt
    
    ;__IC5Interrupt

    goto __DefaultInterrupt
    
    ;__IC6Interrupt

    goto __DefaultInterrupt
    
    ;__OC5Interrupt
    goto __DefaultInterrupt
    
    ;__OC6Interrupt
    goto __DefaultInterrupt
    
    ;__OC7Interrupt
    goto __DefaultInterrupt
    
    ;__OC8Interrupt
    goto __DefaultInterrupt
    
    ;__PMPInterrupt
    goto __DefaultInterrupt
    
    ;__Interrupt46
    goto __DefaultInterrupt
    
    ;__Interrupt47
    goto __DefaultInterrupt
    
    ;__Interrupt48
    goto __DefaultInterrupt
    
    ;__SI2C2Interrupt
    goto __DefaultInterrupt
    
    ;__MI2C2Interrupt
    goto __DefaultInterrupt
    
    ;__Interrupt51
    goto __DefaultInterrupt
    
    ;__Interrupt52
    goto __DefaultInterrupt
    
    ;__INT3Interrupt
    goto __DefaultInterrupt

    ;__INT4Interrupt
    goto __DefaultInterrupt
    
    ;__Interrupt55
    goto __DefaultInterrupt
    
    ;__Interrupt56
    goto __DefaultInterrupt
    
    ;__Interrupt57
    goto __DefaultInterrupt
    
    ;__Interrupt58
    goto __DefaultInterrupt
    
    ;__Interrupt59
    goto __DefaultInterrupt
    
    ;__Interrupt60
    goto __DefaultInterrupt
    
    ;__Interrupt61
    goto __DefaultInterrupt
    
    ;__RTCCInterrupt
    goto __DefaultInterrupt
    
    ;__Interrupt63
    goto __DefaultInterrupt
    
    ;__Interrupt64
    goto __DefaultInterrupt
    
    ;__U1ErrInterrupt
    goto __DefaultInterrupt
    
    ;__U2ErrInterrupt
    goto __DefaultInterrupt
    
    ;__CRCInterrupt
    goto __DefaultInterrupt
    
    ;__Interrupt68
    goto __DefaultInterrupt
    
    ;__Interrupt69
    goto __DefaultInterrupt
    
    ;__Interrupt70
    goto __DefaultInterrupt
    
    ;__Interrupt71
    goto __DefaultInterrupt
    
    ;__LVDInterrupt
    goto __DefaultInterrupt
    
    ;__Interrupt73
    goto __DefaultInterrupt
    
    ;__Interrupt74
    goto __DefaultInterrupt
    
    ;__Interrupt75
    goto __DefaultInterrupt
    
    ;__Interrupt76
    goto __DefaultInterrupt
    
    ;__CTMUInterrupt
    goto __DefaultInterrupt
    
    ;__Interrupt78
    goto __DefaultInterrupt
    
    ;__Interrupt79
    goto __DefaultInterrupt
    
    ;__Interrupt80
    goto __DefaultInterrupt
    
    ;__U3ErrInterrupt
    goto __DefaultInterrupt
    
    ;__U3RXInterrupt
    goto __DefaultInterrupt
    
    ;__U3TXInterrupt
    goto __DefaultInterrupt
    
    ;__SI2C3Interrupt
    goto __DefaultInterrupt
    
    ;__MI2C3Interrupt
    goto __DefaultInterrupt
    
    ;__USB1Interrupt
    goto __USB1Interrupt
    
    ;__U4ErrInterrupt
    goto __DefaultInterrupt
    
    ;__U4RXInterrupt
    goto __DefaultInterrupt
    
    ;__U4TXInterrupt
    goto __DefaultInterrupt
    
    ;__SPI3ErrInterrupt
    goto __DefaultInterrupt
    
    ;__SPI3Interrupt
    goto __DefaultInterrupt
    
    ;__OC9Interrupt
    goto __DefaultInterrupt
    
    ;__IC9Interrupt
    goto __DefaultInterrupt
    
    ;__Interrupt94
    goto __DefaultInterrupt
    
    ;__Interrupt95
    goto __DefaultInterrupt
    
    ;__Interrupt96
    goto __DefaultInterrupt
    
    ;__Interrupt97
    goto __DefaultInterrupt
    
    ;__Interrupt98
    goto __DefaultInterrupt
    
    ;__Interrupt99
    goto __DefaultInterrupt
    
    ;__Interrupt100
    goto __DefaultInterrupt
    
    ;__Interrupt101
    goto __DefaultInterrupt
    
    ;__Interrupt102
    goto __DefaultInterrupt
    
    ;__Interrupt103
    goto __DefaultInterrupt
    
    ;__Interrupt104
    goto __DefaultInterrupt
    
    ;__Interrupt105
    goto __DefaultInterrupt
    
    ;__Interrupt106
    goto __DefaultInterrupt
    
    ;__Interrupt107
    goto __DefaultInterrupt
    
    ;__Interrupt108
    goto __DefaultInterrupt
    
    ;__Interrupt109
    goto __DefaultInterrupt
    
    ;__Interrupt110
    goto __DefaultInterrupt
    
    ;__Interrupt111
    goto __DefaultInterrupt
    
    ;__Interrupt112
    goto __DefaultInterrupt
    
    ;__Interrupt113
    goto __DefaultInterrupt
    
    ;__Interrupt114
    goto __DefaultInterrupt
    
    ;__Interrupt115
    goto __DefaultInterrupt
    
    ;__Interrupt116
    goto __DefaultInterrupt
    
    ;__Interrupt117
    goto __DefaultInterrupt
    
