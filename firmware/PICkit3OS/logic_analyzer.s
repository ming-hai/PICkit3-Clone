;*********************************************************************
;
; Logic Analyzer Low Level Driver
;
;
;*********************************************************************
; FileName:             logic_analyzer.s
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

.include "p24fxxxx.inc"

.global  _LogicAnalyzerAsm
    
;------------------------------------------------------------------------------
;; Code Section
    .text   

;/******************************************************************************
; * Function:        _LogicAnalyzerAsm - 
; *                 WORD LogicAnalyzerAsm(WORD RiseFallMask, WORD SamplingDelay)
; *
; * Overview:        Logic analyzer with edge detection, sample rate up to 1MHz
; *
; * PreCondition:    Interrupts disabled
; *
; * Input:           cnvEdgeMask, PostTrigCount
; *
; * Output:          TrigCount - trigger location
; *                  RAM 0x4000-0x4400 : 1024 samples
; *
; * Side Effects:    Writes RAM addresses 0x4000 to 0x4400 as one sample per 
; *                  location
; *
; * Note:            Samples are stored as one sample per word. Some post 
; *                  processing is performede afterwards to make it compatible
; *                  with the PICkit 2 app. This could be changed in the future
; *                  W3 stores trigger location
; *                  W6 contains address pointer
; *                  W7 holds the delay 
; *                  W8 stores rise/fall pointer
; *                  W9 stores last sample
; *                  W10 contains the edge mask
; *                  W11 contains Post Trigger Count
; *                  W12 presample count, and temporary holder
; *****************************************************************************/
_LogicAnalyzerAsm:
    MOV     W0, W8
    MOV     W1, W7
    MOV     #0x4000, W6         ; sampling buffer address
    MOV     _cnvEdgeMask, W10
    MOV     _PostTrigCount, W11

    MOV     #0x400, W12         ; 1024 presamples
Analyzer_presample:
    ; Get samples from PORTD (RD2, RD3) and PORTF (RF3) and merge them in 
    ; one register
    MOV     PORTD , W0
    MOV     PORTF , W1
    AND     W0, #0xC, W0
    SL      W1, #4, W1
    IOR     W1, W0, W0

    MOV.B   W0, [W6++]
    BCLR    W6, #10                 ; scroll back to 0 if you've exceeded 1024

    ; Add variable Delay, W7=0 for 1 MHz sampling rate
    REPEAT  W7
    NOP

    DEC     W12, W12
    BRA     NZ, Analyzer_presample_redir

    ; load last sample and equalize timing
    MOV     W0, W9
    NOP
    NOP

    BRA     LogicAnalyzerLoop
Analyzer_presample_redir:
    NOP
    NOP
    BRA Analyzer_presample
Analyzer_NoTrigger1:
    ; Check for user pressing button to abort triggering
    BTSS    PORTB, #8
    BRA     Analyzer_Trigabort
    NOP
Analyzer_NoTrigger2:
    MOV.B       [W6++], W9      ; store last sample in W9
    BCLR    W6, #10             ; scroll back to 0 if you've exceeded 1024 samples
LogicAnalyzerLoop:
    ; Get sample from PORTD (RD2, RD3) and PORTF (RF3) and merge them in one 
    ; register. This is due to the fact that we have the 3 target pins on two 
    ; different ports and that two of them share the same position in the port
    ; This wastes 5 instructions for what the PICkit 2 could do in one.
    ; Challenge: Could you do this in less than 5 instructions?
    ; Future todo: add a fast triggering mode that doesn't involve PORTF and 
    ; a fast capture mode
    MOV     PORTD , W0
    MOV     PORTF , W1
    AND     W0, #0xC, W0
    SL      W1, #4, W1
    IOR     W1, W0, W0          ; Now sample is in W0

    ; Add Delay. Only the first operation is needed but we repeat it as many 
    ; times as needed to achieve the required sampling rate. Repeating the
    ; instruction over and over should be of no harm but it helps us save
    ; an instruction and we can fit within the required 16 cycles (1us @16MIPS)
    ; This saves us from having separate functions for separate rates and 
    ; trigger types as the PICkit 2 firmware does
    REPEAT  W7
    MOV.B   W0, [W6]            ; store in buffer

    ; Check for triggers
    AND     _cnvTrigMask, WREG
    XOR     _cnvTrigStates, WREG
    BRA     NZ, Analyzer_NoTrigger1
    AND     W10, W9, W12            ; AND with the edge mask
    XOR     W8, W12, W12
    BRA     NZ, Analyzer_NoTrigger2
    DEC     _TrigCount  
    BRA     NZ, Skip_one_sample_after_trigger_to_update_last_sample
    NOP
Analyzer_Trigger_firstsample:       ; So the trigger position can be stored
    MOV     PORTD , W0
    MOV     PORTF , W1
    AND     W0, #0xC, W0
    SL      W1, #4, W1
    IOR     W1, W0, W0

    ; Advance the pointer since we haven't gotten the chance to in the main loop
    INC     W6,W6
    BCLR    W6, #10                 ; scroll back to 0 if you've exceeded 1024

    MOV     W6, W3                  ; store trigger location in W3
    
    MOV.B   W0, [W6++]
    BCLR    W6, #10                 ; scroll back to 0 if you've exceeded 1024

    ; Add variable Delay, W7=0 for 1 MHz sampling rate
    REPEAT  W7
    NOP

    NOP                             ; add NOPs to make timing equal to 16 cycles
    NOP
    NOP
    NOP
Analyzer_Trigger:
    ; Get sample from PORTD (RD2, RD3) and PORTF (RF3) and merge them in one register
    MOV     PORTD , W0
    MOV     PORTF , W1
    AND     W0, #0xC, W0
    SL      W1, #4, W1
    IOR     W1, W0, W0

    MOV.B   W0, [W6++]
    BCLR    W6, #10                 ; scroll back to 0 if you've exceeded 1024

    ; Add variable Delay, W7=0 for 1 MHz sampling rate
    REPEAT  W7
    NOP

    DEC     W11, W11
    BRA     NZ, Analyzer_Trigger_redir

    ; return sample position    
    MOV     W3, W0
    RETURN

Analyzer_Trigger_redir:
    NOP
    NOP
    BRA Analyzer_Trigger

    ; This is mainly to update W9 so that we don't get false triggers if trigger 
    ; count is larger than one. Due to the tight available cycles, updating W9 
    ; couldn't be done in the main loop body and had to be moved to 
    ; Analyzer_NoTrigger2 which means it won't get executed if we trigger on an
    ; edge but TrigCount > 1. This can lead to multiple subsequent false triggers
    ; (If TrigCount==1 we don't care about last sample).
Skip_one_sample_after_trigger_to_update_last_sample:
    MOV     PORTD , W0
    MOV     PORTF , W1
    AND     W0, #0xC, W0
    SL      W1, #4, W1
    IOR     W1, W0, W0

    MOV.B   [W6++], W9              ; store last sample
    BCLR    W6, #10                 ; scroll back to 0 if you've exceeded 1024

    MOV.B   W0, [W6++]
    BCLR    W6, #10                 ; scroll back to 0 if you've exceeded 1024

    ; Add variable Delay, W7=0 for 1 MHz sampling rate
    REPEAT  W7
    NOP

    NOP                             ; add NOPs to make timing equal to 16 cycles
    NOP
    NOP
    BRA     LogicAnalyzerLoop

Analyzer_Trigabort:
    MOV     #0xFFFF, W0
    RETURN

.end
