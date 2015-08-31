;*********************************************************************
;
; ICSP Routines Low Level Driver
;
;*********************************************************************
; FileName:             icsp.s
; Dependencies:
; Processor:            PIC24
; Assembler/Compiler:   MPLAB XC16 1.00
; Linker:               MPLAB XC16 1.00
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

.macro CLOCK    set_clear
    \set_clear _LATD,#LATD2
.endm

.macro DATA_OUT set_clear
    \set_clear _LATD,#LATD4
.endm

.macro LVP      set_clear	 ; Data in pin for SPI and Microwire devices.
    \set_clear _LATF,#LATF3
.endm

.global  _ShiftBitsOutICSPAsm
.global  _ShiftBitsOutICSPHoldAsm
.global  _ShiftBitsInPIC24Asm
.global  _ShiftBitsInICSPAsm
.global  _SPI_ReadWriteFastAsm
.global  _SPI_ReadWriteSlowAsm
.global  _I2C_WriteFastAsm
.global  _I2C_WriteSlowAsm
.global  _I2C_ReadFastAsm
.global  _I2C_ReadSlowAsm
.extern  _asm_temp2

;------------------------------------------------------------------------------
;; Code Section
    .text

/******************************************************************************
 * Function:        void ShiftBitsOutICSPAsm(WORD outputbyte, WORD numbits,
 *                                           WORD repeat)
 *
 * Overview:        Shifts the given # bits out on the ICSP pins
 *
 * PreCondition:    None
 *
 * Input:           W0 - byte to be shifted out LSB first
 *                  W1 - number of bits to shift
 *                  W2 - number of repititions
 *
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Note:            Assumes ICSP pins are already set to outputs.
 *****************************************************************************/
_ShiftBitsOutICSPAsm:
    BTSS        W0, #0                          ; Check LSB is set
    DATA_OUT    BCLR                            ; If LSB=0 clear data
    BTSC        W0, #0                          ; Check LSB is clear
    DATA_OUT    BSET                            ; If LSB=1 set data

    ; variable setup time
    REPEAT      W2
    NOP

    CLOCK       BSET                            ; set clock high

    ; variable clock width
    REPEAT      W2
    NOP

    CLOCK       BCLR                            ; set clock low
    LSR         W0, W0                          ; shift one position to the
                                                ; right to get the next bit
    DEC         W1, W1                          ; decrement counter
    BRA         NZ, _ShiftBitsOutICSPAsm        ; repeat if not zero

    DATA_OUT    BCLR

    RETURN

/******************************************************************************
 * Function:        void ShiftBitsOutICSPHoldAsm(WORD outputbyte, WORD numbits,
 *                                               WORD repeat)
 *
 * Overview:        Shifts the given # bits out on the ICSP pins
 *                  Differs from ShiftBitsOutICSP in that the instead of
 *                  Setting data, delay, clock high, delay, clock low
 *                  This routine works as
 *                  Setting data, clock high, delay, clock low, delay
 *					This function is used for the HCS devices.
 *
 * PreCondition:    None
 *
 * Input:           W0 - byte to be shifted out LSB first
 *                  W1 - number of bits to shift
 *                  W2 - number of repititions
 *
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Note:            Assumes ICSP pins are already set to outputs.
 *****************************************************************************/
_ShiftBitsOutICSPHoldAsm:
    BTSS        W0, #0                          ; Check LSB is set
    DATA_OUT    BCLR                            ; If LSB=0 clear data
    BTSC        W0, #0                          ; Check LSB is clear
    DATA_OUT    BSET                            ; If LSB=1 set data

    CLOCK       BSET                            ; set clock high

    ; variable hold time
    REPEAT      W2
    NOP

    CLOCK       BCLR                            ; set clock low

    ; variable clock width
    REPEAT      W2
    NOP

    LSR         W0, W0                          ; shift one position to the
                                                ; right to get the next bit
    DEC         W1, W1                          ; decrement counter
    BRA         NZ, _ShiftBitsOutICSPHoldAsm    ; repeat if not zero

    DATA_OUT    BCLR

    RETURN

/******************************************************************************
 * Function:        unsigned char ShiftBitsInPIC24Asm(WORD numbits, WORD repeat)
 *
 * Overview:        Shifts in up to a byte of data.  Shifts in LSB first.
 *                  If less than 8 bits, return byte is right justified.
 *                  If full, sets error Pk2Status.UpLoadFull
 *                  Data is latched on Rising Edge of clock
 * PreCondition:    None
 *
 * Input:           W0 - # bits to shift in (max 8)
 *                  W1 - number of repititions
 *
 * Output:          returns bits right-justified.
 *
 * Side Effects:
 *
 * Note:            Assumes ICSPCLK is output.  Sets ICSPDAT to input then restores
 *                  previous state.
 *                  W2 contains shifted value
 *                  W4 contains how many bits to right justify
 *****************************************************************************/
_ShiftBitsInPIC24Asm:
    CLR         W2                              ; Clear all bits
    SUBR        W0, #8, W4
ShiftLoop24:
    BTSC        PORTD, #3                       ; Read data input pin
    BSET        W2, #0                          ; Set LSB if data input high
    CLOCK       BSET                            ; set clock high
    NOP
    NOP
    ; variable clock width
    REPEAT      W1
    NOP

    CLOCK       BCLR                            ; set clock low

    ; variable clock low time
    REPEAT      W1
    NOP

    RRNC.B      W2, W2                          ; shift one position to the

    DEC         W0, W0                          ; decrement counter
    BRA         NZ, ShiftLoop24                 ; repeat if not zero
                                                ; right to get the next bit
    LSR         W2, W4, W0                      ; right justify

    RETURN

/******************************************************************************
 * Function:        unsigned char ShiftBitsInICSP(unsigned char numbits)
 *
 * Overview:        Shifts in up to a byte of data.  Shifts in LSB first.
 *                  If less than 8 bits, return byte is right justified.
 *                  If full, sets error Pk2Status.UpLoadFull
 * PreCondition:    None
 *
 * Input:           W0 - # bits to shift in (max 8)
 *                  W1 - number of repititions
 *
 * Output:          returns bits right-justified.
 *
 * Side Effects:    Advances upload buffer write pointer, if err Pk2Status.StatusHigh != 0
 *
 * Note:            Assumes ICSPCLK is output.  Sets ICSPDAT to input then restores
 *                  previous state.
 *                  W2 contains shifted value
 *                  W4 contains how many bits to right justify
 *****************************************************************************/
_ShiftBitsInICSPAsm:
    CLR         W2                              ; Clear all bits
    SUBR        W0, #8, W4
ShiftLoopICSP:
    CLOCK       BSET                            ; set clock high
    NOP
    NOP
    NOP
    NOP

    ; variable clock width
    REPEAT      W1
    NOP

    BTSC        PORTD, #3                       ; Read data input pin
    BSET        W2, #0                          ; Set LSB if data input high

    CLOCK       BCLR                            ; set clock low

    ; variable clock low time
    REPEAT      W1
    NOP

    RRNC.B      W2, W2                          ; shift one position to the

    DEC         W0, W0                          ; decrement counter
    BRA         NZ, ShiftLoopICSP               ; repeat if not zero
                                                ; right to get the next bit
    LSR         W2, W4, W0                      ; right justify

    ; some delay
    REPEAT      #10
    NOP

    RETURN

/******************************************************************************
 * Function:        unsigned char SPI_ReadWriteFastAsm(unsigned char outputbyte,
 *                                                     WORD numbits)
 *
 * Overview:        Shifts outputbyte out on the LVP pin with PGC as SCK
 *                  At the same time, bits on PGD are shifted in and
 *                  returned as the read value (fast programming mode).
 *
 * PreCondition:    PGC ==> SCK(clock output to SPI device)
 *					LVP ==> SDI(data output to SPI device)
 *					PGD <== SDO(data input from SPI device)
 *
 * Input:           outputbyte - byte to be shifted out MSb first on AUX
 *
 *
 * Output:          returns the byte shifted in MSb first from PGD
 *
 * Side Effects:    None
 *
 * Note:            Assumes ICSP pins are already set to outputs.
 *****************************************************************************/
_SPI_ReadWriteFastAsm:
    ; Write the data out
    BTSS        W0, #7                          ; Check if the MSB is set.
    LVP         BCLR                            ; If MSB = 0 data out = 0.
    BTSC        W0, #7                          ; Check if the MSB is clear.
    LVP         BSET                            ; If MSB = 1 data out = 1.

    ; Setup time
    NOP
    CLOCK       BSET                            ; Set the clock high.
    NOP
	NOP
	NOP

    RLNC.B      W0, W0                          ; Shift one position to the left.
    ; Read data in
    BCLR        W0, #0                          ; Clear LSB.
    CLOCK       BCLR                            ; Set the clock low (had to move the
                                                ; clock up here for 2.5V operation)
    BTSC        PORTD, #3                       ; Read data input pin
    BSET        W0, #0                          ; If data in = 1 set LSB.

    DEC         W1, W1                          ; decrement counter
    BRA         NZ, _SPI_ReadWriteFastAsm       ; repeat if not zero

    ; Done
    LVP         BCLR                            ; Clear the SO pin.
    RETURN

/******************************************************************************
 * Function:        unsigned char SPI_ReadWriteSlowAsm(unsigned char outputbyte,
 *                                                    WORD numbits, WORD repeat)
 *
 * Overview:        Shifts outputbyte out on the LVP pin with PGC as SCK
 *                  At the same time, bits on PGD are shifted in and
 *                  returned as the read value (slow programming mode).
 *
 * PreCondition:    PGC ==> SCK(clock output to SPI device)
 *					LVP ==> SDI(data output to SPI device)
 *					PGD <== SDO(data input from SPI device)
 *
 * Input:           outputbyte - byte to be shifted out MSb first on AUX
 *
 *
 * Output:          returns the byte shifted in MSb first from PGD
 *
 * Side Effects:    None
 *
 * Note:            Assumes ICSP pins are already set to outputs.
 *****************************************************************************/
_SPI_ReadWriteSlowAsm:
    ; Write the data out
    BTSS        W0, #7                          ; Check if the MSB is set.
    LVP         BCLR                            ; If MSB = 0 data out = 0.
    BTSC        W0, #7                          ; Check if the MSB is clear.
    LVP         BSET                            ; If MSB = 1 data out = 1.

    ; Setup time
    REPEAT      W2                              ; Repeat next instruction
    NOP                                         ; number of times in W2.
    CLOCK       BSET                            ; Set the clock high.
	NOP
	NOP
	NOP
	NOP

    RLNC.B      W0, W0                          ; Shift one position to the left.
    ; Read data in
    BCLR        W0, #0                          ; Clear LSB.
    BTSC        PORTD, #3                       ; Read data input pin
    BSET        W0, #0                          ; If data in = 1 set LSB.

    ; Clock width
    REPEAT      W2
    NOP
    CLOCK       BCLR                            ; Set the clock low.

    DEC         W1, W1                          ; decrement counter
    BRA         NZ, _SPI_ReadWriteSlowAsm       ; repeat if not zero

    ; Done
    LVP         BCLR
    RETURN

/******************************************************************************
 * Function:        void I2C_WriteFastAsm(unsigned char outputbyte, WORD numbits)
 *
 * Overview:        Clocks out a byte with PGC = SCL and LVP = SDA.
 *                  Checks for ACK
 *
 * PreCondition:    PGC = output, LVP = input
 *
 * Input:           outputbyte = byte to be written MSB first
 *
 * Output:          Pk3Status.ICDTimeOut set if NACK received
 *
 * Side Effects:    None
 *
 * Note:            PK3 has a 4.7k pull-down resisitor and a Transil on LVP,
 *					so we cannot support writing I2C devices using pull-ups.
 *					(WE CANNOT SUPPORT I2C DEVICES WITH CURRENT H/W DESIGN)
 *****************************************************************************/
_I2C_WriteFastAsm:

	; Write out 0's driving SDA low if MSB = 0.
	BTSS		W0, #7					; Check if the MSB is set.
	BCLR		_TRISF,#TRISF3			; if MSB = 0 LVP pin is output.
	BTSS		W0, #7					; Check if the MSB is set.
	BSET		_LATE,#LATE2			; if MSB = 0 LVP buffer direction A=>B.

	; Write out 1's using pullup on SDA if MSB = 1.
	BTSC		W0, #7					; Check if the MSB is clear.
	BSET		_TRISF,#TRISF3			; if MSB = 1 LVP pin is input.
	BTSC		W0, #7					; Check if the MSB is clear.
	BCLR		_LATE,#LATE2			; if MSB = 1 LVP buffer direction B=>A.

	REPEAT		#8						; 687.5ns setup time
	NOP									; (11 instructions)

	CLOCK		BSET					; Set the clock high for 875nS.
	REPEAT		#11						; (14 instructions)
	NOP									;

	CLOCK		BCLR					; Set the clock low for 1625nS.
	REPEAT		#1						; (26 instructions)
	NOP									;

	RLNC.B		W0, W0					; Shift one position to the left.
	DEC			W1, W1					; decrement counter
	BRA			NZ, _I2C_WriteFastAsm	; loop if not zero

	; Release SDA for ACK.
	BSET		_TRISF,#TRISF3			; LVP pin set to input.
	BCLR		_LATE,#LATE2			; LVP buffer direction B => A.
	REPEAT		#15						;
	NOP									;

	; ACK Clock
	CLOCK		BSET					; Set the clock high for 875nS.
	REPEAT		#9						; (14 instructions)
	NOP									;

	BTSC        PORTF, #3				; Read SDA pin
	BSET		_asm_temp2, #0			; ACK was received
	CLOCK		BCLR					; Set the clock low.

	RETURN

/******************************************************************************
 * Function:        void I2C_WriteSlowAsm(unsigned char outputbyte, WORD numbits)
 *
 * Overview:        Clocks out a byte with PGC = SCL and LVP = SDA.
 *                  Checks for ACK
 *
 * PreCondition:    PGC = output, LVP = input
 *
 * Input:           outputbyte = byte to be written MSB first
 *
 * Output:          Pk3Status.ICDTimeOut set if NACK received
 *
 * Side Effects:    None
 *
 * Note:            PK3 has a 4.7k pull-down resisitor and a Transil on LVP,
 *					so we cannot support writing I2C devices using pull-ups.
 *					(WE CANNOT SUPPORT I2C DEVICES WITH CURRENT H/W DESIGN)
 *****************************************************************************/
_I2C_WriteSlowAsm:

	; Write out 0's driving SDA low if MSB = 0.
	BTSS		W0, #7					; Check if the MSB is set.
	BCLR		_TRISF,#TRISF3			; if MSB = 0 LVP pin is output.
	BTSS		W0, #7					; Check if the MSB is set.
	BSET		_LATE,#LATE2			; if MSB = 0 LVP buffer direction A=>B.

	; Write out 1's using pullup on SDA if MSB = 1.
	BTSC		W0, #7					; Check if the MSB is clear.
	BSET		_TRISF,#TRISF3			; if MSB = 1 LVP pin is input.
	BTSC		W0, #7					; Check if the MSB is clear.
	BCLR		_LATE,#LATE2			; if MSB = 1 LVP buffer direction B=>A.

	REPEAT		#41						; 2.7us - 3.0us setup time
	NOP									; (44-48 instructions)

	CLOCK		BSET					; Set the clock high for 3.5us.
	REPEAT		#53						; (56 instructions)
	NOP									;

	CLOCK		BCLR					; Set the clock low for 6.5us.
	REPEAT		#46						; (104 instructions)
	NOP									;

	RLNC.B		W0, W0					; Shift one position to the left.
	DEC			W1, W1					; decrement counter
	BRA			NZ, _I2C_WriteSlowAsm	; loop if not zero

	; Release SDA for ACK.
	BSET		_TRISF,#TRISF3			; LVP pin set to input.
	BCLR		_LATE,#LATE2			; LVP buffer direction B => A.
	REPEAT		#48						;
	NOP									;

	; ACK Clock
	CLOCK		BSET					; Set the clock high for 3.5us.
	REPEAT		#51						; (56 instructions)
	NOP

	BTSC        PORTF, #3				; Read SDA pin
	BSET		_asm_temp2, #0			; ACK was received
	CLOCK		BCLR					; Set the clock low.

	RETURN

/******************************************************************************
 * Function:        unsigned char I2C_ReadFastAsm(unsigned char readByte,
 *                                                WORD numbits,
 *												  unsigned char giveack)
 *
 * Overview:        Clocks in a byte with PGC = SCL and LVP = SDA.
 *                  Provides and ACK for the byte if giveack is 0.
 *
 * PreCondition:    PGC = output, LVP = input
 *
 * Input:           readByte, numbits, giveack - ACK the byte if 0, else NACK
 *
 * Output:          Returns byte read MSB first.
 *
 * Side Effects:    None
 *
 * Note:            PK3 has a 4.7k pull-down resisitor and a Transil on LVP,
 *					so we cannot support reading I2C devices using pull-ups.
 *					(WE CANNOT SUPPORT I2C DEVICES WITH CURRENT H/W DESIGN)
 *****************************************************************************/
_I2C_ReadFastAsm:
	RLNC.B      W0, W0					; Shift one position to the left.

	REPEAT		#8						; 687.5ns setup time
	NOP									; (11 instructions)

	CLOCK		BSET					; Set the clock high for 875nS.
	REPEAT		#9						; (14 instructions)
	NOP									;

    BTSC        PORTF, #3				; Read SDA pin
    BSET        W0, #0					; If data in = 1 set LSB.

	CLOCK		BCLR					; Set the clock low for 1625nS.
	REPEAT		#9						; (26 instructions)
	NOP									;

	DEC			W1, W1					; decrement counter
	BRA			NZ, _I2C_ReadFastAsm	; loop if not zero

	LVP			BCLR					; Ensure LAT bit is initialized to zero.
	BTSS		W2, #0					; Test if ACK should be sent
	BCLR		_TRISF,#TRISF3			; if 0, LVP pin is output (ACK).
	BTSS		W2, #0					; Check if the MSB is set.
	BSET		_LATE,#LATE2			; if MSB = 0 LVP buffer direction A=>B.

	; ACK Clock
	REPEAT		#5						; 687.5ns setup time
	NOP									; (11 instructions)

	CLOCK		BSET					; Set the clock high for 875nS.
	REPEAT		#11						; (14 instructions)
	NOP									;

	CLOCK		BCLR					; Set the clock low.
	REPEAT		#4						; Just keeping the clock low for a few
	NOP									; cycles before returning from fuction.

	RETURN

/******************************************************************************
 * Function:        unsigned char I2C_ReadSlowAsm(unsigned char readByte,
 *                                                WORD numbits,
 *												  unsigned char giveack)
 *
 * Overview:        Clocks in a byte with PGC = SCL and LVP = SDA.
 *
 * PreCondition:    PGC = output, LVP = input
 *
 * Input:           readByte, numbits, giveack - ACK the byte if 0, else NACK
 *
 * Output:          Returns byte read MSB first.
 *
 * Side Effects:    None
 *
 * Note:            PK3 has a 4.7k pull-down resisitor and a Transil on LVP,
 *					so we cannot support reading I2C devices using pull-ups.
 *					(WE CANNOT SUPPORT I2C DEVICES WITH CURRENT H/W DESIGN)
 *****************************************************************************/
_I2C_ReadSlowAsm:

	RLNC.B      W0, W0					; Shift one position to the left.

	REPEAT		#42						; 2.75us setup time
	NOP									; (44 instructions)

	CLOCK		BSET					; Set the clock high for 3.5us.
	REPEAT		#51						; (56 instructions)
	NOP									;

    BTSC        PORTF, #3				; Read SDA pin
    BSET        W0, #0					; If data in = 1 set LSB.

	CLOCK		BCLR					; Set the clock low for 6.5us.
	REPEAT		#53						; (104 instructions)
	NOP									;

	DEC			W1, W1					; decrement counter
	BRA			NZ, _I2C_ReadSlowAsm	; loop if not zero

	LVP			BCLR					; Ensure LAT bit is initialized to zero.
	BTSS		W2, #0					; Test if ACK should be sent
	BCLR		_TRISF,#TRISF3			; if 0, LVP pin is output (ACK).
	BTSS		W2, #0					; Check if the MSB is set.
	BSET		_LATE,#LATE2			; if MSB = 0 LVP buffer direction A=>B.

	; ACK Clock
	REPEAT		#39						; 2.750us setup time
	NOP									; (44 instructions)

	CLOCK		BSET					; Set the clock high for 3.5uS.
	REPEAT		#53						; (56 instructions)
	NOP									;

	CLOCK		BCLR					; Set the clock low.
	REPEAT		#16						; Just keeping the clock low for a few
	NOP									; cycles before returning from fuction.

	RETURN

/******************************************************************************
* End of file
******************************************************************************/
