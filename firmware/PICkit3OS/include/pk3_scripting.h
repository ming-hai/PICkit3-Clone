/*********************************************************************
 *
 * Main scripting engine header file
 *
 *********************************************************************
 * FileName:            pk3_scripting.h
 * Dependencies:
 * Processor:           PIC24
 * Assembler/Compiler:  MPLAB XC16 1.00
 * Linker:              MPLAB XC16 1.00
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

#ifndef PK3_SCR_DOT_H
#define PK3_SCR_DOT_H
//********************************************************************
// Defines
//********************************************************************
#define BUF_SIZE        64          // USB buffers

#define SCRIPTBUF_SIZE  768         // Script buffer size
#define SCRIPT_ENTRIES  32          // Script Table entries
#define SCRIPT_MAXLEN   61          // maximum length of script
#define SCRIPTRSV_SIZE  0           // size of reserved memory at end of script buffer (may be used for canned scripts)
#define SCRIPTBUFSPACE  (SCRIPTBUF_SIZE - SCRIPTRSV_SIZE)

#define DOWNLOAD_SIZE   256         // download buffer size
#define UPLOAD_SIZE     128         // upload buffer size

// SCRIPT CONTROL BYTE DEFINITIONS
#define VDD_ON              0xFF
#define VDD_OFF             0xFE
#define VDD_GND_ON          0xFD
#define VDD_GND_OFF         0xFC
#define VPP_ON              0xFB
#define VPP_OFF             0xFA
#define VPP_PWM_ON          0xF9
#define VPP_PWM_OFF         0xF8
#define MCLR_GND_ON         0xF7
#define MCLR_GND_OFF        0xF6
#define BUSY_LED_ON         0xF5
#define BUSY_LED_OFF        0xF4
#define SET_ICSP_PINS       0xF3
#define WRITE_BYTE_LITERAL  0xF2
#define WRITE_BYTE_BUFFER   0xF1
#define READ_BYTE_BUFFER    0xF0
#define READ_BYTE           0xEF
#define WRITE_BITS_LITERAL  0xEE
#define WRITE_BITS_BUFFER   0xED
#define READ_BITS_BUFFER    0xEC
#define READ_BITS           0xEB
#define SET_ICSP_SPEED      0xEA
#define LOOP                0xE9
#define DELAY_LONG          0xE8
#define DELAY_SHORT         0xE7
#define IF_EQ_GOTO          0xE6
#define IF_GT_GOTO          0xE5
#define GOTO_INDEX          0xE4
#define EXIT_SCRIPT         0xE3
#define PEEK_SFR            0xE2
#define POKE_SFR            0xE1
#define ICDSLAVE_RX         0xE0
#define ICDSLAVE_TX_LIT     0xDF
#define ICDSLAVE_TX_BUF     0xDE
#define LOOPBUFFER          0xDD
#define ICSP_STATES_BUFFER  0xDC
#define POP_DOWNLOAD        0xDB
#define COREINST18          0xDA
#define COREINST24          0xD9
#define NOP24               0xD8
#define VISI24              0xD7
#define RD2_BYTE_BUFFER     0xD6
#define RD2_BITS_BUFFER     0xD5
#define WRITE_BUFWORD_W     0xD4
#define WRITE_BUFBYTE_W     0xD3
#define CONST_WRITE_DL      0xD2
#define WRITE_BITS_LIT_HLD  0xD1
#define WRITE_BITS_BUF_HLD  0xD0
#define SET_AUX             0xCF
#define AUX_STATE_BUFFER    0xCE
#define I2C_START           0xCD
#define I2C_STOP            0xCC
#define I2C_WR_BYTE_LIT     0xCB
#define I2C_WR_BYTE_BUF     0xCA
#define I2C_RD_BYTE_ACK     0xC9
#define I2C_RD_BYTE_NACK    0xC8
#define SPI_WR_BYTE_LIT     0xC7
#define SPI_WR_BYTE_BUF     0xC6
#define SPI_RD_BYTE_BUF     0xC5
#define SPI_RDWR_BYTE_LIT   0xC4
#define SPI_RDWR_BYTE_BUF   0xC3
#define ICDSLAVE_RX_BL      0xC2
#define ICDSLAVE_TX_LIT_BL  0xC1
#define ICDSLAVE_TX_BUF_BL  0xC0
#define MEASURE_PULSE       0xBF
#define UNIO_TX             0xBE
#define UNIO_TX_RX          0xBD
#define JT2_SETMODE         0xBC
#define JT2_SENDCMD         0xBB
#define JT2_XFERDATA8_LIT   0xBA
#define JT2_XFERDATA32_LIT  0xB9
#define JT2_XFRFASTDAT_LIT  0xB8
#define JT2_XFRFASTDAT_BUF  0xB7
#define JT2_XFERINST_BUF    0xB6
#define JT2_GET_PE_RESP     0xB5
#define JT2_WAIT_PE_RESP    0xB4
#define JT2_PE_PROG_RESP    0xB3
#define CONST_WRITE_DL2     0xB2

// MPLAB Commands
#define cmd_GETVERSIONS_MPLAB   0x41

#define ACK_BYTE            0x00
#define NO_ACK_BYTE         0x0F

#define STATUSHI_ERRMASK    0xFE

#define PK2GO_MODE_OFF      0x00
#define PK2GO_MODE_LEARN    0x01
#define PK2GO_MODE_GO       0x02

#define CONNECT_TIMEOUT     20000       // Taken from PK3

#define NOPSPERCOUNT        6           // how many NOPs per one count of icsp_baud
#define NOPSPERCOUNT_HOLD   9           // This is for the clock pulse width for HCS devices.
#define NOPSPERCOUNT_SPI    7           // This is for a slow clock rate for SPI devices.

//********************************************************************
// Local Functions
//********************************************************************
void ScriptEngine(unsigned char *scriptstart_ptr, unsigned char scriptlength);
extern void SetVDDPowerSense(typVddSense state);
void SendVddVppUSB(void);
void ClearUploadBuffer(void);
void ShortDelay(unsigned char count);
void LongDelay(unsigned char count);
void ShiftBitsOutICSP(WORD outputbyte, WORD numbits);
void ShiftBitsOutICSPAsm(WORD outputbyte, WORD numbits, WORD repeat);
void ShiftBitsOutICSPHold(WORD outputbyte, WORD numbits);
void ShiftBitsOutICSPHoldAsm(WORD outputbyte, WORD numbits, WORD repeat);
unsigned char ShiftBitsInPIC24(WORD numbits);
unsigned char ShiftBitsInPIC24Asm(WORD numbits, WORD repeat);
unsigned char ShiftBitsInICSP(WORD numbits);
unsigned char ShiftBitsInICSPAsm(WORD numbits, WORD repeat);
void SetICSP_PinStates(unsigned char icsp_byte);
BYTE GetICSP_PinStates(void);
void SetAUX_PinState(unsigned char aux_byte);
BYTE GetAUX_PinState(void);
unsigned char ICDSlave_Receive (void);
void ICDSlave_transmit (unsigned char TransmitByte);
void WriteUploadBuffer(unsigned char byte2write);
unsigned char ReadDownloadBuffer(void);
void ReadUploadDataBuffer(void);
void P32GetPEResponse(unsigned char savedata, unsigned char execute);
void P32SendCommand (unsigned char command);
void P32SetMode (unsigned char numbits, unsigned char value);
unsigned char P32DataIO (unsigned char numbits, unsigned char tdi, unsigned char tms);
void JTAG2W4PH(void);
void P32XferData8 (unsigned char byte0);
unsigned char P32XferData32 (unsigned char byte3, unsigned char byte2, unsigned char byte1, unsigned char byte0, char rxdata);
void P32XferFastData32(unsigned char byte3, unsigned char byte2, unsigned char byte1, unsigned char byte0);
void P32XferInstruction(void);
void SendStatusUSB(void);
void ClearScriptTable(void);
void StoreScriptInBuffer(WORD *usbindex);
void RunScript(unsigned char scriptnumber, unsigned char repeat);
void SendScriptChecksumsUSB(void);
void ClearDownloadBuffer(void);
void WriteDownloadDataBuffer(WORD *usbindex);
void ReadUploadDataBufferNoLength(void);
void WriteByteDownloadBuffer(unsigned char DataByte);
void LogicAnalyzer(void);
WORD LogicAnalyzerAsm(WORD RiseFallMask, WORD SamplingDelay);
WORD ConvertMaskstoPK3(BYTE mask);
void CopyRamUpload(unsigned char addrl, unsigned char addrh);
BYTE ConvertSampletoPK2(BYTE val);
BYTE SwapByte(BYTE val);
unsigned char SPI_ReadWrite(unsigned char outputbyte);
unsigned char SPI_ReadWriteFastAsm(unsigned char outputbyte, WORD numbits);
unsigned char SPI_ReadWriteSlowAsm(unsigned char outputbyte, WORD numbits, WORD repeat);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_Write(unsigned char outputbyte);
void I2C_WriteFastAsm(unsigned char outputbyte, WORD numbits);
void I2C_WriteSlowAsm(unsigned char outputbyte, WORD numbits);
unsigned char I2C_Read(unsigned char giveack);
unsigned char I2C_ReadFastAsm(unsigned char readByte, WORD numbits, unsigned char giveack);
unsigned char I2C_ReadSlowAsm(unsigned char readByte, WORD numbits, unsigned char giveack);
void UNIO (unsigned char device_addr, unsigned char txbytes, unsigned char rxbytes);
void WriteInternalEEPROM(WORD *usbindex);
void ReadInternalEEPROM(WORD *usbindex);
void EE_WriteByte(unsigned char byte_address, unsigned char write_byte);
unsigned char EE_ReadByte(unsigned char byte_address);
void SendFWVersionUSB(void);
void TestBootloaderExists();

//********************************************************************
// External globals
//********************************************************************
extern POWER_INFO gPowerInfo;
extern BYTE _gblIntRamStorage[];
extern BYTE *   inbuffer;
extern BYTE *   outbuffer;
extern SYSVOLTAGES gblSysVoltages;
/********************************************/

#endif //PK3_SCR_DOT_H
