/*********************************************************************
 *
 * Main scripting engine and functions
 *
 *********************************************************************
 * FileName:            pk3_scripting.c
 * Dependencies:        pickit3.h pk3_scripting.h
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

#include "pickit3.h"
#include "pk3_scripting.h"
#include <libpic30.h>

//********************************************************************
// Defines
//********************************************************************
#define BOOTMODE            'B'     // Enter bootloader mode
#define NO_OPERATION        'Z'     // does nothing

#define SWITCH_TO_BL        0x24
#define TEST_BOOTLOADER     0x2A

#define SETVDD              0xA0
#define SETVPP              0xA1
#define READ_STATUS         0xA2
#define READ_VOLTAGES       0xA3
#define DOWNLOAD_SCRIPT     0xA4
#define RUN_SCRIPT          0xA5
#define EXECUTE_SCRIPT      0xA6
#define CLR_DOWNLOAD_BUFFER 0xA7
#define DOWNLOAD_DATA       0xA8
#define CLR_UPLOAD_BUFFER   0xA9
#define UPLOAD_DATA         0xAA
#define CLR_SCRIPT_BUFFER   0xAB
#define UPLOAD_DATA_NOLEN   0xAC
#define END_OF_BUFFER       0xAD
#define RESET               0xAE
#define SCRIPT_BUFFER_CHKSM 0xAF
#define SET_VOLTAGE_CALS    0xB0
#define WR_INTERNAL_EE      0xB1
#define RD_INTERNAL_EE      0xB2
#define ENTER_UART_MODE     0xB3
#define EXIT_UART_MODE      0xB4
#define ENTER_LEARN_MODE    0xB5
#define EXIT_LEARN_MODE     0xB6
#define ENABLE_PK2GO_MODE   0xB7
#define LOGIC_ANALYZER_GO   0xB8
#define COPY_RAM_UPLOAD     0xB9
#define DUMMY_COMMAND       0xBA    // used for debugging and setting breakpoints
// META-COMMANDS
#define READ_OSCCAL         0x80
#define WRITE_OSCCAL        0x81
#define START_CHECKSUM      0x82
#define VERIFY_CHECKSUM     0x83
#define CHECK_DEVICE_ID     0x84
#define READ_BANDGAP        0x85
#define WRITE_CFG_BANDGAP   0x86
#define CHANGE_CHKSM_FRMT   0x87

//********************************************************************
// Globals
//********************************************************************
BYTE asm_temp1;
BYTE asm_temp2;
BYTE asm_temp3;
BYTE asm_temp4;
BYTE asm_temp5;
BYTE asm_temp6;
BYTE asm_temp7;
BYTE asm_temp8;
BYTE asm_temp9;
BYTE asm_temp10;
BYTE asm_temp11;
BYTE asm_temp12;
BYTE asm_temp13;

WORD cnvTrigMask;
WORD cnvTrigStates;
WORD cnvEdgeMask;
WORD TrigCount;
WORD PostTrigCount;

unsigned char icsp_pins;
unsigned char icsp_baud;
unsigned char aux_pin;

struct {
    unsigned int    write_index;        // buffer write index
    unsigned int    read_index;         // buffer read index
    unsigned int    used_bytes;         // # bytes in buffer
} downloadbuf_mgmt;

union {     // Status bits
    struct {
        unsigned char   StatusLow;
        unsigned char   StatusHigh;
    };
    struct{
        // StatusLow
        unsigned VddGNDOn:1;    // bit 0
        unsigned VddOn:1;
        unsigned VppGNDOn:1;
        unsigned VppOn:1;
        unsigned VddError:1;
        unsigned VppError:1;
        unsigned ButtonPressed:1;
        unsigned :1;
        //StatusHigh
        unsigned Reset:1;       // bit 0
        unsigned UARTMode:1;
        unsigned ICDTimeOut:1;
        unsigned UpLoadFull:1;
        unsigned DownloadEmpty:1;
        unsigned EmptyScript:1;
        unsigned ScriptBufOvrFlow:1;
        unsigned DownloadOvrFlow:1;
    };
} Pk3Status;

struct {
    unsigned char   write_index;        // buffer write index
    unsigned char   read_index;         // buffer read index
    unsigned int    used_bytes;         // # bytes in buffer
} uploadbuf_mgmt;

struct script_table_t{                      // Script table - keeps track of scripts in the Script Buffer.
    unsigned char   Length;
    int StartIndex; // offset from uc_script_buffer[0] of beginning of script.
};

struct script_table_t * ScriptTable;

BYTE *  uc_download_buffer; // Download Data Buffer
BYTE *  uc_upload_buffer;       // Upload Data Buffer
BYTE *  uc_script_buffer;       // Script Buffer
BYTE *  uc_ScriptBuf_ptr;

BYTE *  inbuffer;
BYTE *  outbuffer;

//********************************************************************
// External globals
//********************************************************************
extern bool _ButtonChanged;

/***************************************************/

/******************************************************************************
 * Function:        void InitScripting()
 *
 * Overview:        Initializes buffer locations and sizes
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
void InitScripting()
{
    uc_script_buffer = _gblIntRamStorage;
    uc_ScriptBuf_ptr = uc_script_buffer;
    uc_download_buffer = uc_script_buffer + SCRIPTBUF_SIZE;
    uc_upload_buffer = uc_download_buffer + DOWNLOAD_SIZE;
    inbuffer = uc_upload_buffer + UPLOAD_SIZE;
    outbuffer = inbuffer + BUF_SIZE;
    ScriptTable = (struct script_table_t *) (outbuffer + BUF_SIZE);
}

/******************************************************************************
 * Function:        void ScriptingHandler()
 *
 * Overview:        Main PICkit 3 script handling loop
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
void ScriptingHandler()
{

    WORD usb_idx = 0, temp;

    USBGet(inbuffer, BUF_SIZE);

    do
    {
         asm_temp1 = inbuffer[usb_idx++];

        switch(asm_temp1)        // parse buffer for commands
        {
            case NO_OPERATION:          // Do nothing
            // format:      0x5A
            break;

            case cmd_GETVERSIONS_MPLAB:
            // format:      0x41 0x00
            if (inbuffer[usb_idx]==0)
            {
                SendFWVersionUSB();
            }
            usb_idx=64;

            break;

            case SWITCH_TO_BL:
            // format:      0x24 0x00
            if (inbuffer[usb_idx]==0)
            {
                SwitchToBootloader();
            }
            usb_idx=64;

            break;

            case TEST_BOOTLOADER:
            // format:      0x2A 0x00
            if (inbuffer[usb_idx]==0)
            {
                TestBootloaderExists();
            }
            usb_idx=64;

            break;

            case SETVDD:
            // format:      0xA0 <VDDL><VDDH> where VDD = vdd desired value / 0.125
            // response:    -
            temp = inbuffer[usb_idx] + (((WORD)inbuffer[usb_idx+1])<<8);
            SetVdd(temp);
            usb_idx += 2;
            break;

            case SETVPP:
            // format:      0xA1 <VPPL><VPPH>
            // response:    -
            temp = inbuffer[usb_idx] + (((WORD)inbuffer[usb_idx+1])<<8);
            SetVpp(temp);
            usb_idx += 2;
            break;

            case READ_STATUS:       // 0xA2
                SendStatusUSB();
                break;

            case READ_VOLTAGES:     // 0xA3
                SendVddVppUSB();
                break;

            case DOWNLOAD_SCRIPT:   // Store a script in the Script Buffer
                // format:      0xA4 <Script#><ScriptLengthN><Script1><Script2>....<ScriptN>
                // response:    -
                StoreScriptInBuffer(&usb_idx);
                break;

            case RUN_SCRIPT:    // run a script from the script buffer
                // format:      0xA5 <Script#><iterations>
                // response:    -
                RunScript(inbuffer[usb_idx], inbuffer[usb_idx + 1]);
                usb_idx+=2;
                break;

            case EXECUTE_SCRIPT:    // immediately executes the included script
                // format:      0xA6 <ScriptLengthN><Script1><Script2>....<ScriptN>
                // response:    -
                ScriptEngine((unsigned char *) &inbuffer[usb_idx + 1], inbuffer[usb_idx]);
                usb_idx += (inbuffer[usb_idx] + 1);
                break;

            case CLR_DOWNLOAD_BUFFER:   // empties the download buffer
                // format:      0xA7
                // response:    -
                ClearDownloadBuffer();
                break;

            case DOWNLOAD_DATA: // add data to download buffer
                // format:      0xA8 <datalength><data1><data2>....<dataN>
                // response:    -
                WriteDownloadDataBuffer(&usb_idx);
                break;

            case CLR_UPLOAD_BUFFER:  // empties the upload buffer
                // format:      0xA9
                // response:    -
                ClearUploadBuffer();
                break;

            case UPLOAD_DATA:// reads data from upload buffer
                // format:      0xAA
                // response:    <DataLengthN><data1><data2>....<dataN>
                ReadUploadDataBuffer();
                break;

            case CLR_SCRIPT_BUFFER:
                // format:      0xAB
                // response:    -
                ClearScriptTable();
                break;

            case UPLOAD_DATA_NOLEN:// reads data from upload buffer
                // format:      0xAC
                // response:    <data1><data2>....<dataN>
                ReadUploadDataBufferNoLength();
                break;

            case SCRIPT_BUFFER_CHKSM://0xAF
                SendScriptChecksumsUSB();
                break;

            case ENTER_UART_MODE: // Puts the firmware in UART Mode
                // format:      0xB3 <BaudValueL><BaudValueH>
                //                   BaudValue = 65536 ï¿½ [((1/BAUD) ï¿½ 3e-6) / 1.67e-7]
                // response:    -
                //EnterUARTMode(&usb_idx);
                break;

            case EXIT_UART_MODE: // Exits the firmware from UART Mode
                // format:      0xB4
                // response:    -
                //ExitUARTMode();
                break;

            case LOGIC_ANALYZER_GO:
                // format:      0xB8<EdgeRising><TrigMask><TrigStates><EdgeMask>
                //                   <TrigCount><PostTrigCountL><PostTrigCountH><SampleRateFactor>
                // response:    <TrigLocL><TrigLocH>
                asm_temp12 = inbuffer[usb_idx++];
                asm_temp1 = inbuffer[usb_idx++];
                asm_temp3 = inbuffer[usb_idx++];
                asm_temp5 = inbuffer[usb_idx++];
                asm_temp6 = inbuffer[usb_idx++];
                asm_temp7 = inbuffer[usb_idx++];
                asm_temp8 = inbuffer[usb_idx++];
                asm_temp11 = inbuffer[usb_idx++];
                asm_temp13 = inbuffer[usb_idx++];
                LogicAnalyzer();
                break;

            case COPY_RAM_UPLOAD:
                // format:      0xB9<StartAddrL><StartAddrH>
                // response:    -
                CopyRamUpload(inbuffer[usb_idx], inbuffer[usb_idx+1]);
                usb_idx += 2;
                break;
                
            case DUMMY_COMMAND:
                NOP();
                break;

            case RESET:
                Reset();
                break;
                
            case END_OF_BUFFER: //0xAD
                usb_idx = 64;
                break;
                
            case WR_INTERNAL_EE:
            	// Write bytes to SPI Serial EEPROM
				// format:      0xB1 <address><datalength><data1><data2>....<dataN>
  				//              N = 32 Max
				// response:	-
				WriteInternalEEPROM(&usb_idx); // This is really the SPI device U3 on PK3.
				break;
				
            case RD_INTERNAL_EE:
				// Read bytes from SPI Serial EEPROM
				// format:      0xB2 <address><datalength>
				//                   N = 32 Max
				// response:	<data1><data2>....<dataN>
				ReadInternalEEPROM(&usb_idx); // This is really the SPI device U3 on PK3.
				break;

            // unimplemented commands
            case ENTER_LEARN_MODE:
            case SET_VOLTAGE_CALS:
                usb_idx += 4;
                break;

            case BOOTMODE:
            case EXIT_LEARN_MODE:
                break;
                
            case ENABLE_PK2GO_MODE:

            default:
            _HALT();
            usb_idx = 64;
            break;

        } // end switch


    } while(usb_idx < 64);
}

/******************************************************************************
 * Function:        void ScriptEngine(unsigned char *scriptstart_ptr, unsigned char scriptlength)
 *
 * Overview:        Executes the script pointed to by scriptstart_ptr from the Script Buffer
 *                  Aborts if a control byte attempts to use a byte from an empty Download buffer
 *                  or store a byte in a full Upload Buffer.  Will not execute if
 *                  Pk3Status.StatusHigh != 0 (a script error exists.)
 *
 * PreCondition:    None
 *
 * Input:           *scriptstart_ptr - Pointer to start of script
 *                  scriptlength - length of script
 *
 * Output:          uc_downloadbuf_read - advanced by the number of bytes read.
 *                  uc_upload_buffer[] - new data may be stored
 *                  uc_uploadbuf_write - advance by number of bytes written.
 *                  Pk3Status.StatusHigh - set if script error occurs
 *
 * Side Effects:    Uses Timer0.
 *
 * Note:            None
 *****************************************************************************/
void ScriptEngine(unsigned char *scriptstart_ptr, unsigned char scriptlength)
{
    BYTE scriptindex = 0;
    BYTE temp_byte;
    BYTE loopcount=0, loopindex=0, loopbufferindex=0;
    WORD loopbuffercount=0;
    BOOL loopactive = FALSE;
    BOOL loopbufferactive = FALSE;
    unsigned char *SFR_ptr;

    //INTCONbits.T0IE = 0; // ensure Timer0 interrupt diabled.
    //T0CON = 0x07;       // 16-bit timer, 1:256 prescale.

    if ((scriptlength == 0) || (scriptlength > SCRIPT_MAXLEN))
    {
        Pk3Status.EmptyScript = 1;      // set error - script length out of bounds
        return;
    }

    while (((Pk3Status.StatusHigh & STATUSHI_ERRMASK) == 0) && (scriptindex < scriptlength))
    {
        asm_temp1 = *(scriptstart_ptr + scriptindex); // script command

        if (asm_temp1 < 0xB2)   // minimum acceptable script number
        {
            scriptindex = scriptlength;;
            continue;
        }
//        if (asm_temp1 < 0xD5)
//        {
//          _asm
//                bra ScriptJumpTable2
//            _endasm
//        }


        switch (asm_temp1) {
            case RD2_BITS_BUFFER:
                scriptindex++;
                WriteUploadBuffer(ShiftBitsInPIC24(*(scriptstart_ptr + scriptindex)));
                scriptindex++;
                break;

            case RD2_BYTE_BUFFER:
                scriptindex++;
                WriteUploadBuffer(ShiftBitsInPIC24(8));
                break;

            case VISI24:
                scriptindex++;
                ShiftBitsOutICSP(1, 4);
                ShiftBitsOutICSP(0, 8);
                WriteUploadBuffer(ShiftBitsInPIC24(8));
                WriteUploadBuffer(ShiftBitsInPIC24(8));
                break;

            case NOP24:
                scriptindex++;
            ShiftBitsOutICSP(0, 4);
            ShiftBitsOutICSP(0, 8);
            ShiftBitsOutICSP(0, 8);
            ShiftBitsOutICSP(0, 8);
                break;

            case COREINST24:
                scriptindex++;
                ShiftBitsOutICSP(0, 4);
                ShiftBitsOutICSP(*(scriptstart_ptr + scriptindex++), 8);
                ShiftBitsOutICSP(*(scriptstart_ptr + scriptindex++), 8);
                ShiftBitsOutICSP(*(scriptstart_ptr + scriptindex++), 8);
                break;

            case COREINST18:
                scriptindex++;
                ShiftBitsOutICSP(0, 4);
                ShiftBitsOutICSP(*(scriptstart_ptr + scriptindex++), 8);
                ShiftBitsOutICSP(*(scriptstart_ptr + scriptindex++), 8);
                break;

            case POP_DOWNLOAD:
                ReadDownloadBuffer();
                scriptindex++;
                break;

            case ICSP_STATES_BUFFER:
                WriteUploadBuffer(GetICSP_PinStates());
                scriptindex++;
                break;

            case LOOPBUFFER:
                if (loopbufferactive)
                {
                    loopbuffercount--;
                    if (loopbuffercount == 0)
                    {
                        loopbufferactive = FALSE;
                        scriptindex+=2;
                        break;
                    }
                    scriptindex = loopbufferindex;
                    break;
                }
                loopbufferindex = scriptindex - *(scriptstart_ptr + scriptindex + 1);
                loopbuffercount = (unsigned int) ReadDownloadBuffer();  // low byte
                loopbuffercount += (256 * ReadDownloadBuffer());        // upper byte
                if (loopbuffercount == 0)
                { // value of "zero" 0x0000 means no loops.
                    scriptindex+=2;
                    break;
                }
                loopbufferactive = TRUE;
                scriptindex = loopbufferindex;
                break;

            case EXIT_SCRIPT:
                scriptindex = scriptlength;
                break;

            case GOTO_INDEX:
                scriptindex = scriptindex + (signed char)*(scriptstart_ptr + scriptindex + 1);
                break;

            case IF_GT_GOTO:
                temp_byte = uc_upload_buffer[uploadbuf_mgmt.write_index - 1]; // last byte written
            if (temp_byte > *(scriptstart_ptr + scriptindex + 1))
            {
                scriptindex = scriptindex + (signed char)*(scriptstart_ptr + scriptindex + 2);
            }
            else
            {
                scriptindex+=3;
            }
                break;

            case IF_EQ_GOTO:
                temp_byte = uc_upload_buffer[uploadbuf_mgmt.write_index - 1]; // last byte written
            if (temp_byte == *(scriptstart_ptr + scriptindex + 1))
            {
                scriptindex = scriptindex + (signed char)*(scriptstart_ptr + scriptindex + 2);
            }
            else
            {
                scriptindex+=3;
            }
                break;

            case DELAY_SHORT:
                scriptindex++;
                ShortDelay(*(scriptstart_ptr + scriptindex));
                scriptindex++;
                break;

            case DELAY_LONG:
                scriptindex++;
                LongDelay(*(scriptstart_ptr + scriptindex));
                scriptindex++;
                break;

            case LOOP:
                if (loopactive)
                {
                    loopcount--;
                    if (loopcount == 0)
                    {
                        loopactive = FALSE;
                        scriptindex+=3;
                        break;
                    }
                    scriptindex = loopindex;
                    break;
                }
                loopactive = TRUE;
                loopindex = scriptindex - *(scriptstart_ptr + scriptindex + 1);
                loopcount = *(scriptstart_ptr + scriptindex + 2);
                scriptindex = loopindex;
                break;

            case SET_ICSP_SPEED:
                scriptindex++;
                icsp_baud = *(scriptstart_ptr + scriptindex);
                scriptindex++;
                break;

            case READ_BITS:
                scriptindex++;
                ShiftBitsInICSP(*(scriptstart_ptr + scriptindex));
                scriptindex++;
                break;

            case READ_BITS_BUFFER:
                scriptindex++;
                WriteUploadBuffer(ShiftBitsInICSP(*(scriptstart_ptr + scriptindex)));
                scriptindex++;
                break;

            case WRITE_BITS_BUFFER:
                scriptindex++;
                ShiftBitsOutICSP(ReadDownloadBuffer(), *(scriptstart_ptr + scriptindex));
                scriptindex++;
                break;

            case WRITE_BITS_LITERAL:
                scriptindex++;
                ShiftBitsOutICSP(*(scriptstart_ptr + scriptindex + 1), *(scriptstart_ptr + scriptindex));
                scriptindex+=2;
                break;

            case READ_BYTE:
                ShiftBitsInICSP(8);
                scriptindex++;
                break;

            case READ_BYTE_BUFFER:
                WriteUploadBuffer(ShiftBitsInICSP(8));
                scriptindex++;
                break;

            case WRITE_BYTE_BUFFER:
                ShiftBitsOutICSP(ReadDownloadBuffer(), 8);
                scriptindex++;
                break;

            case WRITE_BYTE_LITERAL:
                scriptindex++;
                ShiftBitsOutICSP(*(scriptstart_ptr + scriptindex), 8);
                scriptindex++;
                break;

            case SET_ICSP_PINS:
                scriptindex++;
                icsp_pins = *(scriptstart_ptr + scriptindex);
                SetICSP_PinStates(icsp_pins);
                scriptindex++;
                break;

            case BUSY_LED_OFF:
                SetLED_IDLE();
                scriptindex++;
                break;

            case BUSY_LED_ON:
                SetLED_BUSY();
                scriptindex++;
                break;

            case MCLR_GND_OFF:
                pin_VPP_GROUND = 0;
                scriptindex++;
                break;

            case MCLR_GND_ON:
                pin_VPP_GROUND = 1;
                scriptindex++;
                break;

            case VPP_PWM_OFF:
                TURN_OFF_VPP_SUPPLY;
                pin_VPP_PUMP = 0;
                scriptindex++;
                break;

            case VPP_PWM_ON:
                TURN_ON_VPP_SUPPLY;
                scriptindex++;
                break;

            case VPP_OFF:
                pin_VPP_ON = 0;
                scriptindex++;
                break;

            case VPP_ON:
                pin_VPP_ON = 1;
                scriptindex++;
                break;

            case VDD_GND_OFF:
                pin_INT_NVDD_GND = 1;
                scriptindex++;
                break;

            case VDD_GND_ON:
                pin_INT_NVDD_GND = 0;
                scriptindex++;
                break;

            case VDD_OFF:
                pin_SUPPLY_PWR = 1;
                scriptindex++;
                break;

            case VDD_ON:
                pin_SUPPLY_PWR = 0;
                scriptindex++;
                break;

            case JT2_PE_PROG_RESP: //0xB3
                scriptindex++;
                P32GetPEResponse(0, 0);
                break;

            case JT2_WAIT_PE_RESP:
                scriptindex++;
                P32GetPEResponse(0, 1);
                break;

            case JT2_GET_PE_RESP:
                scriptindex++;
                P32GetPEResponse(1, 1);
                break;

            case JT2_XFERINST_BUF:
                scriptindex++;
                P32XferInstruction();
                break;

            case JT2_XFRFASTDAT_BUF:
                scriptindex++;
                P32XferFastData32(ReadDownloadBuffer(), ReadDownloadBuffer(), ReadDownloadBuffer(), ReadDownloadBuffer());
                break;

            case JT2_XFRFASTDAT_LIT:
                scriptindex++;
                P32XferFastData32(*(scriptstart_ptr + scriptindex + 3), *(scriptstart_ptr + scriptindex + 2), *(scriptstart_ptr + scriptindex + 1), *(scriptstart_ptr + scriptindex));
                scriptindex += 4;
                break;

            case JT2_XFERDATA32_LIT:
                scriptindex++;
                P32XferData32(*(scriptstart_ptr + scriptindex + 3), *(scriptstart_ptr + scriptindex + 2), *(scriptstart_ptr + scriptindex + 1), *(scriptstart_ptr + scriptindex), 1);
                scriptindex += 4;
                break;

            case JT2_XFERDATA8_LIT:
                scriptindex++;
                P32XferData8(*(scriptstart_ptr + scriptindex));
                scriptindex++;
                break;

            case JT2_SENDCMD:
                 scriptindex++;
                P32SendCommand(*(scriptstart_ptr + scriptindex));
                scriptindex++;
                break;

            case JT2_SETMODE:
                scriptindex++;
                P32SetMode(*(scriptstart_ptr + scriptindex), *(scriptstart_ptr + scriptindex + 1));
                scriptindex += 2;
                break;
                
            case UNIO_TX_RX:
                scriptindex++;
                UNIO(*(scriptstart_ptr + scriptindex), *(scriptstart_ptr + scriptindex + 1), *(scriptstart_ptr + scriptindex + 2));
                scriptindex += 3;
                break;
                
            case UNIO_TX:
                scriptindex++;
                UNIO(*(scriptstart_ptr + scriptindex), *(scriptstart_ptr + scriptindex + 1), 0);
                scriptindex+= 2;
                break;
                
            case SPI_RDWR_BYTE_BUF:
                scriptindex++;
                WriteUploadBuffer(SPI_ReadWrite(ReadDownloadBuffer()));
                break;

            case SPI_RDWR_BYTE_LIT:
                scriptindex++;
                WriteUploadBuffer(SPI_ReadWrite(*(scriptstart_ptr + scriptindex)));
                scriptindex++;
                break;

            case SPI_RD_BYTE_BUF:
                WriteUploadBuffer(SPI_ReadWrite(0));
                scriptindex++;
                break;

            case SPI_WR_BYTE_BUF:
                scriptindex++;
                SPI_ReadWrite(ReadDownloadBuffer());
                break;

            case SPI_WR_BYTE_LIT:
                scriptindex++;
                SPI_ReadWrite(*(scriptstart_ptr + scriptindex));
                scriptindex++;
                break;

            case I2C_RD_BYTE_NACK:
                WriteUploadBuffer(I2C_Read(NO_ACK_BYTE));
                scriptindex++;
                break;

            case I2C_RD_BYTE_ACK:
                WriteUploadBuffer(I2C_Read(ACK_BYTE));
                scriptindex++;
                break;

            case I2C_WR_BYTE_BUF:
                scriptindex++;
                I2C_Write(ReadDownloadBuffer());
                break;

            case I2C_WR_BYTE_LIT:
                scriptindex++;
                I2C_Write(*(scriptstart_ptr + scriptindex));
                scriptindex++;
                break;

            case I2C_STOP:
                I2C_Stop();
                scriptindex++;
                break;

            case I2C_START:
                I2C_Start();
                scriptindex++;
                break;

            case AUX_STATE_BUFFER:
                WriteUploadBuffer(GetAUX_PinState());
                scriptindex++;
                break;

            case SET_AUX:
                scriptindex++;
                aux_pin = *(scriptstart_ptr + scriptindex);
                SetAUX_PinState(aux_pin);
                scriptindex++;
                break;

            case WRITE_BITS_BUF_HLD: // This case is used for the HCS devices.
                scriptindex++;
                ShiftBitsOutICSPHold(ReadDownloadBuffer(), *(scriptstart_ptr + scriptindex));
			    scriptindex++;
                break;

            case WRITE_BITS_LIT_HLD: // This case is used for the HCS devices.
                scriptindex++;
                ShiftBitsOutICSPHold(*(scriptstart_ptr + scriptindex + 1), *(scriptstart_ptr + scriptindex));
			    scriptindex+=2;
                break;

            case CONST_WRITE_DL:
                scriptindex++;
                WriteByteDownloadBuffer(*(scriptstart_ptr + scriptindex));
                scriptindex++;
                break;

            case CONST_WRITE_DL2:
                scriptindex++;
                for(temp_byte = *(scriptstart_ptr + scriptindex); temp_byte > 0; temp_byte--)
                {
                    scriptindex++;
                    WriteByteDownloadBuffer(*(scriptstart_ptr + scriptindex));
                }
                break;

            case WRITE_BUFBYTE_W:
                scriptindex++;
                ShiftBitsOutICSP(0, 4); // six code
                ShiftBitsOutICSP(*(scriptstart_ptr + scriptindex), 4); // W nibble
                ShiftBitsOutICSP(ReadDownloadBuffer(), 8); // literal LSB
                ShiftBitsOutICSP(0, 8); // literal MSB
                ShiftBitsOutICSP(0x2, 4); // opcode
                scriptindex++;
                break;

            case WRITE_BUFWORD_W:      // 0xD4
                scriptindex++;
                ShiftBitsOutICSP(0, 4); // six code
                ShiftBitsOutICSP(*(scriptstart_ptr + scriptindex), 4); // W nibble
                ShiftBitsOutICSP(ReadDownloadBuffer(), 8); // literal LSB
                ShiftBitsOutICSP(ReadDownloadBuffer(), 8); // literal MSB
                ShiftBitsOutICSP(0x2, 4); // opcode
                scriptindex++;
                break;

            case ICDSLAVE_TX_BUF:
                scriptindex++;
                ICDSlave_transmit(ReadDownloadBuffer());
                break;

            case ICDSLAVE_TX_LIT:
                scriptindex++;
                ICDSlave_transmit(*(scriptstart_ptr + scriptindex++));
                break;

            case ICDSLAVE_RX:
                scriptindex++;
                WriteUploadBuffer(ICDSlave_Receive());
                break;

            case POKE_SFR:
                scriptindex++;
                SFR_ptr = (unsigned char *)0x0F00 + *(scriptstart_ptr + scriptindex);
                WriteUploadBuffer(*SFR_ptr);
                scriptindex++;
                break;

            case PEEK_SFR:
                scriptindex++;
                SFR_ptr = (unsigned char *)0x0F00 + *(scriptstart_ptr + scriptindex++);
                *SFR_ptr = *(scriptstart_ptr + scriptindex++);
                break;

            // unimplemented script commands:
 //           case UNIO_TX_RX:
 //               scriptindex ++;
 //           case UNIO_TX:
 //               scriptindex ++;
 //               scriptindex ++;
            case MEASURE_PULSE:
            case ICDSLAVE_TX_BUF_BL:
            case ICDSLAVE_TX_LIT_BL:
            case ICDSLAVE_RX_BL:
                scriptindex ++;
            default:
                // TODO
                _HALT();
                break;
       } // switch(asm_temp1)

    } // end;

} //end void ScriptEngine(unsigned char *scriptstart_ptr, unsigned char scriptlength)

/******************************************************************************
 * Function:        void ClearUploadBuffer(void)
 *
 * Overview:        Clears the Upload Buffer
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:
 *
 * Side Effects:
 *
 * Note:            None
 *****************************************************************************/
void ClearUploadBuffer(void)
{
    uploadbuf_mgmt.write_index = 0;     // init buffer to empty
    uploadbuf_mgmt.read_index = 0;
    uploadbuf_mgmt.used_bytes = 0;
}

/******************************************************************************
 * Function:        void SendVddVppUSB(void)
 *
 * Overview:        ADC converts VDD and VPP voltages and send results via USB.
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          Transmits HID Tx report .
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
void SendVddVppUSB(void)
{
    gblSysVoltages.VPP = GetVppInCounts();
    gblSysVoltages.VDD = gblSysVoltages.VDDTARGET = GetVddInCounts();

    USBPut((BYTE *)&gblSysVoltages, sizeof(SYSVOLTAGES));
} // end void SendVddVppUSB(void)

/******************************************************************************
 * Function:        void ShortDelay(unsigned char count)
 *
 * Overview:        Delays in increments of 21.3us * count.
 *
 * PreCondition:    None
 *
 * Input:           count - units of delay (21.3us each)
 *
 * Output:          None.
 *
 * Side Effects:    Uses Timer4/5
 *
 * Note:            Uses Timer4/5 to allow more accurate timing with interrupts.
 *
 *****************************************************************************/
void ShortDelay(unsigned char count)
{
    Delayus((213L * count)/10+1);
}

/******************************************************************************
 * Function:        void LongDelay(unsigned char count)
 *
 * Overview:        Delays in increments of 5.46ms * count.
 *
 * PreCondition:    None
 *
 * Input:           count - units of delay (5.46ms each)
 *
 * Output:          None.
 *
 * Side Effects:    Uses Timer4/5
 *
 * Note:            Uses Timer4/5 to allow more accurate timing with interrupts.
 *
 *****************************************************************************/
void LongDelay(unsigned char count)
{
    Delayus(5460L * count);
}

/******************************************************************************
 * Function:        void ICDSlave_transmit (unsigned char TransmitByte)
 *
 * Overview:        Handles handshake and slave transmission of a byte to debug exec.
 *
 * PreCondition:    None
 *
 * Input:           TransmitByte - byte to be clocked out by DE
 *
 * Output:          Pk3Status.ICDTimeOut if eight bits not clocked out in 500ms
 *
 * Side Effects:    Shuts off interrupts during execution - leaves ICDDATA pin as output.
 *
 * Note:            None
 *****************************************************************************/
void ICDSlave_transmit (unsigned char TransmitByte)
{
    // TODO: needed only in debugging via MLAB which is not supported under this mode.

    return;
} // end void ICDSlave_transmit (unsigned char TransmitByte)


/******************************************************************************
 * Function:        unsigned char ICDSlave_Receive (void)
 *
 * Overview:        Handles handshake and slave reception of a byte from debug exec.
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          Pk3Status.ICDTimeOut if eight bits not received in 500ms
 *                  Returns byte value received.
 *
 * Side Effects:    Shuts off interrupts during execution
 *
 * Note:            None
 *****************************************************************************/
unsigned char ICDSlave_Receive (void)
{
    // TODO: needed only in debugging via MLAB which is not supported under this mode.

    return asm_temp1;
} // end unsigned char ICDSlave_Receive (void)

/******************************************************************************
 * Function:        void ShiftBitsOutICSP(WORD outputbyte, WORD numbits)
 *
 * Overview:        Shifts the given # bits out on the ICSP pins
 *
 * PreCondition:    None
 *
 * Input:           outputbyte - byte to be shifted out LSB first
 *
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Note:            Assumes ICSP pins are already set to outputs.
 *****************************************************************************/
void ShiftBitsOutICSP(WORD outputbyte, WORD numbits)
{
    DISABLE_SYSTEM_INTERRUPT
    ShiftBitsOutICSPAsm(outputbyte, numbits, (icsp_baud<2)?0:((icsp_baud-1)*NOPSPERCOUNT));
    RESTORE_SYSTEM_INTERRUPT
}

/******************************************************************************
 * Function:        void ShiftBitsOutICSPHold(WORD outputbyte, WORD numbits)
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
 * Input:           outputbyte - byte to be shifted out LSB first
 *
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Note:            Assumes ICSP pins are already set to outputs.
 *****************************************************************************/
void ShiftBitsOutICSPHold(WORD outputbyte, WORD numbits)
{
    DISABLE_SYSTEM_INTERRUPT
    ShiftBitsOutICSPHoldAsm(outputbyte, numbits, (icsp_baud<2)?0:((icsp_baud-1)*NOPSPERCOUNT_HOLD));
    RESTORE_SYSTEM_INTERRUPT
}

/******************************************************************************
 * Function:        unsigned char ShiftBitsInPIC24(WORD numbits)
 *
 * Overview:        Shifts in up to a byte of data.  Shifts in LSB first.
 *                  If less than 8 bits, return byte is right justified.
 *                  If full, sets error Pk3Status.UpLoadFull
 *                  Data is latched on Rising Edge of clock
 * PreCondition:    None
 *
 * Input:           numbits - # bits to shift in (max 8)
 *
 * Output:          returns bits right-justified.
 *
 * Side Effects:    None
 *
 * Note:            Assumes ICSPCLK is output.  Sets ICSPDAT to input then restores
 *                  previous state.
 *****************************************************************************/
unsigned char ShiftBitsInPIC24(WORD numbits)
{
    DATA_DIR(IN_DIR);       // set input

    asm_temp1 = ShiftBitsInPIC24Asm(numbits, (icsp_baud<2)?0:((icsp_baud-1)*NOPSPERCOUNT));

    if (!(icsp_pins & 0x02))
    {
            DATA_DIR(OUT_DIR);
    }

    return asm_temp1;
}

/******************************************************************************
 * Function:        unsigned char ShiftBitsInICSP(WORD numbits)
 *
 * Overview:        Shifts in up to a byte of data.  Shifts in LSB first.
 *                  If less than 8 bits, return byte is right justified.
 * PreCondition:    None
 *
 * Input:           numbits - # bits to shift in (max 8)
 *
 * Output:          returns bits right-justified.
 *
 * Side Effects:    None
 *
 * Note:            Assumes ICSPCLK is output.  Sets ICSPDAT to input then restores
 *                  previous state.
 *****************************************************************************/
unsigned char ShiftBitsInICSP(WORD numbits)
{
    DATA_DIR(IN_DIR);       // set input

    asm_temp1 = ShiftBitsInICSPAsm(numbits, (icsp_baud<2)?0:((icsp_baud-1)*NOPSPERCOUNT));

    if (!(icsp_pins & 0x02))
    {
            DATA_DIR(OUT_DIR);
    }

    return asm_temp1;
}

/******************************************************************************
 * Function:        void WriteUploadBuffer(unsigned char byte2write)
 *
 * Overview:        Attempts to write a byte to the upload buffer.
 *                  If full, sets error Pk3Status.UpLoadFull
 * PreCondition:    None
 *
 * Input:           byte2write - byte to be written
 *
 * Output:          uc_upload_buffer - byte written to end of buffer.
 *
 * Side Effects:    Advances download buffer write pointer, if err Pk3Status.StatusHigh != 0
 *
 * Note:            None
 *****************************************************************************/
void WriteUploadBuffer(unsigned char byte2write)
{

    if ((uploadbuf_mgmt.used_bytes + 1) > UPLOAD_SIZE)     // not enough room for data
    {
        Pk3Status.UpLoadFull = 1;
        return;
    }

    uc_upload_buffer[uploadbuf_mgmt.write_index++] = byte2write;
    if (uploadbuf_mgmt.write_index >= UPLOAD_SIZE) // handle index wrap
    {
        uploadbuf_mgmt.write_index = 0;
    }
    uploadbuf_mgmt.used_bytes++;  // used another byte.
}

/******************************************************************************
 * Function:        unsigned char ReadDownloadBuffer(void)
 *
 * Overview:        Attempts to pull a byte from the Download Buffer.
 *                  If empty, sets error Pk3Status.DownloadEmpty
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          Returns byte from top of buffer.
 *
 * Side Effects:    Advances download buffer read pointer, if err Pk3Status.StatusHigh != 0
 *
 * Note:            None
 *****************************************************************************/
unsigned char ReadDownloadBuffer(void)
{
    unsigned char readbyte;

    if (downloadbuf_mgmt.used_bytes == 0)
    {
        Pk3Status.DownloadEmpty = 1;
        return 0;
    }

    readbyte = uc_download_buffer[downloadbuf_mgmt.read_index++];
    downloadbuf_mgmt.used_bytes--;        // just removed a byte.
    if (downloadbuf_mgmt.read_index >= DOWNLOAD_SIZE)   // circular buffer - handle wrap.
        downloadbuf_mgmt.read_index = 0;

    return  readbyte;
}

/******************************************************************************
 * Function:        void ReadUploadDataBuffer(void)
 *
 * Overview:        Sends data from upload data buffer over USB.
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          Transmits HID Tx report with data length and data.
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
void ReadUploadDataBuffer(void)
{
    unsigned char i, length;

    length = uploadbuf_mgmt.used_bytes;
    if (length > (BUF_SIZE - 1))        // limited to # bytes in USB report - length byte
    {
        length = (BUF_SIZE - 1);
    }

    outbuffer[0] = length;
    for (i = 1; i<= length; i++)
    {
        outbuffer[i] = uc_upload_buffer[uploadbuf_mgmt.read_index++];
        if (uploadbuf_mgmt.read_index >= UPLOAD_SIZE)  // manage buffer wrap.
        {
            uploadbuf_mgmt.read_index = 0;
        }

    }

    uploadbuf_mgmt.used_bytes -= length;    // read out this many bytes.

    // transmit data
    USBPut(outbuffer, BUF_SIZE);
    //USBHIDTxBlocking();
} // end void ReadUploadDataBuffer(void)

/******************************************************************************
 * Function:        void P32XferData8 (unsigned char byte0)
 *
 * Overview:        Completes an 8-bit XferData psuedo op.
 *
 * PreCondition:    Assumes PGC = output low
 *
 * Input:           byte0 - data to send
 *
 * Output:          received byte in Upload buffer.
 *
 * Side Effects:    None
 *
 * Note:            Also sends TMS header and footer
 *****************************************************************************/
void P32XferData8 (unsigned char byte0)
{
    unsigned char rxbyte = 0;

    //TMS header 100
    P32SetMode(3, 0x01); // sent LSb first

    // TDO first bit received on last phase of setmode
    if (asm_temp3 & 0x80)
        rxbyte = 1;

    // data
    P32DataIO(8, byte0, 0x80); // TMS on last bit
    rxbyte |= (asm_temp3 << 1);
    WriteUploadBuffer(rxbyte);

    // TMS footer 10
    P32SetMode(2, 0x01); // sent LSb first
}

/******************************************************************************
 * Function:        unsigned char P32XferData32(unsigned char byte3, unsigned char byte2, unsigned char byte1, unsigned char byte0, char rxdata)
 *
 * Overview:        Completes a 32-bit XferData psuedo op.  If rxdata > 0, places
 *                  received data in the Upload buffer.
 *
 * PreCondition:    Assumes PGC = output low
 *
 * Input:           byte0 - LSB of data to send
 *                  byte1
 *                  byte2
 *                  byte3
 *                  rxdata - if > 0, save received data
 *
 * Output:          four received bytes in Upload buffer (contigent on rxdata)
 *                  return value is the third of the 4 bytes (for PrACC bit)
 *
 * Side Effects:    None
 *
 * Note:            Also sends TMS header and footer
 *****************************************************************************/
unsigned char P32XferData32(unsigned char byte3, unsigned char byte2, unsigned char byte1, unsigned char byte0, char rxdata )
{
    unsigned char rxbyte = 0;
    unsigned char thirdbyte = 0;

    //TMS header 100
    P32SetMode(3, 0x01); // sent LSb first

    // TDO first bit received on last phase of setmode
    if (asm_temp3 & 0x80)
        rxbyte = 1;

    // data
    P32DataIO(8, byte0, 0);
    rxbyte |= (asm_temp3 << 1);
    if (rxdata)
        WriteUploadBuffer(rxbyte);
    rxbyte = 0;
    if (asm_temp3 & 0x80)
        rxbyte = 1;

    P32DataIO(8, byte1, 0);
    rxbyte |= (asm_temp3 << 1);
    if (rxdata)
        WriteUploadBuffer(rxbyte);
    rxbyte = 0;
    if (asm_temp3 & 0x80)
        rxbyte = 1;

    P32DataIO(8, byte2, 0);
    rxbyte |= (asm_temp3 << 1);
    if (rxdata)
        WriteUploadBuffer(rxbyte);
    thirdbyte = rxbyte;
    rxbyte = 0;
    if (asm_temp3 & 0x80)
        rxbyte = 1;

    P32DataIO(8, byte3, 0x80); // TMS on last bit
    rxbyte |= (asm_temp3 << 1);
    if (rxdata)
        WriteUploadBuffer(rxbyte);

    // TMS footer 10
    P32SetMode(2, 0x01); // sent LSb first

    return thirdbyte;
}

/******************************************************************************
 * Function:        void P32XferFastData32(unsigned char byte3, unsigned char byte2, unsigned char byte1, unsigned char byte0
 *
 * Overview:        Completes a 32-bit XferFastData psuedo op.  If PrAcc = 0
 *                  sets Pk3Status.ICDTimeout
 *
 * PreCondition:    Assumes PGC = output low
 *
 * Input:           byte0 - LSB of data to send
 *                  byte1
 *                  byte2
 *                  byte3
 *
 * Output:          None
 *
 * Side Effects:    May set Pk3Status.ICDTimeout
 *
 * Note:            Also sends TMS header and footer
 *****************************************************************************/
void P32XferFastData32(unsigned char byte3, unsigned char byte2, unsigned char byte1, unsigned char byte0)
{
    //TMS header 100
    P32SetMode(3, 0x01); // sent LSb first

    // PrAcc first bit received on last phase of setmode
    if ((asm_temp3 & 0x80) == 0)
    { // unrecoverable error.
        P32SetMode(5, 0x1F);
        Pk3Status.ICDTimeOut = 1;
        return;
    }

    // PrAcc bit
    P32SetMode(1, 0);

    // data
    P32DataIO(8, byte0, 0);
    P32DataIO(8, byte1, 0);
    P32DataIO(8, byte2, 0);
    P32DataIO(8, byte3, 0x80); // TMS on last bit

    // TMS footer 10
    P32SetMode(2, 0x01); // sent LSb first
}

/******************************************************************************
 * Function:        void P32XferInstruction(void)
 *
 * Overview:        Completes the entire XferInstruction psuedo op.
 *                  Has a timeout on PrACC bit check, which sets Pk3Status.ICDTimeout
 *
 * PreCondition:    Assumes PGC = output low
 *
 * Input:           uses 4 bytes from the Download Buffer
 *
 * Output:          None
 *
 * Side Effects:    May set Pk3Status.ICDTimeout, Pk3Status.DownloadEmpty
 *
 * Note:            Uses Timer0
 *****************************************************************************/
void P32XferInstruction(void)
{
    unsigned char praccbyte;
    WORD timeout = CONNECT_TIMEOUT;

    // This may be extraneous
    //P32SendCommand(0x05);           // MTAP_SW_ETAP
    //P32SetMode(6, 0x1F);

    P32SendCommand(0x0A);           // ETAP_CONTROL
    do
    {
        praccbyte = P32XferData32(0x00, 0x04, 0xD0, 0x00, 0);
    } while (((praccbyte & 0x04) == 0) && --timeout);
    if (timeout==0)
    {
        Pk3Status.ICDTimeOut = 1;   // Timeout error.
        return;
    }
    P32SendCommand(0x09);           // ETAP_DATA
    // actual instruction:
    P32XferData32(ReadDownloadBuffer(), ReadDownloadBuffer(), ReadDownloadBuffer(), ReadDownloadBuffer(), 0);
    P32SendCommand(0x0A);           // ETAP_CONTROL
    P32XferData32(0x00, 0x00, 0xC0, 0x00, 0);
}

/******************************************************************************
 * Function:        void P32GetPEResponse(unsigned char savedata, unsigned char execute)
 *
 * Overview:        Completes the entire GET PE RESPONSE psuedo op.
 *                  Has a timeout on PrACC bit check, which sets Pk3Status.ICDTimeout
 *
 * PreCondition:    Assumes PGC = output low
 *
 * Input:           savedata - if true (> 0), response stored in Upload buffer, else trashed
 *                  noexecute - if true (> 0), the "tell CPU to execute instruction" cmds are skipped
 *
 * Output:          4 bytes in Upload Buffer
 *
 * Side Effects:    May set Pk3Status.ICDTimeout, Pk3Status.UploadFull
 *
 * Note:            Uses Timer0
 *****************************************************************************/
void P32GetPEResponse(unsigned char savedata, unsigned char execute)
{
    unsigned char praccbyte;
    WORD timeout = CONNECT_TIMEOUT;

    P32SendCommand(0x0A);           // ETAP_CONTROL
    do
    {
        praccbyte = P32XferData32(0x00, 0x04, 0xD0, 0x00, 0);
    } while (((praccbyte & 0x04) == 0) && --timeout);
    if (timeout==0)
    {
        Pk3Status.ICDTimeOut = 1;   // Timeout error.
        return;
    }

        P32SendCommand(0x09);           // ETAP_DATA
        // actual instruction:
        P32XferData32(0, 0, 0, 0, savedata);   // response in upload buffer

    if (execute)
    {
        P32SendCommand(0x0A);           // ETAP_CONTROL
        P32XferData32(0x00, 0x00, 0xC0, 0x00, 0);
    }

}

/******************************************************************************
 * Function:        void P32SetMode (unsigned char numbits, unsigned char value)
 *
 * Overview:        Transmits the "numbits" LSbs of "value" as TMS bits on
 *                  JTAG 2W 4PH.  TDI = 0, TDO is ignored.
 *
 * PreCondition:    Assumes PGC = output low
 *
 * Input:           numbits - # of LSbs of "value" to tx as TMS bits
 *                  value - bits to transmit as TMS bits
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Note:            numbits must be > 0
 *****************************************************************************/
void P32SetMode (unsigned char numbits, unsigned char value)
{
    unsigned char i;

    if (numbits == 0)
        return;
    asm_temp1 = 0;      // TDI
    asm_temp2 = value;  // TMS

    for (i=0; i < numbits; i++)
    {
        JTAG2W4PH();
        asm_temp2 >>= 1;
    }
}

/******************************************************************************
 * Function:        unsigned char P32DataIO (unsigned char numbits, unsigned char tdi, unsigned char tms)
 *
 * Overview:        Transmits up to 8 bits at a time on TDI/TMS and receives
 *                  up to 8 bits from TDO as the return value.
 *
 * PreCondition:    Assumes PGC = output low
 *
 * Input:           numbits - # bit sequences
 *                  tdi - value for TDI, LSb first
 *                  tms - value for TMS, LSb first
 *
 * Output:          Returns TDO data (note TDO bit x comes from sequence before
 *                  TDI bit x) shifted in from MSb
 *
 * Side Effects:    None
 *
 * Note:            numbits must be > 0
 *****************************************************************************/
unsigned char P32DataIO (unsigned char numbits, unsigned char tdi, unsigned char tms)
{
    unsigned char i;

    if (numbits == 0)
        return 0;

    asm_temp1 = tdi;
    asm_temp2 = tms;
    asm_temp3 = 0;

    for (i=0; i < numbits; i++)
    {
        asm_temp3 >>= 1;
        JTAG2W4PH();
        asm_temp1 >>= 1;
        asm_temp2 >>= 1;
    }

    return asm_temp3;
}

/******************************************************************************
 * Function:        void P32SendCommand (unsigned char command)
 *
 * Overview:        Transmits a 5 bit command via JTAG 2W 4PH
 *
 * PreCondition:    Assumes PGC = output low
 *
 * Input:           command - 5 bit command value is 5 LSbs
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Note:            Also sends TMS header and footer
 *****************************************************************************/
void P32SendCommand (unsigned char command)
{
    //TMS header 1100
    P32SetMode(4, 0x03); // sent LSb first

    // command itself
    P32DataIO(5, command, 0x10); // TMS on MSB

    // TMS footer 10
    P32SetMode(2, 0x01); // sent LSb first

}

/**
 * PIC32 2-Wire 4-Phase JTAG.
 *
 * Accepts the least-significant bits of <tt>asm_temp1</tt> as TDI and
 * <tt>asm_temp2</tt> as TMS. Outputs TDO as least significant bit of
 * <tt>asm_temp3</tt>.
 *
 * Using C18 3.38, this C code generates identical machine code to the
 * handwritten assembly. That gives a minimum PGCx high or low time of 83 ns
 * and a total execution time (25 instructions, including call and return) of
 * 2.1 µs.
 *
 * \warning ICSPCLK should be an output and driven low before calling.
 */
void JTAG2W4PH(void)
{
    //DISABLE_SYSTEM_INTERRUPT

    pin_MSCK = 1;        // CLK high phase 1
    pin_MSDO = 0;        // TDI = 0?
    if (asm_temp1 & 0x01)   // Test for TDI value
        pin_MSDO = 1;    // TDI = 1
    DATA_DIR(OUT_DIR);;       // Output TDI (PGD = output)
    Nop();
    pin_MSCK = 0;        // CLK low Phase 1
    if (!(asm_temp2 & 0x01))   // Test for TMS value low
        pin_MSDO = 0;    // TMS = 0
    //Insert NOP
    NOP();
    pin_MSCK = 1;        // CLK high phase 2
    if (asm_temp2 & 0x01)   // Test for TMS value high
        pin_MSDO = 1;    // TMS = 1
    asm_temp3 &= ~0x80;     // TDO = 0?
    pin_MSCK = 0;        // CLK low Phase 2
    pin_MSCK = 1;        // CLK high phase 3
    DATA_DIR(IN_DIR);       // Input TDO (PGD = input)
    pin_MSCK = 0;        // CLK low Phase 3
    //Insert NOP
    Nop();
    pin_MSCK = 1;        // CLK high phase 4
    //Insert NOP
    Nop();
    if (pin_MSDI)         // Test for TDO value
        asm_temp3 |= 0x80;  // TDO = 1
    pin_MSCK = 0;        // CLK low Phase 4

    //RESTORE_SYSTEM_INTERRUPT
}

/******************************************************************************
 * Function:        void SendStatusUSB(void)
 *
 * Overview:        Sends READ_STATUS response over USB.
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          Transmits HID Tx report with Pk3Status.
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
void SendStatusUSB(void)
{
    Pk3Status.StatusLow &= 0xF0;    // clear bits to be tested
    if (Vpp_ON_pin)         // active high
        Pk3Status.VppOn = 1;
    if (VPP_Gnd_pin)       // active high
        Pk3Status.VppGNDOn = 1;
    if (!Vdd_TGT_P_pin)     // active low
        Pk3Status.VddOn = 1;
    if (!NVdd_TGT_N_pin)      // active high
        Pk3Status.VddGNDOn = 1;

    if(_ButtonChanged)
        Pk3Status.ButtonPressed = 1;
    _ButtonChanged = 0;     // button status is cleared on every reading of status

    outbuffer[0] = Pk3Status.StatusLow;
    outbuffer[1] = Pk3Status.StatusHigh;

    // Now that it's in the USB buffer, clear errors & flags
    Pk3Status.StatusLow &= 0x8F;
    Pk3Status.StatusHigh &= 0x00;
    SetLED_IDLE();                   // ensure it stops blinking at off.

    // transmit status
    USBPut(outbuffer, BUF_SIZE);
    //USBHIDTxBlocking();
} // end void SendStatusUSB(void)

/******************************************************************************
 * Function:        void ClearScriptTable(void)
 *
 * Overview:        Clears Script buffer by setting all Script Table length entries to zero.
 *
 * PreCondition:    None
 *
 * Input:           None.
 *
 * Output:          ScriptTable[x].Length = 0 for all valid x.
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
void ClearScriptTable(void)
{
    unsigned char i;

    for (i=0; i < SCRIPT_ENTRIES; i++) // init script table to empty.
    {
        ScriptTable[i].Length = 0;
    }
} // end void ClearScriptTable(void)

/******************************************************************************
 * Function:        void StoreScriptInBuffer(WORD *usbindex)
 *
 * Overview:        Stores the script from USB buffer into Script Buffer & updates
 *                  the Script Table.
 *                  Prior script at the given script # is deleted and all following
 *                  scripts are moved up.  New script is appended at end.
 *
 * PreCondition:    None
 *
 * Input:           *usbindex - index to script # byte in USB buffer
 *
 * Output:          uc_script_buffer[] - updated
 *                  ScriptTable[] - updated
 *                  Pk3Status.ScriptBufOvrFlow - set if script length > remaining buffer
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
void StoreScriptInBuffer(WORD *usbindex)
{
    int i;
    int LengthOfAllScripts;
    int Temp_1, Temp_2;

    Temp_1 = inbuffer[(*usbindex)+1];   // Length of new script

    // First, make sure script length is valid
    if (Temp_1 > SCRIPT_MAXLEN)
    {
        Pk3Status.ScriptBufOvrFlow = 1;     // set error - script longer than max allowed
        return;
    }

    Temp_2 = inbuffer[*usbindex];       // Script# of new script

    // calculate length of all scripts.
    LengthOfAllScripts = 0;
    for (i=0; i < SCRIPT_ENTRIES; i++)
    {
        LengthOfAllScripts += ScriptTable[i].Length;
    }
    LengthOfAllScripts -= ScriptTable[Temp_2].Length;   // don't count length of script being replaced
    if (Temp_1 > (SCRIPTBUFSPACE-LengthOfAllScripts)) // if there isn't enough room
    {
        Pk3Status.ScriptBufOvrFlow = 1;     // set error - not enough room in script buffer
        return;
    }

    // Next, make sure script# is valid
    if (Temp_2 > (SCRIPT_ENTRIES-1))    // 0-31 valid
    {
        Pk3Status.ScriptBufOvrFlow = 1;     // set error - script# invalid
        return;
    }


    if (ScriptTable[Temp_2].Length != 0)  // If a script exists in that location
    {
        // Move space created by deleting existing script to end of buffer.
        Temp_1 = (SCRIPTBUFSPACE - ScriptTable[Temp_2].Length) - 1;  // last copy location.
        for (i=ScriptTable[Temp_2].StartIndex; i < Temp_1; i++)
        {
            *(uc_ScriptBuf_ptr + i) = *(uc_ScriptBuf_ptr + ScriptTable[Temp_2].Length + i);
        }
        // update script table entries
        for (i=0; i < SCRIPT_ENTRIES; i++)
        {
            if (ScriptTable[i].StartIndex > ScriptTable[Temp_2].StartIndex) // if script is in moved section
            {
                ScriptTable[i].StartIndex -= ScriptTable[Temp_2].Length;  // adjust by amount moved
            }
        }
    }

    // Store new script at end of buffer
    ScriptTable[Temp_2].Length = inbuffer[(*usbindex)+1];   // update Script Table Entry with new length.
    ScriptTable[Temp_2].StartIndex = LengthOfAllScripts;    // update entry with new index at end of buffer.
    *usbindex += 2; // point to first byte of new script in USB buffer.
    for (i = 0; i < ScriptTable[Temp_2].Length; i++)
    {
        *(uc_ScriptBuf_ptr + LengthOfAllScripts + i) =  inbuffer[(*usbindex)++];
    }

} // end void StoreScriptInBuffer(unsigned char *usbindex)

/******************************************************************************
 * Function:        void RunScript(unsigned char scriptnumber, unsigned char repeat)
 *
 * Overview:        Runs a given script# from the Script Buffer "repeat" number of
 *                  times.
 *
 * PreCondition:    Must be a valid script in the script buffer
 *
 * Input:           scriptnumber = # of script to run
 *                  repeat = how many times to run the script
 *
 * Output:          Pk3Status.EmptyScript set if no script at given script#
 *
 * Side Effects:    Dependent on script being run.
 *
 * Note:            None
 *****************************************************************************/
void RunScript(unsigned char scriptnumber, unsigned char repeat)
{
    // check for valid script #
    if ((scriptnumber >= SCRIPT_ENTRIES) || (ScriptTable[scriptnumber].Length == 0))
    {
        Pk3Status.EmptyScript = 1;  // set error
        return;
    }

    do
    {
        ScriptEngine((uc_ScriptBuf_ptr + ScriptTable[scriptnumber].StartIndex) , ScriptTable[scriptnumber].Length);
        repeat--;
    } while (repeat > 0);

} // end void RunScript(unsigned char scriptnumber, unsigned char repeat)

/******************************************************************************
 * Function:        void SendScriptChecksumsUSB(void)
 *
 * Overview:        Calculates and responds with checksums of the Script Buffer.
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          Transmits HID Tx report with 4 bytes.
 *                      First 2 bytes are a 16-bit sum of the lengths in the
 *                           ScriptTable
 *                      Second 2 are 16-bit sum of all used script bytes.
 *
 * Side Effects:    Stops and restarts Timer1/ADC voltage monitor.
 *
 * Note:            None
 *****************************************************************************/
void SendScriptChecksumsUSB(void)
{
    int length_checksum = 0;
    int buffer_checksum = 0;
    int i = 0;

    for (i = 0; i < SCRIPT_ENTRIES; i++)
    {
        length_checksum += ScriptTable[i].Length;
    }

    for (i = 0; i < length_checksum; i++)
    {
        buffer_checksum += *(uc_ScriptBuf_ptr + i);
    }

    outbuffer[0] = (length_checksum & 0xFF);
    outbuffer[1] = (length_checksum >> 8);
    outbuffer[2] = (buffer_checksum & 0xFF);;
    outbuffer[3] = (buffer_checksum >> 8);;
    // transmit conversion results
    USBPut(outbuffer, BUF_SIZE);
    //USBHIDTxBlocking();
}

/******************************************************************************
 * Function:        void ClearDownloadBuffer(void)
 *
 * Overview:        Clears the Download Buffer
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:
 *
 * Side Effects:
 *
 * Note:            None
 *****************************************************************************/
void ClearDownloadBuffer(void)
{
    downloadbuf_mgmt.write_index = 0;   // init buffer to empty
    downloadbuf_mgmt.read_index = 0;
    downloadbuf_mgmt.used_bytes = 0;
}

/******************************************************************************
 * Function:        void WriteDownloadDataBuffer(WORD *usbindex)
 *
 * Overview:        Writes a given # of bytes into the data download buffer.
 *
 * PreCondition:    None
 *
 * Input:           *usbindex - index to length of data in USB buffer
 *
 * Output:          uc_download_buffer[] - updated with new data
 *                  downloadbuf_mgmt.write_index - incremented by length of data stored.
 *                  downloadbuf_mgmt.used_bytes - incremented by length of data stored.
 *                  Pk3Status.DownloadOvrFlow - set if data length > remaining buffer
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
void WriteDownloadDataBuffer(WORD *usbindex)
{
    unsigned int i, numbytes;

    numbytes = inbuffer[(*usbindex)++] & 0xFF;   // i= # bytes data (length)

    if ((numbytes + downloadbuf_mgmt.used_bytes)  > DOWNLOAD_SIZE)     // not enough room for data
    {
        Pk3Status.DownloadOvrFlow = 1;
        return;
    }

    for (i = 0; i < numbytes; i++)
    {
        uc_download_buffer[downloadbuf_mgmt.write_index++] = inbuffer[(*usbindex)++];
        if (downloadbuf_mgmt.write_index >= DOWNLOAD_SIZE) // handle index wrap
        {
            downloadbuf_mgmt.write_index = 0;
        }
        downloadbuf_mgmt.used_bytes++;  // used another byte.
    }
} // end void WriteDownloadDataBuffer(unsigned char *usbindex)

/******************************************************************************
 * Function:        void ReadUploadDataBufferNoLength(void)
 *
 * Overview:        Sends data from upload data buffer over USB,
 *                  but does not add a length byte.
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          Transmits HID Tx report with data only.
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
void ReadUploadDataBufferNoLength(void)
{
    unsigned char i, length;

    length = uploadbuf_mgmt.used_bytes;
    if (length > (BUF_SIZE))        // limited to # bytes in USB report
    {
        length = (BUF_SIZE);
    }

    for (i = 0; i < length; i++)
    {
        outbuffer[i] = uc_upload_buffer[uploadbuf_mgmt.read_index++];
        if (uploadbuf_mgmt.read_index >= UPLOAD_SIZE)  // manage buffer wrap.
        {
            uploadbuf_mgmt.read_index = 0;
        }

    }

    uploadbuf_mgmt.used_bytes -= length;    // read out this many bytes.

    // transmit data
    USBPut(outbuffer, BUF_SIZE);
    //USBHIDTxBlocking();
} // end void ReadUploadDataBufferNoLength(void)

/******************************************************************************
 * Function:        void WriteByteDownloadBuffer (unsigned char DataByte)
 *
 * Overview:        Puts a byte in the download buffer
 *
 * PreCondition:    None
 *
 * Input:           DataByte - Byte to be put at Write pointer
 *
 * Output:          write pointer and used_bytes updated.
 *
 * Side Effects:
 *
 * Note:            None
 *****************************************************************************/
void WriteByteDownloadBuffer(unsigned char DataByte)
{
        uc_download_buffer[downloadbuf_mgmt.write_index++] = DataByte;
        if (downloadbuf_mgmt.write_index >= DOWNLOAD_SIZE) // handle index wrap
        {
            downloadbuf_mgmt.write_index = 0;
        }
        downloadbuf_mgmt.used_bytes++;  // used another byte.
}

/******************************************************************************
 * Function:        WORD ConvertMaskstoPK3(BYTE mask)
 *                  BYTE ConvertSampletoPK2(BYTE val)
 *                  BYTE SwapByte(BYTE val)
 *
 * Overview:        Utility functions used for converting collected data from
 *                  the format understood by the Pk3 to a format compatible with
 *                  the PICkit2 app.
 *
 * PreCondition:    None
 *
 * Input:
 *
 * Output:
 *
 * Side Effects:
 *
 * Note:            None
 *****************************************************************************/
WORD ConvertMaskstoPK3(BYTE mask)
{
    WORD ret = 0;

    if(mask & 0x4)          // RA2 on PK2 (DAT)
        ret |= 0x8;         // RD3 on PK3

    if(mask & 0x8)          // RA3 on PK2 (CLK)
        ret |= 0x4;         // RD2 on PK3

    if(mask & 0x10)         // RA4 on PK2 (AUX)
        ret |= 0x80;        // RF3 on PK3 (LVP)

    return ret;
}

BYTE ConvertSampletoPK2(BYTE val)
{
    WORD ret = 0;

    if(val & 0x8)           // RD3 on PK3
        ret |= 0x4;         // RA2 on PK2 (DAT)

    if(val & 0x4)           // RD2 on PK3
        ret |= 0x8;         // RA3 on PK2 (CLK)

    if(val & 0x80)          // RF3 on PK3 (LVP)
        ret |= 0x10;        // RA4 on PK2 (AUX)

    return ret;
}

BYTE SwapByte(BYTE val)
{
    BYTE val1, val2;

    val1 = val << 4;
    val2 = val >> 4;

    return (val1 | val2);
}

/******************************************************************************
 * Function:        void LogicAnalyzer(void)
 *
 * Overview:
 *
 * PreCondition:
 *
 * Input:           asm_temp1 = TrigMask
 *                  asm_temp3 = TrigStates
 *                  asm_temp5 = EdgeMask
 *                  asm_temp6 = TrigCount
 *                  asm_temp7 = PostTrigCountL
 *                  asm_temp8 = PostTrigCountH + 1
 *                  asm_temp11 = SampleRateFactorL
 *                  asm_temp13 = SampleRateFactorH
 *                  asm_temp12 = EdgeRising
 *
 * Output:          TrigCount - stores trigger position
 *                  equals FFFF if trigger is aborted
 *                  asm_temp1 = TrigLocL
 *                  asm_temp2 = TrigLocH
 *
 * Side Effects:    Sets PGD, PGC, & AUX pins to inputs, clears script table
 *
 * Note:            Uses ScriptBuffer RAM 0x600 to 0x7FF
 *****************************************************************************/
void LogicAnalyzer(void)
{
    WORD i, tmp, RiseFallMask, SamplingDelay;

    DISABLE_SYSTEM_INTERRUPT

    // Set all lines to input
    CLK_DIR(IN_DIR);
    DATA_DIR(IN_DIR);
    LVP_DIR(IN_DIR);

    ClearScriptTable(); // script buffer is being used by logic analyzer

    SetLED_BUSY();

    // Convert all masks to use the PK3 format. Masks sent by the App are in PK2 mode.
    // need to shift things around since unlike the PK2, the PK3 input lines are in
    // two different port registers.
    cnvTrigMask = ConvertMaskstoPK3(asm_temp1);
    cnvTrigStates = ConvertMaskstoPK3(asm_temp3);
    cnvEdgeMask = ConvertMaskstoPK3(asm_temp5);
    TrigCount = asm_temp6;
    PostTrigCount = (((WORD)asm_temp8)<<8) + asm_temp7;

    _TRISF4 = 0;

    if (asm_temp12)
        RiseFallMask = 0; // don't invert for rising
    else
        RiseFallMask = cnvEdgeMask; // set to mask so it inverts edge bits.

    SamplingDelay = (((WORD)asm_temp13) << 8) + asm_temp11;

    TrigCount = LogicAnalyzerAsm(RiseFallMask, SamplingDelay);

    // Now convert the buffer to the format the App is expecting. This is not
    // the cleanest way. but it saves on instructions, and it requires minimal
    // changes to the app. Processing time is Ok as it is needed only once per
    // trigger.
    for(i=0; i<1024; i+=2)
    {
        BYTE sample1 = _gblIntRamStorage[i];
        BYTE sample2 = _gblIntRamStorage[i+1];
        sample1 = ConvertSampletoPK2(sample1) & 0x1C;
        sample2 = ConvertSampletoPK2(sample2) & 0x1C;
        sample2 = SwapByte(sample2);
        _gblIntRamStorage[i/2] = sample2 | sample1;
    }

    // Now reverse
    for(i=0; i<256; i++)
    {
        BYTE sample1 = _gblIntRamStorage[i];
        BYTE sample2 = _gblIntRamStorage[511-i];
        // swap samples
        _gblIntRamStorage[i] = sample2;
        _gblIntRamStorage[511-i] = sample1;
    }

    // since the PICkit2 is expecting a smaller buffer (half PK3 size)
    if(TrigCount != 0xFFFF)
    {
        TrigCount -= 0x4000;
        tmp = TrigCount & 1;        // odd or even?
        TrigCount /= 2;
        TrigCount = 0x1FF - TrigCount;
        TrigCount += 0x4000;
        if(tmp)
            TrigCount |= 0x8000;
    }

    asm_temp1 = low(TrigCount);
    asm_temp2 = high(TrigCount);

    SetLED_IDLE();

    RESTORE_SYSTEM_INTERRUPT

    // Send response
    outbuffer[0] = asm_temp1;
    outbuffer[1] = asm_temp2;
    // transmit results
    USBPut(outbuffer, BUF_SIZE);
    //USBHIDTxBlocking();
}

/******************************************************************************
 * Function:        void CopyRamUpload(unsigned char addrl, unsigned char addrh)
 *
 * Overview:        Writes 128 bytes from the given RAM address to the UploadBuffer
 *
 * PreCondition:    None
 *
 * Input:           addrh:addrl = RAM address
 *
 * Output:          Data in upload
 *
 * Side Effects:    If SFRs are read, all kinds!
 *
 * Note:
 *****************************************************************************/
void CopyRamUpload(unsigned char addrl, unsigned char addrh)
{
    unsigned char i;
    unsigned char *ram_ptr;

    ram_ptr = (unsigned char *)(addrl + (addrh * 0x100));
    for (i = 0; i < 128; i++)
    {
        WriteUploadBuffer(*(ram_ptr++));
    }

}

/******************************************************************************
 * Function:        void ReadInternalEEPROM(WORD *usbindex)
 *
 * Overview:        Reads a given # of bytes from the SPI Serial EEPROM (U3).
 *
 * PreCondition:    None
 *
 * Input:           *usbindex - index to start address in USB buffer
 *
 * Output:          Transmits HID Tx report with data.
 * 
 * Side Effects:    None
 *
 * Note:            If the length byte is > 32, only the first 32 bytes are 
 *                  Read.
 *****************************************************************************/
void ReadInternalEEPROM(WORD *usbindex)
{
	unsigned int i, numbytes;
	unsigned char ee_address;
	unsigned char seeReadBuffer[32];
	
	ee_address = inbuffer[(*usbindex)++];		// Starting address.
	numbytes = inbuffer[(*usbindex)++] & 0xFF;	// i = # bytes data (length)
	
	if (numbytes  > 32)     // more than allowed # bytes
	{
		numbytes = 32;
	}
	if (numbytes == 0)
	{
		return;
	}
	
	if(ee_address == 0xF0) // unitID address location used by PICkit 2.
	{
		SeeStructRead(mbr.Parameters.ToolName,seeReadBuffer);
	}
	
	for (i = 0; i < numbytes; i++)
	{
		outbuffer[i] = seeReadBuffer[i];
	}
	
	// transmit data
	USBPut(outbuffer, BUF_SIZE);
	//USBHIDTxBlocking();
} 

/******************************************************************************
 * Function:        void WriteInternalEEPROM(WORD *usbindex)
 *
 * Overview:        Writes a given # of bytes to the SPI Serial EEPROM (U3).
 *
 * PreCondition:    None
 *
 * Input:           *usbindex - index to start address of data in USB buffer
 *
 * Output:          Internal EEPROM - updated with new data
 *
 * Side Effects:    None
 *
 * Note:            If the length byte is > 32, only the first 32 bytes are 
 *                  written.
 *****************************************************************************/
void WriteInternalEEPROM(WORD *usbindex)
{
	unsigned char i, numbytes;
	unsigned char ee_address;
	unsigned char seeWriteBuffer[32];
	
	ee_address = inbuffer[(*usbindex)++];		// Starting address.
	numbytes = inbuffer[(*usbindex)++] & 0xFF;	// i = # bytes data (length)
	
	if (numbytes  > 32)     // more than allowed # bytes
	{
		numbytes = 32;
	}
	if (numbytes == 0)
	{
		return;
	}
	
	for (i = 0; i < numbytes; i++)
	{
		seeWriteBuffer[i] = inbuffer[(*usbindex)++];
	}
	
	if(ee_address == 0xF0) // unitID address location used by PICkit 2.
	{
		SeeStructWrite(mbr.Parameters.ToolName,seeWriteBuffer);
	}
}

/******************************************************************************
 * Function:        void SetICSP_PinStates(unsigned char icsp_byte)
 *
 * Overview:        Sets the value and direction of the ICSP pins.
 *
 * PreCondition:    None
 *
 * Input:           icsp_byte - byte formated
 *                      <7 ? 4> unused
 *                      <3> PGD logic level
 *                      <2> PGC logic level
 *                      <1> 1= PGD input, 0= output
 *                      <0> 1= PGC input, 0= output
 *
 *
 * Output:          Affects bits in TRISA and LATA
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
void SetICSP_PinStates(unsigned char icsp_byte)
{
                // set ISCPCLK latch
                if (icsp_byte & 0x04)
                    pin_MSCK = 1;
                else
                    pin_MSCK = 0;
                // set ISCDAT latch
                if (icsp_byte & 0x08)
                    pin_MSDO = 1;
                else
                    pin_MSDO = 0;

                // set ISCPCLK direction
                if (icsp_byte & 0x01)
                {
                    CLK_DIR(IN_DIR);
                }
                else
                {
                    CLK_DIR(OUT_DIR);
                }
                // set ISCDAT direction
                if (icsp_byte & 0x02)
                {
                    DATA_DIR(IN_DIR);
                }
                else
                {
                    DATA_DIR(OUT_DIR);
                }
}

/******************************************************************************
 * Function:        unsigned char GetICSP_PinStates(void)
 *
 * Overview:        Gets the values of the ICSP pins.
 *
 * PreCondition:    None
 *
 * Input:           icsp_byte - byte formated
 *                      <7 ? 4> unused
 *                      <3> PGD logic level
 *                      <2> PGC logic level
 *
 *
 * Output:          returns a byte with bits:
 *                      <1> PGD state
 *                      <0> PGC state
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
BYTE GetICSP_PinStates(void)
{
    unsigned char states = 0;

    //CLK_DIR(IN_DIR);
    //DATA_DIR(IN_DIR);

    if (pin_MSDI == 1)
    {
        states |= 0x02;
    }
    if (pin_MSCK == 1)
    {
        states |= 0x01;
    }

    return states;
}

/******************************************************************************
 * Function:        void SetAUX_PinState(unsigned char aux_byte)
 *
 * Overview:        Sets the value and direction of the AUX pin.
 *
 * PreCondition:    None
 *
 * Input:           aux_byte - byte formated
 *   					<7 ? 2> unused
 *   					<1> AUX logic level
 *    					<0> 1= AUX input, 0= output
 *
 *
 * Output:          Affects bits in TRISA and LATA
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
void SetAUX_PinState(unsigned char aux_byte)
{
    // set AUX latch
    if (aux_byte & 0x02)
        pin_LVP = 1;
    else
        pin_LVP = 0;

    // set AUX direction
    if (aux_byte & 0x01)
    {
        LVP_DIR(IN_DIR);
    }
    else
    {
        LVP_DIR(OUT_DIR);
    }
}

/******************************************************************************
 * Function:        unsigned char GetAUX_PinState(void)
 *
 * Overview:        Gets the values of the ICSP pins.
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          returns a byte with bits:
 *                      <1> AUX state
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
BYTE GetAUX_PinState(void)
{
    unsigned char states = 0;

    //LVP_DIR(IN_DIR);

    if (pin_LVP_in == 1)
    {
        states |= 0x01;
    }

    return states;
}

/******************************************************************************
 * Function:        unsigned char SPI_ReadWrite(unsigned char outputbyte)
 *
 * Overview:        Shifts outputbyte out on the LVP pin with PGC as SCK
 *                  At the same time, bits on PGD are shifted in and
 *                  returned as the read value.
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
unsigned char SPI_ReadWrite(unsigned char outputbyte)
{
	WORD numbits = 0x0008;

	if(icsp_baud<2)
	{
    	asm_temp1 = SPI_ReadWriteFastAsm(outputbyte, numbits);
 	}
    else
    {
    	asm_temp1 = SPI_ReadWriteSlowAsm(outputbyte, numbits, ((icsp_baud-1)*NOPSPERCOUNT_SPI));
 	}

    return asm_temp1;
}

/******************************************************************************
 * Function:        void I2C_Start(void)
 *
 * Overview:        Creates I2C Start condition with PGC = SCL and LVP = SDA.
 *
 * PreCondition:    PGC = output, LVP = input
 *
 * Input:           None
 *
 * Output:          Affects bits in TRISA and LATA
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
void I2C_Start(void)
{
	pin_MSCK = 1;			// SCL high
	
	__delay32(77);			// Delay approximatly 5us (tcy * 80)
	
	pin_LVP = 0;			// ensure LAT bit is zero
	LVP_DIR(OUT_DIR);		// SDA low
	
	__delay32(77);			// Delay approximatly 5us (tcy * 80)
	
	pin_MSCK = 0;			// SCL Low
	
	__delay32(77);			// Delay approximatly 5us (tcy * 80)
	
	LVP_DIR(IN_DIR);		// SDA released
}

/******************************************************************************
 * Function:        void I2C_Stop(void)
 *
 * Overview:        Creates I2C Stop condition with PGC = SCL and LVP = SDA.
 *
 * PreCondition:    PGC = output, LVP = input
 *
 * Input:           None
 *
 * Output:          Affects bits in TRISA and LATA
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
void I2C_Stop(void)
{
	pin_MSCK = 0;			// SCL low
	
    __delay32(77);			// Delay approximatly 5us (tcy * 80)
	
	pin_LVP = 0;			// ensure LAT bit is zero
	LVP_DIR(OUT_DIR);		// SDA low
	
    __delay32(77);			// Delay approximatly 5us (tcy * 80)
	
	pin_MSCK = 1;			// SCL high
	
    __delay32(77);			// Delay approximatly 5us (tcy * 80)
	
	LVP_DIR(IN_DIR);		// SDA released
}

/******************************************************************************
 * Function:        void I2C_Write(unsigned char outputbyte)
 *
 * Overview:        Clocks out a byte with PGC = SCL and LVP = SDA.
 *                  Checks for ACK
 *
 * PreCondition:    PGC = output, LVP = input
 *
 * Input:           outputbyte = byte to be written MSB first
 *
 * Output:          Affects bits in TRISA and LATA
 *                  Pk3Status.ICDTimeOut set if NACK received
 *
 * Side Effects:    None
 *
 * Note:            PK3 has a 4.7k pull-down and a Transil on LVP, so we cannot
 *					support writing I2C devices using pull-up on SDA.
 *					(WE CANNOT SUPPORT I2C DEVICES WITH CURRENT H/W DESIGN)
 *****************************************************************************/
void I2C_Write(unsigned char outputbyte)
{
	WORD numbits = 0x0008;
	
	pin_LVP = 0;		// Ensure LAT bit is initialized to zero.
	
	if(icsp_baud<2)
	{
		I2C_WriteFastAsm(outputbyte, numbits);
    	
		if(asm_temp2 > 0)
		{
	    	Pk3Status.ICDTimeOut = 1; // NACK received.
		}
	}
	else
	{
		I2C_WriteSlowAsm(outputbyte, numbits);
	    if(asm_temp2 > 0)
		{
			Pk3Status.ICDTimeOut = 1; // NACK received.
		}
	}
}

/******************************************************************************
 * Function:        unsigned char I2C_Read(unsigned char giveack)
 *
 * Overview:        Clocks in a byte with PGC = SCL and LVP = SDA.
 *                  Provides and ACK for the byte if giveack is 0.
 *
 * PreCondition:    PGC = output, LVP = input
 *
 * Input:           giveack - ACK the byte if 0, else NACK
 *
 * Output:          Affects bits in TRISA and LATA
 *                  returns byte read MSB first.
 *
 * Side Effects:    None
 *
 * Note:            PK3 has a 4.7k pull-down and a Transil on LVP, so we cannot
 *					support writing I2C devices using pull-up on SDA.
 *					(WE CANNOT SUPPORT I2C DEVICES WITH CURRENT H/W DESIGN)
 *****************************************************************************/
unsigned char I2C_Read(unsigned char giveack)
{
	unsigned char readByte = 0;
	WORD numbits = 0x0008;
	
	DISABLE_SYSTEM_INTERRUPT
	
	if(icsp_baud<2)
	{
	    asm_temp1 = I2C_ReadFastAsm(readByte, numbits, giveack);
	}
    else
    {
	    asm_temp1 = I2C_ReadSlowAsm(readByte, numbits, giveack);
	}
	
	LVP_DIR(IN_DIR);
	
	RESTORE_SYSTEM_INTERRUPT
	
    return asm_temp1;
}

/******************************************************************************
 * Function:        void UNIO (unsigned char device_addr, unsigned char txbytes, unsigned char rxbytes)
 *
 * Overview:        Executes the UNIO Start Header, then transmits "txbytes"
 *                  bytes from the Download Buffer.  If rxbytes is non-zero,
 *                  receives rxbytes into the Upload Buffer.  
 *                  NoMAK's the last byte. Byte is sent/ received MSb first.
 *
 * PreCondition:    None
 *
 * Input:           device_addr - the Device Address to RX after the header
 *                  txbytes - bytes to transmit from DL buffer
 *                  rxbytes - bytes to receive into UL buffer (0 = None)
 *
 * Output:          None
 *
 * Side Effects:    Uses Timer0. LVP is set to Input at exit.  Interrupts
 *                  shut off during execution.  Uses bytes in DL Buffer
 *
 * Note:            None
 *****************************************************************************/
#define LVP_MASK		0x08
#define NOT_LVP_MASK	0xF7
#define TMR_SET_100KHZ_	 77	// = (10.110us Bit Period).
#define TMR_SET_25KHZ_	317	// = (40.075us Bit Period).

#define CONSTANT(s) str(s)
#define str(s) #s

void UNIO (unsigned char device_addr, unsigned char txbytes, unsigned char rxbytes)
{
	BOOL check_SAK = 0;			// Don't check on start header.
	char bitcount;				//
	unsigned char bitdelay;		//
	
	DISABLE_SYSTEM_INTERRUPT
	
	if (icsp_baud > 1)
	{
		PR4 = TMR_SET_25KHZ_;	// Slow : 25 kHz
		bitdelay = 200;			// MAS: __delay32(200) = 12.5 us.
	}
	else
	{
		PR4 = TMR_SET_100KHZ_;	// Fast : 100 kHz
		bitdelay = 40;			// MAS:	__delay32(40) = 2.5 us.
	}

	// Set LVP pin to high output
	pin_LVP = 1;				// RF3 pin set to 1.
	LVP_DIR(OUT_DIR);			// LVP buffer direction A=>B, TRISF3 = 0.

	__delay32(320);				// Delay  2 * TSS = 20us = tcy * 320
	txbytes+= 2;				// add start header & Device Address.

	T4CON = 0x0;				// 8-bit timer, 1:1 prescale.

	// First byte is start header
	asm_temp1 = 0x55;

	pin_LVP = 0;				//
	
	__delay32(160);				// Delay  2 * THDR = 10us = tcy * 160
	
	TMR4 = 0;					// Preset the timer.
	IFS1bits.T4IF = 0;			// Clear Timer 4 interrupt flag.
	T4CONbits.TON = 1;			// Start the timer, first period counts as THDR.
	
	// TRANSMIT ---------------------------------------------------------------
	do
	{
		txbytes--;				// When 0, send NoMAK;
		for (bitcount = 0; bitcount < 8; bitcount++)
		{
			// First part of bit period.
			asm volatile
			(
					"MOV LATF, W0 \n" // lata in w
					"AND.B " CONSTANT(#NOT_LVP_MASK) ", W0 \n"	// Clear output bit ('1' bit).
					"BTSS %0, #7 \n"
					"IOR.B " CONSTANT(#LVP_MASK) ", W0 \n"		// Set pin  ('0' bit).
				"WAITTX1: \n"
					"BTSS IFS1, #11 \n"		// Wait for Timer 4 interrupt flag to be set.
					"BRA WAITTX1 \n"		//
					"MOV W0, LATF \n"		// Update output - start of bit.
					"BSET LATE, #2 \n"		// LVP buffer direction A=>B (SAK sets B=>A).
					"BCLR TRISF, #3 \n"		// LVP pin set to output (SAK sets to input).
					:: "m"(asm_temp1)
			);
			
			IFS1bits.T4IF = 0;			// Clear Timer 4 interrupt flag.
			asm_temp1 <<= 1;			// Rotate for next bit.
			asm_temp2 = LATF;
			if((txbytes == 0) && (rxbytes == 0))
			{
				asm_temp2 |= LVP_MASK;	// NoMAK
			}
			else
			{
				asm_temp2 &= NOT_LVP_MASK;	// MAK;
			}
			// Second part of bit period.
			asm volatile
			(
				"WAITTX2: \n"
					"BTSS IFS1, #11 \n"	// Wait for Timer 4 interrupt flag to be set.
					"BRA WAITTX2 \n"	//
					"BTG LATF, #3 \n"	// Update output - 2nd half is NOT first.
			);
			
			IFS1bits.T4IF = 0;			// Clear Timer 4 interrupt flag.
		}
		
		// MAK
		// First part of bit period.
		asm volatile
		(
				"MOV.B %0, WREG \n"		// Load W0 with asm_temp2.
			"WAITTXMAK1: \n"
				"BTSS IFS1, #11 \n"		// Wait for Timer 4 interrupt flag to be set.
				"BRA WAITTXMAK1 \n"		//
				"MOV W0, LATF \n"		// Update output - start of bit.
				:: "m"(asm_temp2)
		);
		
		IFS1bits.T4IF = 0;				// Clear Timer 4 interrupt flag.
		
		// Get next byte here.
		if ((check_SAK) && (txbytes > 0))
		{
			asm_temp1 = ReadDownloadBuffer();
		}
		else
		{
			asm_temp1 = device_addr;    // If not checking SAK, next byte is Device Address.
		}
		
		// Second part of bit period.
		asm volatile
		(
			"WAITTXMAK2: \n"
				"BTSS IFS1, #11 \n"		// Wait for Timer 4 interrupt flag to be set.
				"BRA WAITTXMAK2 \n"		//
				"BTG LATF, #3 \n"		// Update output - 2nd half is NOT first.
		);
		
		IFS1bits.T4IF = 0;				// Clear Timer 4 interrupt flag.
		
		// SAK
		// Wait for MAK period to end.
		asm volatile
		(
			"WAITTXSAK1: \n"
				"BTSS IFS1, #11 \n"		// Wait for Timer 4 interrupt flag to be set.
				"BRA WAITTXSAK1 \n"		//
				"BCLR LATE, #2 \n"		// LVP buffer direction B=>A.
				"BSET TRISF, #3 \n"		// LVP pin set to input for SAK.
		);
		
		IFS1bits.T4IF = 0;				// Clear Timer 4 interrupt flag.
		
		// We've just started the first half of the SAK period.
		__delay32(bitdelay);			// Delay before getting state.
		if((check_SAK) && (pin_LVP_in == 1))
		{
			// SAK should be zero.
			Pk3Status.ICDTimeOut = 1;	// Bus error.
			txbytes = 0;
			rxbytes = 0;
		}
		
		// Wait for 2nd half of SAK period.
		asm volatile
		(
			"WAITTXSAK2: \n"
				"BTSS IFS1, #11 \n"		// Wait for Timer 4 interrupt flag to be set.
				"BRA WAITTXSAK2 \n"		//
				"NOP \n"				// Normal bit adjustment period.
		);
		
		IFS1bits.T4IF = 0;				// Clear Timer 4 interrupt flag.
		
		__delay32(bitdelay);			// Delay before getting state.
		if((check_SAK) && (pin_LVP_in == 0))
		{
			// SAK should be one.
			Pk3Status.ICDTimeOut = 1;	// Bus error.
			txbytes = 0;
			rxbytes = 0;
		}		
		
		check_SAK = 1;					// Begin checking SAK after header byte.
		
	}while(txbytes > 0);
	
	// RECEIVE ----------------------------------------------------------------
	// If there are any bytes to receive.
	while (rxbytes-- > 0)
	{
		// Byte is received into asm_temp1
		asm_temp1 = 0;
		for (bitcount = 0; bitcount < 8; bitcount++)
		{
			// Wait for previous period to end.
			asm volatile
			(
				"WAITRXBIT: \n"
					"BTSS IFS1, #11 \n"		// Wait for Timer 4 interrupt flag to be set.
					"BRA WAITRXBIT \n"		//
			);
			
			IFS1bits.T4IF = 0;				// Clear Timer 4 interrupt flag.
			
			asm_temp2 = LATF;
			if (rxbytes == 0)
			{
				asm_temp2 |= LVP_MASK;		// NoMAK
			}
			else
			{
				asm_temp2 &= NOT_LVP_MASK;	// MAK
			}
			
			// We've just started the first half of the Bit Period.
			__delay32(bitdelay - 1);		// Delay before getting state.
			
			if(pin_LVP_in == 0)
			{
				// Adjust TMR0 on '1' bit
				// Wait for 2nd half of Bit Period.
				asm volatile
				(
					"WAITRXADJ1: \n"			//
						"BTSC IFS1, #11 \n"		// T4IF as timeout.
						"BRA WAITRXADJ1_GO \n"	//
						"BTSS PORTF, #3 \n"		// Wait for LVP = 1
						"BRA WAITRXADJ1 \n"		//
					"WAITRXADJ1_GO: \n"			//
						"CLR TMR4 \n"			//
				);
			}
			else
			{
				// Adjust TMR0 on '0' bit
				// Wait for 2nd half of Bit period.
				asm volatile
				(
					"WAITRXADJ0: \n"
						"BTSC IFS1, #11 \n"		// T4IF as timeout.
						"BRA WAITRXADJ0_GO \n"	//
						"BTSC PORTF, #3 \n"		// Wait for LVP = 0
						"BRA WAITRXADJ0 \n"		//
					"WAITRXADJ0_GO: \n"			//
						"CLR TMR4"				//
				);
			}
			
			IFS1bits.T4IF = 0;				// Clear Timer 4 interrupt flag.
			
			__delay32(bitdelay);			// Delay before getting data.
			asm_temp1 <<= 1;				// Rotate bits in towards MSB.
			if (pin_LVP_in == 1)
			{
				asm_temp1 += 1;				// Set LSB = 1.
			}
		}
		
		// MAK
		// First part of bit period.
		asm volatile
		(
				"MOV.B %0, WREG \n"		// Load W0 with asm_temp2.
			"WAITRXMAK1: \n"
				"BTSS IFS1, #11 \n"		// Wait for Timer 4 interrupt flag to be set.
				"BRA WAITRXMAK1 \n"		//
				"MOV W0, LATF \n"		// Update output - start of bit.
				"BSET LATE, #2 \n"		// LVP buffer direction A=>B.
				"BCLR TRISF, #3 \n"		// LVP pin set to Output for MAK.
				:: "m"(asm_temp2)
		);
		
		IFS1bits.T4IF = 0;				// Clear Timer 4 interrupt flag.
		Nop();	// MWR
		Nop();	// MWR
		// Save received byte here
		WriteUploadBuffer(asm_temp1);
		
		// Second part of bit period.
		asm volatile
		(
			"WAITRXMAK2: \n"
				"BTSS IFS1, #11 \n"		// Wait for Timer 4 interrupt flag to be set.
				"BRA WAITRXMAK2 \n"		//
				"BTG LATF, #3 \n"		// Update output - 2nd half is NOT first.
		);
		
		IFS1bits.T4IF = 0;				// Clear Timer 4 interrupt flag.
				
		// SAK
		// Wait for MAK period to end.
		asm volatile
		(
			"WAITRXSAK1: \n"
				"BTSS IFS1, #11 \n"		// Wait for Timer 4 interrupt flag to be set.
				"BRA WAITRXSAK1 \n"		//
				"BCLR LATE, #2 \n"		// LVP buffer direction B=>A.
				"BSET TRISF, #3 \n"		// LVP pin set to input for SAK.
		);
			
		IFS1bits.T4IF = 0;				// Clear Timer 4 interrupt flag.
		
		// We've just started the first half of the SAK period.
		__delay32(bitdelay);			// Delay before getting state.
		if((check_SAK) && (pin_LVP_in == 1))
		{
			// SAK should be zero.
			Pk3Status.ICDTimeOut = 1;	// Bus error.
			rxbytes = 0;
		}
		
		// Wait for 2nd half of SAK period.
		asm volatile
		(
			"WAITRXSAK2: \n"
				"BTSS IFS1, #11 \n"		// Wait for Timer 4 interrupt flag to be set.
				"BRA WAITRXSAK2 \n"		//
				"NOP \n"				// Normal bit adjustment period.
		);
		
		IFS1bits.T4IF = 0;				// Clear Timer 4 interrupt flag.
		
		__delay32(bitdelay);			// Delay before getting state.
		if((check_SAK) && (pin_LVP_in == 0))
		{
			// SAK should be one.
			Pk3Status.ICDTimeOut = 1;	// Bus error.
			rxbytes = 0;
		}
	}
	
	LVP_DIR(IN_DIR);					// LVP buffer direction B=>A, TRISF3 = 1.
	
	RESTORE_SYSTEM_INTERRUPT
}

/******************************************************************************
 * Function:        void SendFWVersionUSB(void)
 *
 * Overview:        Sends firmware version over USB. Has to be compatible with
 *                  MPLAB so the App can determine the flavor of the PICkit3
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          Transmits HID Tx report with versioning information and
 *                  a magic number to identify as a scripting capable Pk3.
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
void SendFWVersionUSB(void)
{
    int i;

    // skip version information area expected by MPLAB and make it all zeros to trigger
    // an MPLAB update.
    for(i=0; i<30; i++)
        outbuffer[i] = 0;

    // Magic Key to identify itself as scripting firmware to the programmer app
    outbuffer[30] = 'P';
    outbuffer[31] = 'k';
    outbuffer[32] = '3';

    outbuffer[33] = gblSysVersion.MAJOR;
    outbuffer[34] = gblSysVersion.MINOR;
    outbuffer[35] = gblSysVersion.REV;

    // transmit version number
    //USBHIDTxBlocking();
    USBPut(outbuffer, BUF_SIZE);

} // end void SendFWVersionUSB(void)

/******************************************************************************
 * Function:        void SwitchToBootloader()
 *
 * Overview:        Writes key in flash area to direct the Pk3 to boot in
 *                  bootloader mode. Used for reverting a scripting capabale Pk3
 *                  back to MPLAB mode since MPLAB can only communicate with the
 *                  bootloader but not with the scripting OS.
 *
 * PreCondition:    a valid bootloader is programmed at 0xF000. This bootloader
 *                  is the same one shipped with MPLAB. If this PICkit3 has been
 *                  converted to script mode from within the PC App, this loader
 *                  should be still resident. If this OS is programmed via an
 *                  external programmer, a compatible bootloader hex file should
 *                  be programmed along.
 *
 * Input:           None
 *
 * Output:          Affects bits in TRISA and LATA
 *
 * Side Effects:    Erases the page at PK3_BL_SWITCH_ADDRESS and programs the
 *                  key PK3_JUMP_TO_BOOT there.
 *
 * Note:            None
 *****************************************************************************/
void SwitchToBootloader()
{
    outbuffer[0] = SWITCH_TO_BL;
    outbuffer[1] = 0;

    SetLED_GOOD();

    // First check that bootloader exist
    if(TestBootloader())
    {
        WriteAPBootloaderSwitch();
        outbuffer[2] = 0x99;
    }
    else
        outbuffer[2] = 0xAA;

    USBPut(outbuffer, BUF_SIZE);
    USBflush();

    Delayus(1000000);
    Reset();
}

/******************************************************************************
 * Function:        void TestBootloaderExists()
 *
 * Overview:        Tests if a valid bootloader is programmed at 0xF000 with the
 *                  appropriate magic numbers in the header and footer. This
 *                  helps the App determine if it can revert to MPLAB mode by
 *                  calling SwitchToBootloader() via the command REVERT_TO_MPLAB
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          command is echoed back to the App along with the value 0x99
 *                  if there is a valid bootlaoder or 0xAA if none found.
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
void TestBootloaderExists()
{
    outbuffer[0] = TEST_BOOTLOADER;
    outbuffer[1] = 0;

    // First check that bootloader exist
    if(TestBootloader())
    {
        outbuffer[2] = 0x99;
    }
    else
        outbuffer[2] = 0xAA;

    USBPut(outbuffer, BUF_SIZE);
}
