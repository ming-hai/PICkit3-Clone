/*********************************************************************
 *
 * standard Type definition file
 *
 *********************************************************************
 * FileName:            types.h
 * Dependencies:        none
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

#ifndef TYPES_DOT_H
#define TYPES_DOT_H

//-------------------------------------------------------------------- 
// COMMON - Types
//-------------------------------------------------------------------- 


/*
 * u8
 * Unsigned 8-bit memory type (0 to 255)
 */
#define u8  unsigned char

/*
 * s8
 * Signed 8-bit memory type (-128 to 127)
 */
#define s8  char

/*
 * u16
 * Unsigned 16-bit memory type (0 to 65,535)
 */
#define u16 unsigned int

/*
 * s16
 * Signed 16bit memory type(-32,768 to 32,767)
 */
#define s16 int

/*
 * u32
 * Unsigned 32-bit memory type (0 to 4,294,967,295)
 */
#define u32 unsigned long

/*
 * s32
 * Signed 32-bit memory type (-2,147,483,648 to 2,147,483,648)
 */
#define s32 long

#define bool unsigned char

/*
 * word
 * Unsigned 16-bit memory type (0 to 65,535)
 */
#define word u16

/*
 * WORD
 * Unsigned 16-bit memory type (0 to 65,535)
 */
#define WORD u16

/*
 * DWORD
 * Unsigned 32-bit memory type (0 to 4,294,967,295)
 */
#define DWORD u32

/*
 * dword
 * Unsigned 32-bit memory type (0 to 4,294,967,295)
 */
#define dword u32

/*
 * byte
 * Unsigned 8-bit memory type (0 to 255)
 */
#define byte u8

/*
 * BYTE
 * Unsigned 8-bit memory type (0 to 255)
 */
#define BYTE u8

#define BOOL bool

#define U08 u8  

#define S08 s8  

#define U16 u16 

#define S16 s16 

#define U32 u32 

#define S32 s32 

#define VOID_FCN_VOID_CAST          void(*)(void)

#define VOID_FCN_PVOID_CAST         void(*)(void*)

#define U16_FCN_BOOL_CAST           u16(*)(bool)

#define BOOL_FCN_VOID_CAST          bool(*)(void)

#define U16_FCN_VOID_CAST           u16(*)(void)

#define text rom u8                     // Compile time text

#define FALSE   0

#define TRUE    !FALSE


/*
 * OK
 * Return value
 */
#define OK      0

/*
 * PASS
 * Pass value
 */
#define PASS    0

/*
 * FAIL
 * Fail value
 */
#define FAIL    -1

#define ON  1

#define OFF 0

#define INPUT   1

#define OUTPUT  0

// two_byte
//
typedef union{
struct
    {
    unsigned char l_byte;   // low byte
    unsigned char h_byte;   // high byte
    };
    unsigned int wrd;       // Word
}two_byte;

// BITFIELD
//
typedef struct
{
    unsigned b0:1;
    unsigned b1:1;
    unsigned b2:1;      
    unsigned b3:1;
    unsigned b4:1;
    unsigned b5:1;              
    unsigned b6:1;  
    unsigned b7:1;  
}BITFIELD,*PBITFIELD;


// four_byte
//
typedef union{
    struct
    {
        u8 l_byte;
        u8 h_byte;
        u8 u_byte;
        u8 p_byte;
    };
    struct
    {
        u16 l_word;
        u16 h_word;
    };
    u32 _long;
}four_byte;


// typRunTimeMsgBox
//
typedef union
{
    struct
    {
        unsigned    DataSendReq:1;      // Data Send Request
        unsigned    DataSendAck:1;      // Data Send Acknowledge
        unsigned    SendSSS_status:1;
    };
    WORD    wrd;
}typRunTimeMsgBox, * pRunTimeMsgBox;

#define block       u8*

// takes more cycles so called Sfcn. Need these to expand in macros
#define _SLow(num) (num & 0xFF)
#define SLow(num) (_SLow(num))
#define SLoword(num) (num & 0xFFFF)
#define SHigh(num) ((num >> 8) & 0xFF)
#define SUpper(num) ((num >> 16) & 0xFF)


#define low(num)    (((BYTE *)&num)[0])
#define loword(num) (num & 0xFFFF)
#define loword2(num)(((WORD *)&num)[0])
#define high(num)   (((BYTE *)&num)[1])
#define hiword(num) (((WORD *)&num)[1])
#define upper(num)  (((BYTE *)&num)[2])
#define mupper(num) (((BYTE *)&num)[3])

// two bytes instead of one
typedef unsigned int   UNICODE;

typedef enum _WORDORDER{BIGENDIAN,LITENDIAN}WORDORDER;

typedef enum _PWRMODE{PWR_MODE_STATE, NO_PWR_MODE, PROGRAMMING_PWR_MODE, DEBUGGING_PWR_MODE}PWRMODE;

#define JOIN2(X,Y) X ## Y
#define JOIN(X,Y) JOIN2(X,Y)
#define mUL(X) JOIN(X,UL)

#define FOREVER     while(1)

// The following definitions are used by the MCHP USB framework
#define UINT16 u16
typedef union _BYTE_VAL
{
    BYTE Val;
    struct
    {
        unsigned char b0:1;
        unsigned char b1:1;
        unsigned char b2:1;
        unsigned char b3:1;
        unsigned char b4:1;
        unsigned char b5:1;
        unsigned char b6:1;
        unsigned char b7:1;
    } bits;
} BYTE_VAL, BYTE_BITS;

typedef union _WORD_VAL
{
    WORD Val;
    BYTE v[2];
    struct
    {
        BYTE LB;
        BYTE HB;
    } val_byte;
    struct
    {
        unsigned char b0:1;
        unsigned char b1:1;
        unsigned char b2:1;
        unsigned char b3:1;
        unsigned char b4:1;
        unsigned char b5:1;
        unsigned char b6:1;
        unsigned char b7:1;
        unsigned char b8:1;
        unsigned char b9:1;
        unsigned char b10:1;
        unsigned char b11:1;
        unsigned char b12:1;
        unsigned char b13:1;
        unsigned char b14:1;
        unsigned char b15:1;
    } bits;
} WORD_VAL, WORD_BITS;

typedef union _DWORD_VAL
{
    DWORD Val;
    WORD w[2];
    BYTE v[4];
    struct
    {
        WORD LW;
        WORD HW;
    } val_word;
    struct
    {
        BYTE LB;
        BYTE HB;
        BYTE UB;
        BYTE MB;
    } val_byte;
    struct
    {
        WORD_VAL low;
        WORD_VAL high;
    }wordUnion;
    struct
    {
        unsigned char b0:1;
        unsigned char b1:1;
        unsigned char b2:1;
        unsigned char b3:1;
        unsigned char b4:1;
        unsigned char b5:1;
        unsigned char b6:1;
        unsigned char b7:1;
        unsigned char b8:1;
        unsigned char b9:1;
        unsigned char b10:1;
        unsigned char b11:1;
        unsigned char b12:1;
        unsigned char b13:1;
        unsigned char b14:1;
        unsigned char b15:1;
        unsigned char b16:1;
        unsigned char b17:1;
        unsigned char b18:1;
        unsigned char b19:1;
        unsigned char b20:1;
        unsigned char b21:1;
        unsigned char b22:1;
        unsigned char b23:1;
        unsigned char b24:1;
        unsigned char b25:1;
        unsigned char b26:1;
        unsigned char b27:1;
        unsigned char b28:1;
        unsigned char b29:1;
        unsigned char b30:1;
        unsigned char b31:1;
    } bits;
} DWORD_VAL;

#endif

