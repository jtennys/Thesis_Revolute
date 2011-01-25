//*****************************************************************************
//*****************************************************************************
//  FILENAME: TX8_3.h
//   Version: 3.3, Updated on 2009/7/10 at 10:46:51
//  Generated by PSoC Designer 5.0.985.0
//
//  DESCRIPTION: TX8 User Module C Language interface file for the
//               22/24/25/26/27xxx PSoC family of devices.
//-----------------------------------------------------------------------------
//      Copyright (c) Cypress MicroSystems 2000-2003. All Rights Reserved.
//*****************************************************************************
//*****************************************************************************

#include <m8c.h>

/* Create pragmas to support proper argument and return value passing */
#pragma fastcall16  TX8_3_SetTxIntMode
#pragma fastcall16  TX8_3_EnableInt
#pragma fastcall16  TX8_3_DisableInt
#pragma fastcall16  TX8_3_Start
#pragma fastcall16  TX8_3_Stop
#pragma fastcall16  TX8_3_SendData
#pragma fastcall16  TX8_3_bReadTxStatus

// High level TX functions
#pragma fastcall16  TX8_3_PutSHexByte
#pragma fastcall16  TX8_3_PutSHexInt
#pragma fastcall16  TX8_3_CPutString
#pragma fastcall16  TX8_3_PutString
#pragma fastcall16  TX8_3_PutChar
#pragma fastcall16  TX8_3_Write
#pragma fastcall16  TX8_3_CWrite
#pragma fastcall16  TX8_3_PutCRLF

//-------------------------------------------------
// Prototypes of the TX8_3 API.
//-------------------------------------------------
extern void  TX8_3_SetTxIntMode(BYTE bTxIntMode);
extern void  TX8_3_EnableInt(void);
extern void  TX8_3_DisableInt(void);
extern void  TX8_3_Start(BYTE bParity);
extern void  TX8_3_Stop(void);
extern void  TX8_3_SendData(BYTE bTxData);
extern BYTE  TX8_3_bReadTxStatus(void);

// High level TX functions
extern void   TX8_3_CPutString(const char * szRomString);
extern void   TX8_3_PutString(char * szRamString);
extern void   TX8_3_PutChar(CHAR cData);
extern void   TX8_3_Write(char * szRamString, BYTE bCount);
extern void   TX8_3_CWrite(const char * szRomString, INT iCount);
extern void   TX8_3_PutSHexByte(BYTE bValue);
extern void   TX8_3_PutSHexInt(INT iValue);
extern void   TX8_3_PutCRLF(void);

// Old style function name, Do Not Use.
// Will be removfr in a future release
#pragma fastcall16 bTX8_3_ReadTxStatus
extern BYTE bTX8_3_ReadTxStatus(void);

//------------------------------------
//  Transmitter Parity masks
//------------------------------------
#define  TX8_3_PARITY_NONE         0x00
#define  TX8_3_PARITY_EVEN         0x02
#define  TX8_3_PARITY_ODD          0x06

//------------------------------------
//  Transmitter Status Register masks
//------------------------------------
#define  TX8_3_TX_COMPLETE         0x20
#define  TX8_3_TX_BUFFER_EMPTY     0x10

#define TX8_3_INT_MODE_TX_REG_EMPTY 0x00
#define TX8_3_INT_MODE_TX_COMPLETE  0x01

//------------------------------------
// Transmitter Interrupt masks
//------------------------------------
#define TX8_3_INT_REG_ADDR                     ( 0x0e1 )
#define TX8_3_bINT_MASK                        ( 0x04 )

// Old style defines, do not use.  These
// will be removed in a future release.
#define  TX8_PARITY_NONE         0x00
#define  TX8_PARITY_EVEN         0x02
#define  TX8_PARITY_ODD          0x06
#define  TX8_TX_COMPLETE         0x20
#define  TX8_TX_BUFFER_EMPTY     0x10



//-------------------------------------------------
// Register Addresses for TX8_3
//-------------------------------------------------
#pragma ioport  TX8_3_CONTROL_REG:  0x02b                  // Control register
BYTE            TX8_3_CONTROL_REG;
#pragma ioport  TX8_3_TX_SHIFT_REG: 0x028                  // TX Shift Register register
BYTE            TX8_3_TX_SHIFT_REG;
#pragma ioport  TX8_3_TX_BUFFER_REG:    0x029              // TX Buffer Register
BYTE            TX8_3_TX_BUFFER_REG;
#pragma ioport  TX8_3_FUNC_REG: 0x128                      // Function register
BYTE            TX8_3_FUNC_REG;
#pragma ioport  TX8_3_INPUT_REG:    0x129                  // Input register
BYTE            TX8_3_INPUT_REG;
#pragma ioport  TX8_3_OUTPUT_REG:   0x12a                  // Output register
BYTE            TX8_3_OUTPUT_REG;

// end of file TX8_3.h
