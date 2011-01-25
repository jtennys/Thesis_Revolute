//*****************************************************************************
//*****************************************************************************
//  FILENAME: RX8_7.h
//   Version: 3.3, Updated on 2009/7/10 at 10:46:15
//  Generated by PSoC Designer 5.0.985.0
//
//  DESCRIPTION:  RX8 User Module C Language interface file for the
//                22/24/25/26/27xxx PSoC family of devices.
//-----------------------------------------------------------------------------
//      Copyright (c) Cypress MicroSystems 2000-2003. All Rights Reserved.
//*****************************************************************************
//*****************************************************************************


// include the global header file
#include <m8c.h>

#define RX8_7_RXBUF_ENABLE 1

//-------------------------------------------------
// Prototypes of the RX8_7 API.
//-------------------------------------------------

#if ( RX8_7_RXBUF_ENABLE )
extern char RX8_7_aRxBuffer[];
extern BYTE RX8_7_bRxCnt;
extern BYTE RX8_7_fStatus;
#endif

// Create pragmas to support proper argument and return value passing
#pragma fastcall16  RX8_7_EnableInt
#pragma fastcall16  RX8_7_DisableInt
#pragma fastcall16  RX8_7_Start
#pragma fastcall16  RX8_7_Stop
#pragma fastcall16  RX8_7_bReadRxData
#pragma fastcall16  RX8_7_bReadRxStatus

#pragma fastcall16  RX8_7_cGetChar
#pragma fastcall16  RX8_7_cReadChar
#pragma fastcall16  RX8_7_iReadChar

#if ( RX8_7_RXBUF_ENABLE )
#pragma fastcall16  RX8_7_CmdReset
#pragma fastcall16  RX8_7_bCmdCheck
#pragma fastcall16  RX8_7_bErrCheck
#pragma fastcall16  RX8_7_bCmdLength
#pragma fastcall16  RX8_7_szGetParam
#pragma fastcall16  RX8_7_szGetRestOfParams
#endif

//-------------------------------------------------
// Prototypes of the RX8_7 API.
//-------------------------------------------------
extern void  RX8_7_EnableInt(void);
extern void  RX8_7_DisableInt(void);
extern void  RX8_7_Start(BYTE bParity);
extern void  RX8_7_Stop(void);
extern BYTE  RX8_7_bReadRxData(void);
extern BYTE  RX8_7_bReadRxStatus(void);

// High level RX functions
extern CHAR         RX8_7_cGetChar(void);
extern CHAR         RX8_7_cReadChar(void);
extern INT          RX8_7_iReadChar(void);

#if ( RX8_7_RXBUF_ENABLE )
extern void   RX8_7_CmdReset(void);
extern BYTE   RX8_7_bCmdCheck(void);
extern BYTE   RX8_7_bErrCheck(void);
extern BYTE   RX8_7_bCmdLength(void);
extern char * RX8_7_szGetParam(void);
extern char * RX8_7_szGetRestOfParams(void);
#endif

// Old function call names, do not use.
// These names will be removed in a future release.
#pragma fastcall16 bRX8_7_ReadRxData
#pragma fastcall16 bRX8_7_ReadRxStatus
extern BYTE bRX8_7_ReadRxData(void);
extern BYTE bRX8_7_ReadRxStatus(void);

//-------------------------------------------------
// Constants for RX8_7 API's.
//-------------------------------------------------

//------------------------------------
// Receiver Interrupt masks
//------------------------------------
#define RX8_7_INT_REG_ADDR                     ( 0x0e1 )
#define RX8_7_bINT_MASK                        ( 0x04 )

//------------------------------------
// Receiver Parity masks
//------------------------------------
#define  RX8_7_PARITY_NONE         0x00
#define  RX8_7_PARITY_EVEN         0x02
#define  RX8_7_PARITY_ODD          0x06

//------------------------------------
//  Receiver Status Register masks
//------------------------------------
#define  RX8_7_RX_ACTIVE           0x10
#define  RX8_7_RX_COMPLETE         0x08
#define  RX8_7_RX_PARITY_ERROR     0x80
#define  RX8_7_RX_OVERRUN_ERROR    0x40
#define  RX8_7_RX_FRAMING_ERROR    0x20
#define  RX8_7_RX_NO_ERROR         0xE0

#define  RX8_7_RX_NO_DATA         0x01

#define  RX8_7_RX_BUF_ERROR               0xF0  // Mask for any Rx that may occur.
#define  RX8_7_RX_BUF_OVERRUN             0x10  // This indicates the software buffer has
                                                           // been over run.
#define  RX8_7_RX_BUF_CMDTERM             0x01  // Command terminator has been received.

// Old defines, will be removed in future release
#define  RX8_PARITY_NONE         0x00
#define  RX8_PARITY_EVEN         0x02
#define  RX8_PARITY_ODD          0x06
#define  RX8_RX_ACTIVE           0x10
#define  RX8_RX_COMPLETE         0x08
#define  RX8_RX_PARITY_ERROR     0x80
#define  RX8_RX_OVERRUN_ERROR    0x40
#define  RX8_RX_FRAMING_ERROR    0x20
#define  RX8_RX_NO_ERROR         0xE0

//-------------------------------------------------
// Register Addresses for RX8_7
//-------------------------------------------------
#pragma ioport  RX8_7_CONTROL_REG:  0x02b                  // Control register
BYTE            RX8_7_CONTROL_REG;
#pragma ioport  RX8_7_RX_SHIFT_REG: 0x028                  // RX Shift Register register
BYTE            RX8_7_RX_SHIFT_REG;
#pragma ioport  RX8_7_RX_BUFFER_REG:    0x02a              // RX Buffer Register
BYTE            RX8_7_RX_BUFFER_REG;
#pragma ioport  RX8_7_FUNC_REG: 0x128                      // Function register
BYTE            RX8_7_FUNC_REG;
#pragma ioport  RX8_7_INPUT_REG:    0x129                  // Input register
BYTE            RX8_7_INPUT_REG;
#pragma ioport  RX8_7_OUTPUT_REG:   0x12a                  // Output register
BYTE            RX8_7_OUTPUT_REG;

// end of file RX8_7.h

