//*****************************************************************************
//*****************************************************************************
//  FILENAME: HELLO_TIMEOUT.h
//   Version: 2.6, Updated on 2009/7/10 at 10:46:29
//  Generated by PSoC Designer 5.0.985.0
//
//  DESCRIPTION: Timer16 User Module C Language interface file
//               for the 22/24/27/29xxx PSoC family of devices
//-----------------------------------------------------------------------------
//  Copyright (c) Cypress MicroSystems 2000-2004. All Rights Reserved.
//*****************************************************************************
//*****************************************************************************

#include <m8c.h>

#pragma fastcall16 HELLO_TIMEOUT_EnableInt
#pragma fastcall16 HELLO_TIMEOUT_DisableInt
#pragma fastcall16 HELLO_TIMEOUT_Start
#pragma fastcall16 HELLO_TIMEOUT_Stop
#pragma fastcall16 HELLO_TIMEOUT_wReadTimer                // Read  DR0
#pragma fastcall16 HELLO_TIMEOUT_wReadTimerSaveCV          // Read  DR0      
#pragma fastcall16 HELLO_TIMEOUT_WritePeriod               // Write DR1
#pragma fastcall16 HELLO_TIMEOUT_wReadCompareValue         // Read  DR2
#pragma fastcall16 HELLO_TIMEOUT_WriteCompareValue         // Write DR2

// The following symbols are deprecated.
// They may be omitted in future releases
//
#pragma fastcall16 wHELLO_TIMEOUT_ReadCounter              // Read  DR0 "Obsolete"
#pragma fastcall16 wHELLO_TIMEOUT_CaptureCounter           // Read  DR0 "Obsolete"
#pragma fastcall16 wHELLO_TIMEOUT_ReadTimer                // Read  DR0 (Deprecated)
#pragma fastcall16 wHELLO_TIMEOUT_ReadTimerSaveCV          // Read  DR0 (Deprecated)
#pragma fastcall16 wHELLO_TIMEOUT_ReadCompareValue         // Read  DR2 (Deprecated)


//-------------------------------------------------
// Prototypes of the HELLO_TIMEOUT API.
//-------------------------------------------------

extern void HELLO_TIMEOUT_EnableInt(void);                           // Proxy 1
extern void HELLO_TIMEOUT_DisableInt(void);                          // Proxy 1
extern void HELLO_TIMEOUT_Start(void);                               // Proxy 1
extern void HELLO_TIMEOUT_Stop(void);                                // Proxy 1
extern WORD HELLO_TIMEOUT_wReadTimer(void);                          // Proxy 1
extern WORD HELLO_TIMEOUT_wReadTimerSaveCV(void);                    // Proxy 2
extern void HELLO_TIMEOUT_WritePeriod(WORD wPeriod);                 // Proxy 1
extern WORD HELLO_TIMEOUT_wReadCompareValue(void);                   // Proxy 1
extern void HELLO_TIMEOUT_WriteCompareValue(WORD wCompareValue);     // Proxy 1

// The following functions are deprecated.
// They may be omitted in future releases
//
extern WORD wHELLO_TIMEOUT_ReadCompareValue(void);       // Deprecated
extern WORD wHELLO_TIMEOUT_ReadTimerSaveCV(void);        // Deprecated
extern WORD wHELLO_TIMEOUT_ReadCounter(void);            // Obsolete
extern WORD wHELLO_TIMEOUT_ReadTimer(void);              // Deprecated
extern WORD wHELLO_TIMEOUT_CaptureCounter(void);         // Obsolete


//--------------------------------------------------
// Constants for HELLO_TIMEOUT API's.
//--------------------------------------------------

#define HELLO_TIMEOUT_CONTROL_REG_START_BIT    ( 0x01 )
#define HELLO_TIMEOUT_INT_REG_ADDR             ( 0x0e1 )
#define HELLO_TIMEOUT_INT_MASK                 ( 0x02 )


//--------------------------------------------------
// Constants for HELLO_TIMEOUT user defined values
//--------------------------------------------------

#define HELLO_TIMEOUT_PERIOD                   ( 0x1e0 )
#define HELLO_TIMEOUT_COMPARE_VALUE            ( 0x0 )


//-------------------------------------------------
// Register Addresses for HELLO_TIMEOUT
//-------------------------------------------------

#pragma ioport  HELLO_TIMEOUT_COUNTER_LSB_REG:  0x020      //Count register LSB
BYTE            HELLO_TIMEOUT_COUNTER_LSB_REG;
#pragma ioport  HELLO_TIMEOUT_COUNTER_MSB_REG:  0x024      //Count register MSB
BYTE            HELLO_TIMEOUT_COUNTER_MSB_REG;
#pragma ioport  HELLO_TIMEOUT_PERIOD_LSB_REG:   0x021      //Period register LSB
BYTE            HELLO_TIMEOUT_PERIOD_LSB_REG;
#pragma ioport  HELLO_TIMEOUT_PERIOD_MSB_REG:   0x025      //Period register MSB
BYTE            HELLO_TIMEOUT_PERIOD_MSB_REG;
#pragma ioport  HELLO_TIMEOUT_COMPARE_LSB_REG:  0x022      //Compare register LSB
BYTE            HELLO_TIMEOUT_COMPARE_LSB_REG;
#pragma ioport  HELLO_TIMEOUT_COMPARE_MSB_REG:  0x026      //Compare register MSB
BYTE            HELLO_TIMEOUT_COMPARE_MSB_REG;
#pragma ioport  HELLO_TIMEOUT_CONTROL_LSB_REG:  0x023      //Control register LSB
BYTE            HELLO_TIMEOUT_CONTROL_LSB_REG;
#pragma ioport  HELLO_TIMEOUT_CONTROL_MSB_REG:  0x027      //Control register MSB
BYTE            HELLO_TIMEOUT_CONTROL_MSB_REG;
#pragma ioport  HELLO_TIMEOUT_FUNC_LSB_REG: 0x120          //Function register LSB
BYTE            HELLO_TIMEOUT_FUNC_LSB_REG;
#pragma ioport  HELLO_TIMEOUT_FUNC_MSB_REG: 0x124          //Function register MSB
BYTE            HELLO_TIMEOUT_FUNC_MSB_REG;
#pragma ioport  HELLO_TIMEOUT_INPUT_LSB_REG:    0x121      //Input register LSB
BYTE            HELLO_TIMEOUT_INPUT_LSB_REG;
#pragma ioport  HELLO_TIMEOUT_INPUT_MSB_REG:    0x125      //Input register MSB
BYTE            HELLO_TIMEOUT_INPUT_MSB_REG;
#pragma ioport  HELLO_TIMEOUT_OUTPUT_LSB_REG:   0x122      //Output register LSB
BYTE            HELLO_TIMEOUT_OUTPUT_LSB_REG;
#pragma ioport  HELLO_TIMEOUT_OUTPUT_MSB_REG:   0x126      //Output register MSB
BYTE            HELLO_TIMEOUT_OUTPUT_MSB_REG;
#pragma ioport  HELLO_TIMEOUT_INT_REG:       0x0e1         //Interrupt Mask Register
BYTE            HELLO_TIMEOUT_INT_REG;


//-------------------------------------------------
// HELLO_TIMEOUT Macro 'Functions'
//-------------------------------------------------

#define HELLO_TIMEOUT_Start_M \
   ( HELLO_TIMEOUT_CONTROL_LSB_REG |=  HELLO_TIMEOUT_CONTROL_REG_START_BIT )

#define HELLO_TIMEOUT_Stop_M  \
   ( HELLO_TIMEOUT_CONTROL_LSB_REG &= ~HELLO_TIMEOUT_CONTROL_REG_START_BIT )

#define HELLO_TIMEOUT_EnableInt_M   \
   M8C_EnableIntMask(  HELLO_TIMEOUT_INT_REG, HELLO_TIMEOUT_INT_MASK )

#define HELLO_TIMEOUT_DisableInt_M  \
   M8C_DisableIntMask( HELLO_TIMEOUT_INT_REG, HELLO_TIMEOUT_INT_MASK )


// end of file HELLO_TIMEOUT.h


