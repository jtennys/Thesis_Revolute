//*****************************************************************************
//*****************************************************************************
//  FILENAME: Timer16_1.h
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

#pragma fastcall16 Timer16_1_EnableInt
#pragma fastcall16 Timer16_1_DisableInt
#pragma fastcall16 Timer16_1_Start
#pragma fastcall16 Timer16_1_Stop
#pragma fastcall16 Timer16_1_wReadTimer                // Read  DR0
#pragma fastcall16 Timer16_1_wReadTimerSaveCV          // Read  DR0      
#pragma fastcall16 Timer16_1_WritePeriod               // Write DR1
#pragma fastcall16 Timer16_1_wReadCompareValue         // Read  DR2
#pragma fastcall16 Timer16_1_WriteCompareValue         // Write DR2

// The following symbols are deprecated.
// They may be omitted in future releases
//
#pragma fastcall16 wTimer16_1_ReadCounter              // Read  DR0 "Obsolete"
#pragma fastcall16 wTimer16_1_CaptureCounter           // Read  DR0 "Obsolete"
#pragma fastcall16 wTimer16_1_ReadTimer                // Read  DR0 (Deprecated)
#pragma fastcall16 wTimer16_1_ReadTimerSaveCV          // Read  DR0 (Deprecated)
#pragma fastcall16 wTimer16_1_ReadCompareValue         // Read  DR2 (Deprecated)


//-------------------------------------------------
// Prototypes of the Timer16_1 API.
//-------------------------------------------------

extern void Timer16_1_EnableInt(void);                           // Proxy 1
extern void Timer16_1_DisableInt(void);                          // Proxy 1
extern void Timer16_1_Start(void);                               // Proxy 1
extern void Timer16_1_Stop(void);                                // Proxy 1
extern WORD Timer16_1_wReadTimer(void);                          // Proxy 1
extern WORD Timer16_1_wReadTimerSaveCV(void);                    // Proxy 2
extern void Timer16_1_WritePeriod(WORD wPeriod);                 // Proxy 1
extern WORD Timer16_1_wReadCompareValue(void);                   // Proxy 1
extern void Timer16_1_WriteCompareValue(WORD wCompareValue);     // Proxy 1

// The following functions are deprecated.
// They may be omitted in future releases
//
extern WORD wTimer16_1_ReadCompareValue(void);       // Deprecated
extern WORD wTimer16_1_ReadTimerSaveCV(void);        // Deprecated
extern WORD wTimer16_1_ReadCounter(void);            // Obsolete
extern WORD wTimer16_1_ReadTimer(void);              // Deprecated
extern WORD wTimer16_1_CaptureCounter(void);         // Obsolete


//--------------------------------------------------
// Constants for Timer16_1 API's.
//--------------------------------------------------

#define Timer16_1_CONTROL_REG_START_BIT        ( 0x01 )
#define Timer16_1_INT_REG_ADDR                 ( 0x0e1 )
#define Timer16_1_INT_MASK                     ( 0x02 )


//--------------------------------------------------
// Constants for Timer16_1 user defined values
//--------------------------------------------------

#define Timer16_1_PERIOD                       ( 0x0 )
#define Timer16_1_COMPARE_VALUE                ( 0x0 )


//-------------------------------------------------
// Register Addresses for Timer16_1
//-------------------------------------------------

#pragma ioport  Timer16_1_COUNTER_LSB_REG:  0x020          //Count register LSB
BYTE            Timer16_1_COUNTER_LSB_REG;
#pragma ioport  Timer16_1_COUNTER_MSB_REG:  0x024          //Count register MSB
BYTE            Timer16_1_COUNTER_MSB_REG;
#pragma ioport  Timer16_1_PERIOD_LSB_REG:   0x021          //Period register LSB
BYTE            Timer16_1_PERIOD_LSB_REG;
#pragma ioport  Timer16_1_PERIOD_MSB_REG:   0x025          //Period register MSB
BYTE            Timer16_1_PERIOD_MSB_REG;
#pragma ioport  Timer16_1_COMPARE_LSB_REG:  0x022          //Compare register LSB
BYTE            Timer16_1_COMPARE_LSB_REG;
#pragma ioport  Timer16_1_COMPARE_MSB_REG:  0x026          //Compare register MSB
BYTE            Timer16_1_COMPARE_MSB_REG;
#pragma ioport  Timer16_1_CONTROL_LSB_REG:  0x023          //Control register LSB
BYTE            Timer16_1_CONTROL_LSB_REG;
#pragma ioport  Timer16_1_CONTROL_MSB_REG:  0x027          //Control register MSB
BYTE            Timer16_1_CONTROL_MSB_REG;
#pragma ioport  Timer16_1_FUNC_LSB_REG: 0x120              //Function register LSB
BYTE            Timer16_1_FUNC_LSB_REG;
#pragma ioport  Timer16_1_FUNC_MSB_REG: 0x124              //Function register MSB
BYTE            Timer16_1_FUNC_MSB_REG;
#pragma ioport  Timer16_1_INPUT_LSB_REG:    0x121          //Input register LSB
BYTE            Timer16_1_INPUT_LSB_REG;
#pragma ioport  Timer16_1_INPUT_MSB_REG:    0x125          //Input register MSB
BYTE            Timer16_1_INPUT_MSB_REG;
#pragma ioport  Timer16_1_OUTPUT_LSB_REG:   0x122          //Output register LSB
BYTE            Timer16_1_OUTPUT_LSB_REG;
#pragma ioport  Timer16_1_OUTPUT_MSB_REG:   0x126          //Output register MSB
BYTE            Timer16_1_OUTPUT_MSB_REG;
#pragma ioport  Timer16_1_INT_REG:       0x0e1             //Interrupt Mask Register
BYTE            Timer16_1_INT_REG;


//-------------------------------------------------
// Timer16_1 Macro 'Functions'
//-------------------------------------------------

#define Timer16_1_Start_M \
   ( Timer16_1_CONTROL_LSB_REG |=  Timer16_1_CONTROL_REG_START_BIT )

#define Timer16_1_Stop_M  \
   ( Timer16_1_CONTROL_LSB_REG &= ~Timer16_1_CONTROL_REG_START_BIT )

#define Timer16_1_EnableInt_M   \
   M8C_EnableIntMask(  Timer16_1_INT_REG, Timer16_1_INT_MASK )

#define Timer16_1_DisableInt_M  \
   M8C_DisableIntMask( Timer16_1_INT_REG, Timer16_1_INT_MASK )


// end of file Timer16_1.h


