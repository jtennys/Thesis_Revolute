;;*****************************************************************************
;;*****************************************************************************
;;  FILENAME: HELLO_1INT.asm
;;   Version: 3.3, Updated on 2009/7/10 at 10:46:15
;;  Generated by PSoC Designer 5.0.985.0
;;
;;  DESCRIPTION: RX8 Interrupt Service Routine.
;;-----------------------------------------------------------------------------
;;  Copyright (c) Cypress MicroSystems 2000-2003. All Rights Reserved.
;;*****************************************************************************
;;*****************************************************************************

include "m8c.inc"
include "memory.inc"
include "HELLO_1.inc"


;-----------------------------------------------
;  Global Symbols
;-----------------------------------------------
export  _HELLO_1_ISR


IF (HELLO_1_RXBUF_ENABLE)
export  HELLO_1_aRxBuffer
export _HELLO_1_aRxBuffer
export  HELLO_1_bRxCnt
export _HELLO_1_bRxCnt
export  HELLO_1_fStatus
export _HELLO_1_fStatus
ENDIF


;-----------------------------------------------
; Variable Allocation
;-----------------------------------------------

IF (HELLO_1_RXBUF_ENABLE)
AREA InterruptRAM(RAM,REL,CON)
 HELLO_1_fStatus:
_HELLO_1_fStatus:      BLK  1
 HELLO_1_bRxCnt:
_HELLO_1_bRxCnt:       BLK  1
AREA HELLO_1_RAM(RAM,REL,CON)
 HELLO_1_aRxBuffer:    
_HELLO_1_aRxBuffer:    BLK HELLO_1_RX_BUFFER_SIZE
ENDIF


AREA InterruptRAM(RAM,REL,CON)

;@PSoC_UserCode_INIT@ (Do not change this line.)
;---------------------------------------------------
; Insert your custom declarations below this banner
;---------------------------------------------------

;------------------------
; Includes
;------------------------

	
;------------------------
;  Constant Definitions
;------------------------


;------------------------
; Variable Allocation
;------------------------


;---------------------------------------------------
; Insert your custom declarations above this banner
;---------------------------------------------------
;@PSoC_UserCode_END@ (Do not change this line.)


AREA UserModules (ROM, REL)

;-----------------------------------------------------------------------------
;  FUNCTION NAME: _HELLO_1_ISR
;
;  DESCRIPTION: Unless modified, this implements only a null handler stub.
;
;-----------------------------------------------------------------------------
;

_HELLO_1_ISR:

   ;@PSoC_UserCode_BODY@ (Do not change this line.)
   ;---------------------------------------------------
   ; Insert your custom code below this banner
   ;---------------------------------------------------
   ;   NOTE: interrupt service routines must preserve
   ;   the values of the A and X CPU registers.

   ;---------------------------------------------------
   ; Insert your custom code above this banner
   ;---------------------------------------------------
   ;@PSoC_UserCode_END@ (Do not change this line.)

 IF (HELLO_1_RXBUF_ENABLE)
   push A
   push X

   IF SYSTEM_LARGE_MEMORY_MODEL
      REG_PRESERVE IDX_PP                                  ; Save the IDX_PP register	
   ENDIF

   mov  X,[HELLO_1_bRxCnt]                                 ; Load X with byte counter
   mov  A,REG[HELLO_1_CONTROL_REG]                         ; Read the control register
   push A                                                  ; Store copy for later test
                                                           ; IF real RX interrupt
   and  A,HELLO_1_RX_REG_FULL                              ; Did really really get an IRQ
   jnz  .UARTRX_ReadRx                                     ; Data ready, go get it
   pop  A                                                  ; Restore stack
   jmp  .RESTORE_IDX_PP

.UARTRX_ReadRx:

   pop  A                                                  ; Restore status flags
                                                           ; IF there is no error, get data
                                                           ; Check for parity or framing error
   and  A,HELLO_1_RX_ERROR
   jz   .UARTRX_NO_ERROR                                   ; If there is not an Error go read data

   or   [HELLO_1_fStatus],A                                ; Set error flags (parity,framing,overrun) bits

   tst  REG[HELLO_1_RX_BUFFER_REG], 0x00                   ; Read the data buffer to clear it.

   and  A,HELLO_1_RX_FRAMING_ERROR                         ; Check for framing error special case
   jz   .RESTORE_IDX_PP                                    ; Not framing error, all done

                                                           ; Disable and re-enable RX to reset after
                                                           ; framing error.
   and   REG[HELLO_1_CONTROL_REG], ~HELLO_1_RX_ENABLE      ; Disable RX
   or    REG[HELLO_1_CONTROL_REG],  HELLO_1_RX_ENABLE      ; Enable RX
   jmp  .RESTORE_IDX_PP                                    ; Done with framing error, leave.


.UARTRX_NO_ERROR:
   mov  A,REG[HELLO_1_RX_BUFFER_REG ]                      ; Read the data buffer

                                                           ; IF buffer not full
   tst  [HELLO_1_fStatus],HELLO_1_RX_BUF_CMDTERM           ; Check for buffer full
   jnz  .RESTORE_IDX_PP                                    ; All done

   cmp  A,HELLO_1_CMD_TERM                                 ; Check for End of command
   jnz  .UARTRX_CHK_CTLCHAR
   or   [HELLO_1_fStatus],HELLO_1_RX_BUF_CMDTERM           ; Set command ready bit

   RAM_SETPAGE_IDX >HELLO_1_aRxBuffer
   RAM_CHANGE_PAGE_MODE FLAG_PGMODE_10b
   mov  [X + HELLO_1_aRxBuffer],00h                        ; Zero out last data
   RAM_CHANGE_PAGE_MODE FLAG_PGMODE_00b
   jmp  .RESTORE_IDX_PP

.UARTRX_CHK_CTLCHAR:                                       ; Ignore charaters below this value
                                                           ; If ignore char is set to 0x00, do not
                                                           ; ignore any characters.
IF(HELLO_1_RX_IGNORE_BELOW)
   cmp  A,HELLO_1_RX_IGNORE_BELOW
   jc   .RESTORE_IDX_PP
ENDIF

.UARTRX_CHK_OVFL:                                          ; Check for MAX String here
   cmp  [HELLO_1_bRxCnt],(HELLO_1_RX_BUFFER_SIZE - 1)
   jc   .UARTRX_ISR_GETDATA
   or   [HELLO_1_fStatus],HELLO_1_RX_BUF_OVERRUN           ; Set error flags (parity,framing,overrun) bits

   RAM_SETPAGE_IDX >HELLO_1_aRxBuffer             ;   using idexed address mode
   RAM_CHANGE_PAGE_MODE FLAG_PGMODE_10b
   mov  [X + HELLO_1_aRxBuffer],00h                        ; Zero out last data
   RAM_CHANGE_PAGE_MODE FLAG_PGMODE_00b
   jmp  .RESTORE_IDX_PP

                                                           ; IF input data == "CR", then end of command
.UARTRX_ISR_GETDATA:
   inc  X                                                  ; Inc the pointer
   mov  [HELLO_1_bRxCnt],X                                 ; Restore the pointer
   dec  X                                                  ; Mov X to its original value

   RAM_SETPAGE_IDX >HELLO_1_aRxBuffer             ;   using idexed address mode
   RAM_CHANGE_PAGE_MODE FLAG_PGMODE_10b
   mov  [X+HELLO_1_aRxBuffer],A                            ; store data in array
   RAM_CHANGE_PAGE_MODE FLAG_PGMODE_00b


.RESTORE_IDX_PP:
   IF SYSTEM_LARGE_MEMORY_MODEL
      REG_RESTORE IDX_PP
   ENDIF

.END_UARTRX_ISR:
   pop  X
   pop  A

ENDIF

HELLO_1_RX_ISR_END:
   reti


; end of file HELLO_1INT.asm
