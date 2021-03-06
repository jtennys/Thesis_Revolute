;;*****************************************************************************
;;*****************************************************************************
;;  FILENAME: RX8_6.inc
;;   Version: 3.3, Updated on 2009/7/10 at 10:46:15
;;  Generated by PSoC Designer 5.0.985.0
;;
;;  DESCRIPTION: Assembler declarations for the RX8 user module interface
;;               for the 22/24/25/26/27xxx PSoC family of devices.
;;-----------------------------------------------------------------------------
;;  Copyright (c) Cypress MicroSystems 2000-2003. All Rights Reserved.
;;*****************************************************************************
;;*****************************************************************************

include "m8c.inc"

;--------------------------------------------------
; Constants for RX8_6 API's
;--------------------------------------------------

;mask value for global int reg bit for RX instance
RX8_6_bINT_MASK:        equ 80h
;RX8 interrupt address
RX8_6_INT_REG:          equ 0e1h

RX8_6_RXBUF_ENABLE:                         equ   1

RX8_6_RX_IGNORE_BELOW:                       equ  20h
RX8_6_RX_BUFFER_SIZE:                        equ  10h
RX8_6_CMD_TERM:                              equ  dh
RX8_6_DELIMITER:                             equ  20h

; This equate will be removed in future revisions.
; Do not use.
bRX8_6_INT_MASK:        equ 80h
;------------------------------------
;  RX8 Parity masks
;------------------------------------
RX8_6_PARITY_NONE:           equ   0h
RX8_6_PARITY_EVEN:           equ   2h
RX8_6_PARITY_ODD:            equ   6h

;------------------------------------
;  RX8 Status Register masks
;------------------------------------

RX8_6_RX_ACTIVE:             equ   10h
RX8_6_RX_COMPLETE:           equ   08h
RX8_6_RX_REG_FULL:           equ   08h
RX8_6_RX_PARITY_ERROR:       equ   80h
RX8_6_RX_OVERRUN_ERROR:      equ   40h
RX8_6_RX_FRAMING_ERROR:      equ   20h
RX8_6_RX_ERROR:              equ   E0h
RX8_6_RX_NO_ERROR:           equ   E0h
RX8_6_RX_ENABLE:             equ   01h

RX8_6_RX_BUF_ERROR:                     equ   F0h  ; Mask for any Rx that may occur.
RX8_6_RX_BUF_OVERRUN:                   equ   10h  ; This indicates the software buffer has
                                                        ; been over run.
RX8_6_RX_BUF_CMDTERM:                   equ   01h  ; Command terminator has been received.

RX8_6_RX_NO_DATA:                       equ   01h
RX8_6_RX_NO_ERROR:                      equ   E0h

; Old equates, will be removed in future release
; Do not use
RX8_PARITY_NONE:           equ   0h
RX8_PARITY_EVEN:           equ   2h
RX8_PARITY_ODD:            equ   6h
RX8_RX_ACTIVE:             equ   10h
RX8_RX_COMPLETE:           equ   08h
RX8_RX_PARITY_ERROR:       equ   80h
RX8_RX_OVERRUN_ERROR:      equ   40h
RX8_RX_FRAMING_ERROR:      equ   20h
RX8_RX_NO_ERROR:           equ   E0h


;--------------------------------------------------
; Register Address Constants for RX8_6
;--------------------------------------------------
RX8_6_CONTROL_REG:  equ 3fh                      ;Control register
RX8_6_RX_SHIFT_REG: equ 3ch                      ;RX Shift Register register
RX8_6_RX_BUFFER_REG:    equ 3eh                  ;RX Buffer Register
RX8_6_FUNC_REG: equ 3ch                          ;Function register
RX8_6_INPUT_REG:    equ 3dh                      ;Input register
RX8_6_OUTPUT_REG:   equ 3eh                      ;Output register

; end of file RX8_6.inc
