        LIST
;*******************************************************************************
; tinyRTX Filename: strc.asm (System TRaCe service)
;
; Copyright 2014 Sycamore Software, Inc.  ** tinyRTX.com **
; Distributed under the terms of the GNU Lesser General Purpose License v3
;
; This file is part of tinyRTX. tinyRTX is free software: you can redistribute
; it and/or modify it under the terms of the GNU Lesser General Public License
; version 3 as published by the Free Software Foundation.
;
; tinyRTX is distributed in the hope that it will be useful, but WITHOUT ANY
; WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
; A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more
; details.
;
; You should have received a copy of the GNU Lesser General Public License
; (filename copying.lesser.txt) and the GNU General Public License (filename
; copying.txt) along with tinyRTX.  If not, see <http://www.gnu.org/licenses/>.
;
; Revision history:
;   20Feb04 SHiggins@tinyRTX.com    Created from scratch.
;   13Aug14 SHiggins@tinyRTX.com    Converted from PIC16877 to PIC18F452.
;   02Sep14 SHiggins@tinyRTX.com
;               Added interrupt protection to ensure trace validity.
;               However it does not seem to be sufficient, and
;               calling smTrace in an ISR such as TaskI2C clobbers
;               both the trace buffer and RAM.  Unresolved.
;   14May15 Stephen_Higgins@KairosAutonomi.com  
;               Substitute #include <ucfg.inc> for <p18f452.inc>.
;   21May15 Stephen_Higgins@KairosAutonomi.com  
;               Define trace buffer size so all trace RAM fills one bank.
;
;*******************************************************************************
;
        errorlevel -302 
;
        #include    <ucfg.inc>  ; Configure board and proc, #include <proc.inc>
;
;*******************************************************************************
;
; STRC trace buffer and related service variables.
;
STRC_UdataSec   UDATA
;
#define STRC_BUFFER_SIZE 0xFC               ; Define trace buffer size so all trace RAM fills 1 bank.
;
STRC_Idx        res     1                   ; Trace buffer current index.
STRC_PtrH       res     1                   ; Pointer to current location in trace buffer (high nibble).
STRC_PtrL       res     1                   ; Pointer to current location in trace buffer (low byte).
STRC_TempINTCON res     1                   ; Saved copy of INTCON.
STRC_Buffer     res     STRC_BUFFER_SIZE    ; Trace buffer.
;
;*******************************************************************************
;
; STRC Trace Service.
;
STRC_CodeSec  CODE  
;
; Initialize trace buffer.
;
        GLOBAL  STRC_Init
STRC_Init
        lfsr        0, STRC_Buffer      ; Buffer start addr goes in FSR0.
        movff       FSR0L, STRC_PtrL    ; Buffer start addr also goes in pointer for STRC_Trace.
        movff       FSR0H, STRC_PtrH    ;
        movlw       STRC_BUFFER_SIZE    ; Get count of locations to clear now.
        banksel     STRC_Idx            ;
        movwf       STRC_Idx            ; Save count here because var not otherwise used in init.
;
STRC_InitLoop
        clrf        POSTINC0            ; Clear buffer location.
        decfsz      STRC_Idx            ; Dec cnt locations, skip if 0 means all are cleared.
        bra         STRC_InitLoop       ; No skip means no match means more init.
;
        clrf        STRC_Idx            ; Clear current buffer index for STRC_Trace. (Redundant I know. :-)
        return
;
;*******************************************************************************
;
;   Value in W is stored at location pointed to by STRC_Ptr++.
;
        GLOBAL  STRC_Trace
STRC_Trace
;
        movff       INTCON, STRC_TempINTCON ; Save current INTCON.GIE.
        bcf         INTCON, GIE             ; Disable interrupts.
;
        movff       STRC_PtrL, FSR0L    ; Get current buffer addr.
        movff       STRC_PtrH, FSR0H    ;
        movwf       POSTINC0            ; Save input arg in buffer. (Old code did pre-increment.)
;
        banksel     STRC_Idx
        incf        STRC_Idx            ; Increment count of stored traces.
        movlw       STRC_BUFFER_SIZE    ; Get max count of traces to store.
        cpfseq      STRC_Idx            ; Skip if stored traces == max count.
        bra         STRC_TraceExit      ; No skip means no match so count and addr OK.
;
; Trace buffer is full.  Reset pointer to beginning of buffer and zero count.
;
STRC_TraceFull
        lfsr        0, STRC_Buffer      ; Buffer start addr goes in FSR0.
        clrf        STRC_Idx            ; Clear current buffer index for STRC_Trace.
;  
STRC_TraceExit
        movff       FSR0L, STRC_PtrL    ; Save pointer to next addr to store trace.
        movff       FSR0H, STRC_PtrH    ;
        btfsc       STRC_TempINTCON, GIE    ; If saved GIE was set..
        bsf         INTCON, GIE             ; ..then re-enable interrupts.
        return
        end