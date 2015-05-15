        LIST
;*******************************************************************************
; tinyRTX Filename: ulcd.asm (User Liquid Crystal Display routines)
;
; Copyright 2014 Sycamore Software, Inc.  ** www.tinyRTX.com **
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
;   21Oct03 SHiggins@tinyRTX.com Created from scratch for PICdem2plus demo board.
;   11Aug14 SHiggins@tinyRTX.com Change ULCD data from Tiny to tiny.
;   13Aug14 SHiggins@tinyRTX.com Converted from PIC16877 to PIC18F452.
;   14Aug14 SHiggins@tinyRTX.com Rewrote ULCD_TableLookup from dt to tblrd.
;   14Apr15 Stephen_Higgins@KairosAutonomi.com
;               Insert SLCD service variables from slcd.asm.
;   27Apr15 Stephen_Higgins@KairosAutonomi.com
;               Move SLCD service variables into minimal slcd.asm.
;   14May15 Stephen_Higgins@KairosAutonomi.com  
;               Substitute #include <ucfg.inc> for <p18f452.inc>.
;
;*******************************************************************************
;
        errorlevel -302 
;
        #include    <ucfg.inc>  ; Configure board and proc, #include <proc.inc>
        #include    <sm16.inc>
        #include    <slcd.inc>
;
;*******************************************************************************
;
; User LCD defines.
;
#define ULCD_LINE_1_START   0x00
#define ULCD_LINE_1_END     0x50
#define ULCD_LINE_1_LENGTH  0x10
;
;*******************************************************************************
;
; User LCD service variables.
;
ULCD_UdataSec       UDATA
;
ULCD_DataXferCnt    res     1   ; Data transfer counter.
ULCD_PositionIdx    res     1   ; Position index (0x00-0x40)
;
; These ULCD_VoltAscii0-4 variables must be kept sequential.
;
        GLOBAL  ULCD_VoltAscii0
ULCD_VoltAscii0     res     1   ; ASCII A/D result, char 0.
ULCD_VoltAscii1     res     1   ; ASCII A/D result, char 1.
ULCD_VoltAscii2     res     1   ; ASCII A/D result, char 2.
ULCD_VoltAscii3     res     1   ; ASCII A/D result, char 3.
ULCD_VoltAscii4     res     1   ; ASCII A/D result, char 4.
;
; These ULCD_TempAscii0-2 variables must be kept sequential.
;
        GLOBAL  ULCD_TempAscii0
ULCD_TempAscii0     res     1   ; ASCII temperature result, char 0.
ULCD_TempAscii1     res     1   ; ASCII temperature result, char 1.
ULCD_TempAscii2     res     1   ; ASCII temperature result, char 2.
;
;*******************************************************************************
;
; User LCD display table.
;
; Input argument: desired index in W.
; No restrictions on where ULCD_Table may be placed, this code works everywhere.
;
; Because of the overhead of computing the table address, we use this code at the beginning
; only once to set up the TBLPTR registers, and do the table read here for the first char.
; For all subsequent chars we use tblrd*+ in the calling loop to blast through the rest of
; the ULCD_LINE_1_LENGTH characters.  The table read will continue to work because the 
; TBLPTR registers are set up correctly.
;
ULCD_TableSec       CODE
ULCD_TableLookup
        addlw   low ULCD_Table      ; Lower 8 bits of table address added to offset in W.
        movwf   TBLPTRL
        movlw   high ULCD_Table     ; "High" (middle) 8 bits of table address.
        movwf   TBLPTRH             
        btfsc   STATUS, C           ; Skip if no carry out of low 8 bits + W.
        incf    TBLPTRH             ; Carry out of low bits means add 1 to middle 8 bits.
        movlw   upper ULCD_Table    ; Upper 5 bits of table address.
        movwf   TBLPTRU             
        btfsc   STATUS, C           ; Skip if no carry out of middle 8 bits + (possibly) 1.
        incf    TBLPTRH             ; Carry out of middle bits means add 1 to upper bits.
        tblrd*+                     ; Read the table.
        movf    TABLAT, W           ; Move table result into W.
        return
;
ULCD_Table
;           "--Display-Data--"  ; offset.
    data    "                "  ; 0x00.
    data    "tinyRTX from... "  ; 0x10.
    data    "Sycamore Softwar"  ; 0x20.
    data    "e, Inc.     SHig"  ; 0x30.
    data    "gins@tinyRTX.com"  ; 0x40.
    data    "                "  ; 0x00 and identically 0x50.

;
;*******************************************************************************
;
; Init LCD display variables.
;
ULCD_CodeSec        CODE
;
        GLOBAL      ULCD_Init
ULCD_Init
        movlw       ULCD_LINE_1_START
        banksel     ULCD_PositionIdx
        movwf       ULCD_PositionIdx
        return
;
;*******************************************************************************
;
; Refresh contents of LCD Line 1 display buffer with current data.
;
        GLOBAL      ULCD_RefreshLine1
ULCD_RefreshLine1
;
        movlw       ULCD_LINE_1_LENGTH  ; Copy entire first line of table to display buffer.
        banksel     ULCD_DataXferCnt
        movwf       ULCD_DataXferCnt
        movf        ULCD_PositionIdx, W ; Saved table start addr into W for lookup argument.
        lfsr        0, SLCD_BufferLine1 ; Indirect pointer gets dest start address.
;
        rcall       ULCD_TableLookup    ; Get data at current index and set TBLPTR registers.
        movwf       POSTINC0            ; Store data in dest data buffer.
        decf        ULCD_DataXferCnt, F ; Dec count of data to copy.
;
ULCD_RefreshLine1Loop
        tblrd*+                             ; Read the table now that the registers are set.
        movff       TABLAT, POSTINC0        ; Move table result into dest data buffer.
        decfsz      ULCD_DataXferCnt, F     ; Dec count of data to copy, skip if all done.
        bra         ULCD_RefreshLine1Loop   ; More data to copy.
;
        incf        ULCD_PositionIdx, F ; Bump saved table start address.
        movlw       ULCD_LINE_1_END     ; Address past last valid start address.
        cpfseq      ULCD_PositionIdx    ; Skip if W == ULCD_PositionIdx, now past last valid address.
        bra         ULCD_RefreshLine1Exit       ; W <> ULCD_PositionIdx, start addr still valid.
        movlw       ULCD_LINE_1_START   ; Get first valid addr.
        banksel     ULCD_PositionIdx
        movwf       ULCD_PositionIdx    ; Reset start addr to first valid addr.
;
ULCD_RefreshLine1Exit
        return
;
;*******************************************************************************
;
; Refresh contents of LCD Line 2 display buffer with current data.
;
        GLOBAL      ULCD_RefreshLine2
ULCD_RefreshLine2
        lfsr        0, SLCD_BufferLine2         ; Indirect pointer gets dest start address.
        movff       ULCD_VoltAscii0, POSTINC0   ; Char 0 (volts ones) to dest ASCII buffer.
        movff       ULCD_VoltAscii1, POSTINC0   ; Char 1 (volts decimal point) to dest ASCII buffer.
        movff       ULCD_VoltAscii2, POSTINC0   ; Char 2 (volts tenths) to dest ASCII buffer.
        movff       ULCD_VoltAscii3, POSTINC0   ; Char 3 (volts hundredths) to dest ASCII buffer.
        movff       ULCD_VoltAscii4, POSTINC0   ; Char 4 (volts thousandths) to dest ASCII buffer.
        movlw       ' '                         ; Char 5 = ASCII constant.
        movwf       POSTINC0                    ; Char 5 to dest ASCII buffer.
        movlw       'V'                         ; Char 6 = ASCII constant.
        movwf       POSTINC0                    ; Char 6 to dest ASCII buffer.
        movlw       ' '                         ; Char 7 = ASCII constant.
        movwf       POSTINC0                    ; Char 7 to dest ASCII buffer.
        movlw       '+'                         ; Char 8 = ASCII pos/neg sign.
        movwf       POSTINC0                    ; Char 8 to dest ASCII buffer.
        movff       ULCD_TempAscii0, POSTINC0   ; Char 9 (degrees hundreds) to dest ASCII buffer.
        movff       ULCD_TempAscii1, POSTINC0   ; Char 10 (degrees tens) to dest ASCII buffer.
        movff       ULCD_TempAscii2, POSTINC0   ; Char 11 (degrees ones) to dest ASCII buffer.
        movlw       ' '                         ; Char 12 = ASCII constant.
        movwf       POSTINC0                    ; Char 12 to dest ASCII buffer.
        movlw       'd'                         ; Char 13 = ASCII constant.
        movwf       POSTINC0                    ; Char 13 to dest ASCII buffer.
        movlw       'g'                         ; Char 14 = ASCII constant.
        movwf       POSTINC0                    ; Char 14 to dest ASCII buffer.
        movlw       'C'                         ; Char 15 = ASCII constant.
        movwf       POSTINC0                    ; Char 15 to dest ASCII buffer.
        return
;
        end