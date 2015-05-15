        LIST
;*******************************************************************************
; tinyRTX Filename: slcd.asm (System Liquid Crystal Display services)
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
;   23Oct03 SHiggins@tinyRTX.com Created from scratch.
;   29Jul14 SHiggins@tinyRTX.com Changed SLCD_ReadByte to macro to save stack.
;   13Aug14 SHiggins@tinyRTX.com Converted from PIC16877 to PIC18F452.
;   14May15 Stephen_Higgins@KairosAutonomi.com
;               Minimal slcd.asm if board or chip doesn't support it.
;               Substitute #include <ucfg.inc> for <p18f452.inc>.
;               Move #define SLCD_BUFFER_LINE_SIZE to slcduser.inc.
;               Init LCD power pin if needed.
;               Removed unnecessary banksel's for SFR's in access RAM.
;
;*******************************************************************************
;
        errorlevel -302 
;
        #include    <ucfg.inc>      ; Configure board and proc, #include <proc.inc>
        #include    <slcduser.inc>
;
;*******************************************************************************
;
; SLCD defines.
;
#define     SLCD_LCD_BUSY   7       ; LCD Busy Flag, 1 = busy.
#define     SLCD_ICTL_RS    7       ; RS flag, 0 = command, 1 = data.
;
; SLCD service variables.
;
; System Liquid Crystal Display variables.
;
SLCD_UdataSec       UDATA
;
        GLOBAL  SLCD_BufferLine1
SLCD_BufferLine1    res     SLCD_BUFFER_LINE_SIZE
        GLOBAL  SLCD_BufferLine2
SLCD_BufferLine2    res     SLCD_BUFFER_LINE_SIZE
;
    IF (UCFG_BOARD==UCFG_PD2P_2002 || UCFG_BOARD==UCFG_PD2P_2010) && UCFG_PROC==UCFG_18F452
;
;   (UCFG_PD2P_2002 OR UCFG_PD2P_2010 specified) AND UCFG_18F452 specified
;   **********************************************************************
;
;   NOTE: This condition must hold in order to use SLCD services.  That is,
;       the board must have an LCD and the chip has 40-pins.
;
SLCD_DataByteRcv    res 1
SLCD_DataByteXmit   res 1
SLCD_DataXmitCnt    res 1
SLCD_DelayCnt8Hi    res 1
SLCD_DelayCnt8Lo    res 1
SLCD_IctlFlag       res 1
;
;*******************************************************************************
;
SLCD_CodeSec   CODE
;
;*******************************************************************************
;
; Read data byte from LCD and place 8-bit result in SLCD_DataByteRcv.
; Implemented as a macro because call tree analysis identified this routine.
;
SLCD_ReadByte  MACRO
;
        movlw       SLCD_DATA_BITSUSED
        iorwf       SLCD_DATA_TRIS, F               ; Lower 4 bits of port become inputs.
;
        bcf         SLCD_CTRL_PORT, SLCD_CTRL_RS    ; RS = 0 = Command mode.
        bsf         SLCD_CTRL_PORT, SLCD_CTRL_RW    ; RW = 1 = Read mode.
        bsf         SLCD_CTRL_PORT, SLCD_CTRL_E     ; E  = 1 = Begin action. (Read high nibble.)
        nop
        nop
;
        movf        SLCD_DATA_PORT, W           ; Read LCD data high nibble from data port low nibble.
        andlw       SLCD_DATA_BITSUSED          ; Save only valid bits from data port.
        banksel     SLCD_DataByteRcv
        movwf       SLCD_DataByteRcv            ; Save valid bits in result low nibble.
        swapf       SLCD_DataByteRcv, F         ; Save valid bits in result high nibble.
;
        bcf         SLCD_CTRL_PORT, SLCD_CTRL_E ; E  = 0 = End   action.
        nop
        nop
        bsf         SLCD_CTRL_PORT, SLCD_CTRL_E ; E  = 1 = Begin action. (Read low nibble.)
        nop
        nop
;
        movf        SLCD_DATA_PORT, W           ; Read LCD data low  nibble from data port.
        andlw       SLCD_DATA_BITSUSED          ; Save only valid bits from data port.
        banksel     SLCD_DataByteRcv
        addwf       SLCD_DataByteRcv, F         ; Save valid bits in result low nibble.
;
        bcf         SLCD_CTRL_PORT, SLCD_CTRL_E ; E  = 0 = End   action.
        ENDM
;
;*******************************************************************************
;
; Loop until LCD Busy Flag goes low, which indicates LCD not busy.
;
SLCD_ChkBusy
;
        SLCD_ReadByte       ; MACRO to read data byte including LCD Busy Flag.
;
        banksel SLCD_DataByteRcv
        btfsc   SLCD_DataByteRcv, SLCD_LCD_BUSY     ; Skip if LCD not busy so OK to exit.
        bra     SLCD_ChkBusy                        ; Still busy, keep checking.
        return
;
;*******************************************************************************
;
; Write data byte from SLCD_DataByteXmit to LCD.
;   If bit SLCD_IctlFlag, SLCD_ICTL_RS is 0 then RS = 0 = Command mode.
;   If bit SLCD_IctlFlag, SLCD_ICTL_RS is 1 then RS = 1 = Data    mode.
;
SLCD_WriteByte  
;
        rcall       SLCD_ChkBusy  
;
        banksel     SLCD_DataByteXmit
        swapf       SLCD_DataByteXmit, F    ; First nibble to xmit is data high nibble.
        rcall       SLCD_WriteNibble  
;
        banksel     SLCD_DataByteXmit
        swapf       SLCD_DataByteXmit, F    ; Second nibble to xmit is data low nibble.
        rcall       SLCD_WriteNibble  
        return
;
;*******************************************************************************
;
; Write low data nibble from SLCD_DataByteXmit (lower 4 bits) to LCD.
;   If bit SLCD_IctlFlag, SLCD_ICTL_RS is 0 then RS = 0 = Command mode.
;   If bit SLCD_IctlFlag, SLCD_ICTL_RS is 1 then RS = 1 = Data    mode.
;
SLCD_WriteNibble  
;
        movlw       SLCD_DATA_BITSUSED              ; Bits used are set, unused are clear.
        xorlw       0xff                            ; Bits used are clear, unused are set.
        andwf       SLCD_DATA_TRIS, F               ; Lower 4 bits of port become outputs.
;
        banksel     SLCD_IctlFlag
        btfss       SLCD_IctlFlag, SLCD_ICTL_RS     ; Skip if internal flag RS = 1.
        bra         SLCD_WriteNibbleCmd
;
SLCD_WriteNibbleData
        bsf         SLCD_CTRL_PORT, SLCD_CTRL_RS    ; RS = 1 = Data mode.
        bra         SLCD_WriteNibble1
;
SLCD_WriteNibbleCmd
        bcf         SLCD_CTRL_PORT, SLCD_CTRL_RS    ; RS = 0 = Command mode.
;
SLCD_WriteNibble1
        bcf         SLCD_CTRL_PORT, SLCD_CTRL_RW    ; RW = 0 = write mode.
        bsf         SLCD_CTRL_PORT, SLCD_CTRL_E     ; E  = 1 = Begin action.
        nop
        nop
;
; Write lower nibble taking care not to disturb upper nibble port data which may include active outputs.
;
        movlw       SLCD_DATA_BITSUSED              ; Bits used are set, unused are clear.
        xorlw       0xff                            ; Bits used are clear, unused are set.
        andwf       SLCD_DATA_PORT, F               ; Lower 4 bits of port are cleared.
;
        banksel     SLCD_DataByteXmit
        movlw       SLCD_DATA_BITSUSED              ; Bits used are set, unused are clear.
        andwf       SLCD_DataByteXmit, W            ; Get low 4 bits of xmit data, high 4 bits are cleared.
;
        iorwf       SLCD_DATA_PORT, F               ; Send low 4 bits to data port.
        nop
        nop
;
        bcf         SLCD_CTRL_PORT, SLCD_CTRL_E     ; E  = 0 = End   action.
        return
;
;*******************************************************************************
;
SLCD_Delay8  
;
; BANKSEL: Altered.
;
;  Assumes 4MHz clock (1us/cycle).
;  ( ( SLCD_DelayCnt8Lo * 3 ) + 6 ) * ( 1us / cycle ) = Total time elapsed.
;
        banksel SLCD_DelayCnt8Lo    ; Init, call and return use 6 cycles.
        movwf   SLCD_DelayCnt8Lo
;
SLCD_Delay8_Loop                    ; Loop uses 3 cycles each iteration.
        decfsz  SLCD_DelayCnt8Lo, F ; Countdown delay timer.
        bra     SLCD_Delay8_Loop    ; Loop until timer expires.
        return
;
;*******************************************************************************
;
SLCD_Delay16  
;
; BANKSEL: Altered.
;
;  Assumes 4MHz clock (1us/cycle).
;    ( SLCD_DelayCnt8Hi * 3 * 256 ) * ( 1us / cycle ) = Total time elapsed.
;
; Approx 770us * W.
;
        banksel SLCD_DelayCnt8Hi        ; Init, call and return use 4 cycles.
        movwf   SLCD_DelayCnt8Hi
        clrf    SLCD_DelayCnt8Lo
;
SLCD_Delay16_Loop                       ; Inner loop uses 3 cycles each iteration.
        decfsz  SLCD_DelayCnt8Lo, F     ; Countdown delay timer.
        bra     SLCD_Delay16_Loop       ; Loop until inner timer expires.

        decfsz  SLCD_DelayCnt8Hi, F     ; Countdown delay timer.
        bra     SLCD_Delay16_Loop       ; Loop until outer timer expires.
        return
;
;*******************************************************************************
;
        GLOBAL  SLCD_Init  
SLCD_Init  
;
        bcf         SLCD_CTRL_PORT, SLCD_CTRL_RS    ; RS = 0 (init).
        bcf         SLCD_CTRL_PORT, SLCD_CTRL_RW    ; RW = 0 (init).
        bcf         SLCD_CTRL_PORT, SLCD_CTRL_E     ; E  = 0 (init).
;
        bcf         SLCD_CTRL_TRIS, SLCD_CTRL_RS    ; RS = output.
        bcf         SLCD_CTRL_TRIS, SLCD_CTRL_RW    ; RW = output.
        bcf         SLCD_CTRL_TRIS, SLCD_CTRL_E     ; E  = output.
;
;   PICDEM2plus 2010 board also has a power control line.
;
    IF UCFG_BOARD==UCFG_PD2P_2010
        bcf         SLCD_CTRL_PORT, SLCD_CTRL_PWR   ; PWR = 0 (init).
        bcf         SLCD_CTRL_TRIS, SLCD_CTRL_PWR   ; PWR = output.
        bsf         SLCD_CTRL_PORT, SLCD_CTRL_PWR   ; PWR = 1 (turn on).
    ENDIF
;
; All writes in this routine are command (not data) writes.
;
        banksel     SLCD_IctlFlag       
        bcf         SLCD_IctlFlag, SLCD_ICTL_RS ; RS = 0 = Command mode.
;
; Wait min 15ms after LCD Vdd rises above 4.5V.  Cannot check Busy Flag.
;
        movlw       0x14                        ; 20 * 770us = 15ms. (@ 4MHz clk)
        rcall       SLCD_Delay16  
;
; Function set (8-bit interface), followed by min 4.1 ms wait.  Cannot check Busy Flag.
;
        movlw       0x03                        ; Control seq #1: 0b0011 nibble
        banksel     SLCD_DataByteXmit
        movwf       SLCD_DataByteXmit
        rcall       SLCD_WriteNibble  
        movlw       0x06                        ; 6 * 770us = 4.6 ms. (@ 4MHz clk)
        rcall       SLCD_Delay16  
;
; Function set (8-bit interface), followed by min 100 us wait.  Cannot check Busy Flag.
;
        movlw       0x03                        ; Control seq #2: 0b0011 nibble
        banksel     SLCD_DataByteXmit
        movwf       SLCD_DataByteXmit
        rcall       SLCD_WriteNibble  
        movlw       0x20                        ; 32 * 3us = 100us. (@ 4MHz clk)
        rcall       SLCD_Delay8  
;
; Function set (8-bit interface), followed by min 100 us wait.
;   Unclear whether we can check the Busy Flag, so just wait 100 us.
;
        movlw       0x03                        ; Control seq #3: 0b0011 nibble
        banksel     SLCD_DataByteXmit
        movwf       SLCD_DataByteXmit
        rcall       SLCD_WriteNibble  
        movlw       0x20                        ; 32 * 3us = 100us. (@ 4MHz clk)
        rcall       SLCD_Delay8  
;
; Function set (DL = 0 = 4-bit interface).
; After this instruction we may check Busy Flag.
; Using WriteNibble means we have to explicitly check BF here beforehand.
;
        rcall       SLCD_ChkBusy  
;
        movlw       0x02                        ; Control seq #4: 0b0010 nibble
        banksel     SLCD_DataByteXmit
        movwf       SLCD_DataByteXmit
        rcall       SLCD_WriteNibble  
;
; Function set (DL = 0 = 4-bit interface, N = 1 = 2 display lines, F = 0 = 5x7 chars).
; WriteByte does BF check from now on.
;
        movlw       0x28                        ; Control seq #5.
        banksel     SLCD_DataByteXmit
        movwf       SLCD_DataByteXmit
        rcall       SLCD_WriteByte  
;
; Display on/off (D = 1 = display on, S/C = 0 = no shift, B = 0 = cursor blink off).
;
        movlw       0x0c                        ; Control seq #6.
        banksel     SLCD_DataByteXmit
        movwf       SLCD_DataByteXmit
        rcall       SLCD_WriteByte  
;
; Clear display.
;
        movlw       0x01                        ; Control seq #7.
        banksel     SLCD_DataByteXmit
        movwf       SLCD_DataByteXmit
        rcall       SLCD_WriteByte  
;
; Entry mode set (I/D = 1 = increment cursor position, S = 0 = no display shift).
;
        movlw       0x06                        ; Control seq #8.
        banksel     SLCD_DataByteXmit
        movwf       SLCD_DataByteXmit
        rcall       SLCD_WriteByte  
;
; DDRAM address set (address = 0bX0000000 (7 bits)).
;
        movlw       0x80                        ; Control seq #9.
        banksel     SLCD_DataByteXmit
        movwf       SLCD_DataByteXmit
        rcall       SLCD_WriteByte  
;
        return
;
;*******************************************************************************
;
; Refresh hardware LCD Line 1 with contents of SLCD_BufferLine1.
;
        GLOBAL  SLCD_RefreshLine1
SLCD_RefreshLine1
;
; Write SLCD_BufferLine1 to LCD.
;
        banksel     SLCD_IctlFlag       
        bcf         SLCD_IctlFlag, SLCD_ICTL_RS ; RS = 0 = Command mode.
        movlw       0x80                        ; Command: Set LCD cursor to line 1.
        movwf       SLCD_DataByteXmit
        rcall       SLCD_WriteByte              ; Write command to LCD.
;
        movlw       SLCD_BUFFER_LINE_SIZE
        banksel     SLCD_DataXmitCnt
        movwf       SLCD_DataXmitCnt            ; Set transmit count to all of Line 1.
;
        bsf         SLCD_IctlFlag, SLCD_ICTL_RS ; RS = 1 = Data mode.
        lfsr        0, SLCD_BufferLine1         ; Indirect pointer gets start address.
;
SLCD_RefreshLoop1
        movff       POSTINC0, SLCD_DataByteXmit ; Get data at pointer.
        rcall       SLCD_WriteByte              ; Write data to LCD.
        banksel     SLCD_DataXmitCnt
        decfsz      SLCD_DataXmitCnt, F         ; Dec count of data to xmit.
        bra         SLCD_RefreshLoop1           ; Loop if data count not zero.
;                                              
        return
;
;*******************************************************************************
;
; Refresh hardware LCD Line 2 with contents of SLCD_BufferLine2.
;
        GLOBAL  SLCD_RefreshLine2
SLCD_RefreshLine2
;
; Write SLCD_BufferLine2 to LCD.
;
        banksel     SLCD_IctlFlag       
        bcf         SLCD_IctlFlag, SLCD_ICTL_RS ; RS = 0 = Command mode.
        movlw       0xc0                        ; Command: Set LCD cursor to line 2.
        movwf       SLCD_DataByteXmit
        rcall       SLCD_WriteByte              ; Write command to LCD.
;
        movlw       SLCD_BUFFER_LINE_SIZE
        banksel     SLCD_DataXmitCnt
        movwf       SLCD_DataXmitCnt            ; Set transmit count to all of Line 2.
;
        bsf         SLCD_IctlFlag, SLCD_ICTL_RS ; RS = 1 = Data mode.
        lfsr        0, SLCD_BufferLine2         ; Indirect pointer gets start address.
;
SLCD_RefreshLoop2
        movff       POSTINC0, SLCD_DataByteXmit ; Get data at pointer.
        rcall       SLCD_WriteByte              ; Write data to LCD.
        banksel     SLCD_DataXmitCnt
        decfsz      SLCD_DataXmitCnt, F         ; Dec count of data to xmit.
        bra         SLCD_RefreshLoop2           ; Loop if data count not zero.
;                                              
        return
;
    ENDIF
        end
