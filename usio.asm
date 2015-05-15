        LIST
;*******************************************************************************
; tinyRTX Filename: usio.asm (User Serial I/O communication routines)
;
; Copyright 2015 Sycamore Software, Inc.  ** www.tinyRTX.com **
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
;   17Apr15 Stephen_Higgins@KairosAutonomi.com
;               Created from ui2c.asm.
;   27Apr15 Stephen_Higgins@KairosAutonomi.com
;               Added USIO_TxLCDMsgToSIO.
;
;*******************************************************************************
;
        errorlevel -302 
;
        #include    <ucfg.inc>  ; Configure board and proc, #include <proc.inc>
        #include    <srtx.inc>
        #include    <ssio.inc>
        #include    <slcd.inc>
        #include    <slcduser.inc>
;
;*******************************************************************************
;
; User SIO defines.
;
    IF UCFG_BOARD==UCFG_PD2P_2002 || UCFG_BOARD==UCFG_PD2P_2010
;
;   UCFG_PD2P_2002 or UCFG_PD2P_2010 specified.
;   *******************************************
;
;       UCFG_18F2620 specified.
;       ***********************
;
        IF UCFG_PROC==UCFG_18F2620
#define USIO_SPBRGH_VAL .0
#define USIO_SPBRG_VAL  .8
;
; Baud rate for 4Mhz clock, BRGH = 1, BRG16 = 1
;
;   USIO_SPBRG_VAL  .103    =   9.6K
;   USIO_SPBRG_VAL  .51     =  19.2K
;   USIO_SPBRG_VAL  .16     =  57.6K
;   USIO_SPBRG_VAL  .8      = 115.2K
;
        ENDIF
;
;       UCFG_18F452 specified.
;       **********************
;
        IF UCFG_PROC==UCFG_18F452
#define USIO_SPBRG_VAL  .12
;
; Baud rate for 4Mhz clock, BRGH = 1
;
;   USIO_SPBRG_VAL  .25     =   9.6K
;   USIO_SPBRG_VAL  .12     =  19.2K
;
        ENDIF
    ENDIF
;
    IF UCFG_BOARD==UCFG_DJPCB_280B
;
;   UCFG_DJPCB_280B specified.
;   **************************
;
#define USIO_SPBRGH_VAL .0
#define USIO_SPBRG_VAL  .86
;
; Baud rate for 40 Mhz clock, BRGH = 1, BRG16 = 1
;
;   SPBRG  .1040    =   9.6K    
;   SPBRG  .520     =  19.2K
;   SPBRG  .172     =  57.6K
;   SPBRG  .86      = 115.2K    (USIO_SPBRGH_VAL = 0, USIO_SPBRG_VAL = 86)
;
    ENDIF
;
#define USIO_TXSTA_VAL  0x24
;
; bit 7 : CSRC  : 0 : Don't care (Asynch mode)
; bit 6 : TX9   : 0 : 8-bit transmission
; bit 5 : TXEN  : 1 : Transmit enabled
; bit 4 : SYNC  : 0 : Asynchronous mode
; bit 3 : SENDB : 0 : Sync Break transmission completed
; bit 2 : BRGH  : 1 : High speed Baud rate
; bit 1 : TRMT  : 0 : Transmit Shift Register full
; bit 0 : TX9D  : 0 : Don't care (9th bit data)
;
#define USIO_RCSTA_VAL  0x90
;
; bit 7 : SPEN  : 1 : Serial port enabled (RX/DT and TX/CK used)
; bit 6 : RX9   : 0 : 8-bit reception
; bit 5 : SREN  : 0 : Don't care (Asynch mode)
; bit 4 : CREN  : 1 : Continuous Receive enabled
; bit 3 : ADDEN : 0 : Don't care (Asynch mode, RX9=0)
; bit 2 : FERR  : 0 : No framing error
; bit 1 : OERR  : 0 : No overrun error
; bit 0 : RX9D  : 0 : Don't care (9th bit data)
;
#define USIO_BAUDCON_VAL    0x48 ;0x40
;
; bit 7 : ABDOVF: 0 : No BRG rollover
; bit 6 : RCIDL : 1 : Receive operation idle
; bit 5 : RXDTP : 0 : RX data not inverted
; bit 4 : TXCKP : 0 : Idle state for TX is high
; bit 3 : BRG16 : 1 : 16-bit, uses SPBRG and SPBRGH
; bit 2 : BAUD2 : 0 : Not implemented (not writable)
; bit 1 : WUE   : 0 : RX pin not monitored, no edge detection
; bit 0 : ABDEN : 0 : Autobaud disabled or completed
;
;*******************************************************************************
;
; User SIO service variables.
;
SIO_UdataSec   UDATA
;
USIO_TempData       res     1   ; Temporary data.
USIO_DataXferCnt    res     1   ; Data transfer counter.
;
;*******************************************************************************
;
; Init SIO hardware.  Assumes GIE and PEIE already enabled.
;
USIO_CodeSec    CODE
;
        GLOBAL  USIO_Init
USIO_Init
;
; PIE1 changed: TRISC7, TRISC6 enabled.
;
        bsf     TRISC, TRISC7       ; Enable USART control of RC pin.
        bsf     TRISC, TRISC6       ; Enable USART control of TX pin.
;
    IF UCFG_PROC==UCFG_18F2620
        movlw   USIO_BAUDCON_VAL    ; Set baud rate.
        movwf   BAUDCON
        movlw   USIO_SPBRGH_VAL     ; Set baud rate.
        movwf   SPBRGH
    ENDif
;
        movlw   USIO_SPBRG_VAL      ; Set baud rate.
        movwf   SPBRG
;
        movlw   USIO_TXSTA_VAL      ; Enable transmission and high baud rate.
        movwf   TXSTA
;
        movlw   USIO_RCSTA_VAL      ; Enable serial port and reception and 16-bit BRG.
        movwf   RCSTA
;
        call    SSIO_InitFlags      ; Initialize SSIO internal flags.
        call    SSIO_InitTxBuffer   ; Initialize transmit buffer.
        call    SSIO_InitRxBuffer   ; Initialize receive buffer.
;
; PIE1 changed: RCIE, TXIE enabled.
;
        bsf     PIE1, RCIE          ; Enable RX interrupts.
;
; Don't set TXIE until something is actually written to the TX buffer.
; Otherwise we're going to get a TX int right now because at init the TX hardware
; is empty and TXIF is set.
;
        return
;
;*******************************************************************************
;
; USIO_TxLCDMsgToSIO is called from SUSR_TaskADC when an A/D conversion completes. 
; It moves the message from the (unused) SLCD buffer to the SIO transmit buffer,
; effectively replacing the LCD function with a transmit over the SIO (RS-232).
;
; Note that we use FSR1 because SSIO routines use FSR0. 
;
        GLOBAL  USIO_TxLCDMsgToSIO
USIO_TxLCDMsgToSIO
;
        movlw   SLCD_BUFFER_LINE_SIZE       ; Size of source buffer.
        banksel USIO_DataXferCnt
        movwf   USIO_DataXferCnt            ; Size saved in data transfer counter.
;
        lfsr    1, SLCD_BufferLine2         ; Data pointer gets source start address.
;
USIO_TxLCDMsgToSIO_NextByte
;
        movf    POSTINC1, W                 ; Get char from source LCD buffer.
        call    SSIO_PutByteTxBuffer        ; Move char to dest SIO Tx buffer.
;
        banksel USIO_DataXferCnt
        decfsz  USIO_DataXferCnt, F         ; Dec count of data to copy, skip if all done.
        bra     USIO_TxLCDMsgToSIO_NextByte ; More data to copy.
;
        movlw   0x0d
        call    SSIO_PutByteTxBuffer        ; Move <CR> to dest SIO Tx buffer.
        movlw   0x0a
        call    SSIO_PutByteTxBuffer        ; Move <LF> to dest SIO Tx buffer.
        return
;
;*******************************************************************************
;
; USIO_MsgReceived is called from SSIO/SUSR when an SIO message completes. 
; It moves the message from the receive buffer to the transmit buffer, effectively
;   echoing it back to the sender.
;
        GLOBAL  USIO_MsgReceived
USIO_MsgReceived
;
        call    SSIO_GetByteRxBuffer    ; Get data from receive buffer.
        banksel USIO_TempData
        movwf   USIO_TempData           ; Save data to test if it is <CR>.
;
        call    SSIO_PutByteTxBuffer    ; Copy data into transmit buffer.
        banksel USIO_TempData
        movf    USIO_TempData, W        ; Retrieve data.
;
        xorlw   0x0d                    ; Compare with <CR>. 
        bnz     USIO_MsgReceived        ; If data not <CR> then move another byte.
;
        movlw   0x0a
        call    SSIO_PutByteTxBuffer    ; Move <LF> to dest SIO Tx buffer.
;
        return
        end