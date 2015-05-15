        LIST
;*******************************************************************************
; tinyRTX Filename: ssio.asm (System Serial I/O communication services)
;             Assumes USART module is available on chip.
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
; AS THIS SOFTWARE WAS DERIVED FROM SOFTWARE WRITTEN BY MICROCHIP, THIS LICENSE IS
; SUBORDINATE TO THE RESTRICTIONS IMPOSED BY THE ORIGINAL MICROCHIP TECHNOLOGIES
; LICENSE INCLUDED BELOW IN ITS ENTIRETY.
;
; Revision history:
;   16Apr15 Stephen_Higgins@KairosAutonomi.com
;               Modified from p18_tiri.asm from
;               Mike Garbutt at Microchip Technology Inc. All the interrupt discovery
;               was ripped out as tinyRTX already does that in SISD.  All the
;               "application loop" code was ripped out as that is handled by SRTX,
;               and the copy RX to TX action upon detecting <CR> became a tinyRTX user task.
;               The old GetData was rewritten to schedule that user task.
;               Also the high/low interrupt priorities replaced by non-prioritized ints.
;               Lots of banksel directives added, as now interfacing with code and variables 
;               that are not necessarily local. 
;   29Apr15 Stephen_Higgins@KairosAutonomi.com
;               Optimized for locating all variables in same RAM page.
;               Code now (just barely) supports 115.2K baud operation with 4 Mhz clock.
;               Added persistant error counters.
;               Removed redundant "buffer full" error checking.
;               Fixed a fatal flaw in original code, where for receieve buffer overrun error,
;               "ReceivedCR" is set but not stored in receive buffer.  This will trigger 
;               transfer of data from receive to transmit buffer, and then that code will never
;               find a CR to stop the transfer.  Instead it will get data of 0, and write that
;               over the transmit buffer FOREVER.  Solution is to always store last received
;               byte in receive buffer, so transfer routine can find it and quit.
;
;*******************************************************************************
;
        errorlevel -302 
;
        #include    <ucfg.inc>  ; Configure board and proc, #include <proc.inc>
        #include    <srtx.inc>
;       #include    <si2cuser.inc>
        #include    <susr.inc>
;
;=============================================================================
; Software License Agreement
;
; The software supplied herewith by Microchip Technology Incorporated 
; (the "Company") for its PICmicro® Microcontroller is intended and 
; supplied to you, the Company’s customer, for use solely and 
; exclusively on Microchip PICmicro Microcontroller products. The 
; software is owned by the Company and/or its supplier, and is 
; protected under applicable copyright laws. All rights are reserved. 
; Any use in violation of the foregoing restrictions may subject the 
; user to criminal sanctions under applicable laws, as well as to 
; civil liability for the breach of the terms and conditions of this 
; license.
;
; THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES, 
; WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED 
; TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
; PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT, 
; IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR 
; CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
;
;=============================================================================
;   Filename:   p18_tiri.asm
;=============================================================================
;   Author:     Mike Garbutt
;   Company:    Microchip Technology Inc.
;   Revision:   1.00
;   Date:       August 6, 2002
;   Assembled using MPASMWIN V3.20
;=============================================================================
;   Include Files:  p18f452.inc V1.3
;=============================================================================
;   PIC18XXX USART example code for with transmit and receive interrupts.
;   Received data is put into a buffer, called RxBuffer. When a carriage
;   return <CR> is received, the received data in RxBuffer is copied into
;   another buffer, TxBuffer. The data in TxBuffer is then transmitted.
;   Receive uses high priority interrupts, transmit uses low priority.
;=============================================================================
;
        radix   dec             ; Default radix for constants is decimal.
;
;Bit Definitions
;
#define SSIO_TxBufFull  0       ; Tx buffer is full.
#define SSIO_TxBufEmpty 1       ; Tx buffer is empty.
#define SSIO_RxBufFull  2       ; Rx buffer is full.
#define SSIO_RxBufEmpty 3       ; Rx buffer is empty.
#define SSIO_ReceivedCR 4       ; <CR> character received.
#define SSIO_VerifyFlag 7       ; For verification.
;
;*******************************************************************************
;
; SSIO service variables, receive and transmit buffers.
;
SSIO_UdataSec   UDATA       ; Currently this whole data section crammed into one
                            ; 256 RAM bank.  Code ASSUMES this is so, as it only
                            ; changes RAM bank first time any of these vars used.
                            ; Also all pointer arithmetic optimized to modulo
                            ; 256, variables and buffers) must be in one bank.
;
;   NOTE: To allow buffers to cross RAM bank boundary, code needs to look like:
;SSIO_PutByteRxBuffer1
;
; (ADD) movff   FSR0H, SSIO_RxTailPtrH      ;save new EndPointer high byte
;       movff   FSR0L, SSIO_RxTailPtrL      ;save new EndPointer low byte
;
; (ADD) movf    SSIO_RxHeadPtrH, W          ;get start pointer
; (ADD) cpfseq  SSIO_RxTailPtrH             ;and compare with end pointer
; (ADD) bra     SSIO_PutByteRxBuffer2       ;skip low bytes if high bytes not equal
;
;       movf    SSIO_RxHeadPtrL, W          ;get start pointer
;       cpfseq  SSIO_RxTailPtrL             ;and compare with end pointer
;       bra     SSIO_PutByteRxBufferExit
;
;       bsf     SSIO_Flags, SSIO_RxBufFull  ;if same then indicate buffer full
;
#define SSIO_TX_BUF_LEN 0x70    ; Define transmit buffer size.
#define SSIO_RX_BUF_LEN 0x70    ; Define receive buffer size.
;
SSIO_Flags          res     1       ; SSIO flag bits.
SSIO_TempRxData     res     1       ; Temp data in Rx routines. 
SSIO_TempTxData     res     1       ; Temp data in Tx routines. 
SSIO_TxHeadPtrH     res     1       ; Transmit buffer data head pointer (high byte).
SSIO_TxHeadPtrL     res     1       ; Transmit buffer data head pointer (low byte).
SSIO_TxTailPtrH     res     1       ; Transmit buffer data tail pointer (high byte).
SSIO_TxTailPtrL     res     1       ; Transmit buffer data tail pointer (low byte).
SSIO_TxPrevTailPtrL res     1       ; Transmit buffer data PREV tail pointer (low byte).
SSIO_RxHeadPtrH     res     1       ; Receive buffer data head pointer (high byte).
SSIO_RxHeadPtrL     res     1       ; Receive buffer data head pointer (low byte).
SSIO_RxTailPtrH     res     1       ; Receive buffer data tail pointer (high byte).
SSIO_RxTailPtrL     res     1       ; Receive buffer data tail pointer (low byte).
SSIO_RxPrevTailPtrL res     1       ; Receive buffer data PREV tail pointer (low byte).
SSIO_RxCntOERR      res     1       ; Count of Overrun errors.
SSIO_RxCntFERR      res     1       ; Count of Framing errors.
SSIO_RxCntBufOver   res     1       ; Count of RX Buffer Overrun errors.
SSIO_TxCntBufOver   res     1       ; Count of TX Buffer Overrun errors.
SSIO_VarsSpace1     res     .15     ; Reserve space so buffers align on boundaries.
SSIO_TxBuffer       res     SSIO_TX_BUF_LEN     ; Transmit data buffer.
SSIO_RxBuffer       res     SSIO_RX_BUF_LEN     ; Receive data buffer.
;
;*******************************************************************************
;
; SSIO interrupt handling code.
;
SSIO_CodeSec    CODE
;
;*******************************************************************************
;
;   Initialize SSIO internal flags.  Call this before SSIO_InitTxBuffer and SSIO_InitRxBuffer.
;
        GLOBAL  SSIO_InitFlags
SSIO_InitFlags
;
        banksel SSIO_Flags      
        clrf    SSIO_Flags          ; Clear all flags.
        clrf    SSIO_RxCntOERR      ; Clear all error counters.
        clrf    SSIO_RxCntFERR
        clrf    SSIO_RxCntBufOver
        clrf    SSIO_TxCntBufOver
        return
;
;*******************************************************************************
;
;   Initialize transmit buffer.
;
        GLOBAL  SSIO_InitTxBuffer
SSIO_InitTxBuffer
;
        banksel SSIO_TxBuffer      
        movlw   HIGH SSIO_TxBuffer          ; First address of buffer.
        movwf   SSIO_TxHeadPtrH             ; Init transmit head pointer.
        movwf   SSIO_TxTailPtrH             ; Init transmit tail pointer.
;
        movlw   LOW SSIO_TxBuffer           ; First address of buffer.
        movwf   SSIO_TxHeadPtrL             ; Init transmit head pointer.
        movwf   SSIO_TxTailPtrL             ; Init transmit tail pointer.
;
        movlw   LOW (SSIO_TxBuffer+(SSIO_TX_BUF_LEN-1)) ; Last address of buffer.
        movwf   SSIO_TxPrevTailPtrL         ; Init transmit PREV tail pointer.
;
        bcf     SSIO_Flags, SSIO_TxBufFull  ; Tx buffer is not full.
        bsf     SSIO_Flags, SSIO_TxBufEmpty ; Tx buffer is empty.
        return
;
;*******************************************************************************
;
;   Initialize receive buffer.
;
        GLOBAL  SSIO_InitRxBuffer
SSIO_InitRxBuffer
;
        banksel SSIO_RxBuffer      
        movlw   HIGH SSIO_RxBuffer          ; First address of buffer.
        movwf   SSIO_RxHeadPtrH             ; Init receive head pointer.
        movwf   SSIO_RxTailPtrH             ; Init receive tail pointer.
;
        movlw   LOW SSIO_RxBuffer           ; First address of buffer.
        movwf   SSIO_RxHeadPtrL             ; Init receive head pointer.
        movwf   SSIO_RxTailPtrL             ; Init receive tail pointer.
;
        movlw   LOW (SSIO_RxBuffer+(SSIO_RX_BUF_LEN-1)) ; Last address of buffer.
        movwf   SSIO_RxPrevTailPtrL         ; Init receive PREV tail pointer.
;
        bcf     SSIO_Flags, SSIO_RxBufFull  ; Rx buffer is not full.
        bsf     SSIO_Flags, SSIO_RxBufEmpty ; Rx buffer is empty.
        return
;
;*******************************************************************************
;
;Read data from the transmit buffer and put into transmit register.
;
        GLOBAL  SSIO_PutByteIntoTxHW
SSIO_PutByteIntoTxHW
;
        banksel SSIO_Flags
        btfss   SSIO_Flags, SSIO_TxBufEmpty ; Skip if transmit buffer empty.
        bra     SSIO_PutByteIntoTxHW1       ; Else not empty so go transmit.
;
;   NOTE: code unlikely to reach here, but just in case.
;
        bcf     PIE1,TXIE                   ; Empty so disable Tx interrupt.
        bra     SSIO_PutByteIntoTxHW_Exit 
;
SSIO_PutByteIntoTxHW1
        rcall   SSIO_GetByteTxBuffer        ; Get data from non-empty buffer.
        movwf   TXREG                       ; Put data in HW transmit register.
;
SSIO_PutByteIntoTxHW_Exit
        return
;
;*******************************************************************************
;
;   Get received data from USART data register and write it into receive buffer.
;
        GLOBAL  SSIO_GetByteFromRxHW
SSIO_GetByteFromRxHW
;
;   Check for serial errors and handle them if found.
;
        banksel SSIO_Flags
        btfsc   RCSTA, OERR                     ; Skip if no overrun error.
        bra     SSIO_GetByteFromRxHW_ErrOERR    ; Else go handle error.
;
        btfsc   RCSTA, FERR                     ; Skip if no framing error.
        bra     SSIO_GetByteFromRxHW_ErrFERR    ; Else go handle error.
;
        btfsc   SSIO_Flags, SSIO_RxBufFull      ; Skip if no buffer overrun error.
        bra     SSIO_GetByteFromRxHW_ErrRxOver  ; Else go handle error.
;
        bra     SSIO_GetByteFromRxHW_DataGood   ; Otherwise no errors so get good data.
;
;   Overrun error handling.
;
SSIO_GetByteFromRxHW_ErrOERR
        bcf     RCSTA, CREN                     ; Reset the receiver logic.
        bsf     RCSTA, CREN                     ; Enable reception again.
;
        incfsz  SSIO_RxCntOERR, F               ; Increment overrun error count.
        bra     SSIO_GetByteFromRxHW_Exit       ; Overrun error count did not rollover.
;
        decf    SSIO_RxCntOERR, F               ; Max overrun error count.
        bra     SSIO_GetByteFromRxHW_Exit
;
;   Framing error handling.
;
SSIO_GetByteFromRxHW_ErrFERR
        movf    RCREG, W                        ; Discard received data that has error.
;
        incfsz  SSIO_RxCntFERR, F               ; Increment framing error count.
        bra     SSIO_GetByteFromRxHW_Exit       ; Overrun error count did not rollover.
;
        decf    SSIO_RxCntFERR, F               ; Max framing error count.
        bra     SSIO_GetByteFromRxHW_Exit
;
;  Receive buffer overrun error.  Save the data (overwriting last data).
;
SSIO_GetByteFromRxHW_ErrRxOver
        incfsz  SSIO_RxCntBufOver, F            ; Increment overrun error count.
        bra     SSIO_GetByteFromRxHW_DataGood   ; Overrun error count did not rollover.
;
        decf    SSIO_RxCntBufOver, F            ; Max overrun error count.
;
;   Put good or overrun data into receive buffer, schedule task if <CR> found.
;   THIS WILL HAVE TO BE RE-EXAMINED FOR MORE GENERIC WAY OF TRIGGERING USER TASK
;   BECAUSE <CR> IS NOT A UNIVERSAL DEMARCATION OF COMPLETED MESSAGE.
;
SSIO_GetByteFromRxHW_DataGood
        movf    RCREG, W                        ;get received data
        xorlw   0x0d                            ;compare with <CR>      
        btfsc   STATUS, Z                       ;check if the same
        bsf     SSIO_Flags, SSIO_ReceivedCR     ;indicate <CR> character received
;
        xorlw   0x0d                            ;change back to valid data
        rcall   SSIO_PutByteRxBuffer            ;and put in buffer
        banksel SSIO_Flags                      ; In case bank bits changed in subroutine.
;
;    If we find <CR> then schedule user task to process data.  Then interrupt can exit.
;    SRTX Dispatcher will find task scheduled and invoke SUSR_TaskSIO.
;
SSIO_GetByteFromRxHW_CheckCR
        btfss   SSIO_Flags, SSIO_ReceivedCR     ; Skip if <CR> received.
        bra     SSIO_GetByteFromRxHW_Exit       ; <CR> not received so just exit.
;
        bcf     SSIO_Flags, SSIO_ReceivedCR     ; Clear <CR> received flag.
;
;    Schedule user task SUSR_TaskSIO.
;
SSIO_GetByteFromRxHW_SchedTask
        banksel SRTX_Sched_Cnt_TaskSIO
        incfsz  SRTX_Sched_Cnt_TaskSIO, F       ; Increment task schedule count.
        bra     SSIO_GetByteFromRxHW_Exit       ; Task schedule count did not rollover.
;
        decf    SRTX_Sched_Cnt_TaskSIO, F       ; Max task schedule count.
;
SSIO_GetByteFromRxHW_Exit
        return
;
;*******************************************************************************
;
;   Add a byte (in WREG) to tail of transmit buffer.
;
;   NOTE: If we are storing a byte and the buffer is already full, don't store
;   at current tail pointer nor update tail.  Instead store at previous
;   tail pointer.  Continue to overwrite the last location in the buffer.
;
        GLOBAL  SSIO_PutByteTxBuffer
SSIO_PutByteTxBuffer
;
        bcf     PIE1, TXIE                      ; Disable Tx interrupt.
;
        banksel SSIO_Flags      
        btfss   SSIO_Flags, SSIO_TxBufFull      ; Skip if buffer full.
        bra     SSIO_PutByteTxBuffer_StoreByte  ; Else store the byte at old tail ptr.
;
;   Overrun error because storing new data and the buffer is already full.
;
        incfsz  SSIO_TxCntBufOver, F            ; Increment overrun error count.
        bra     SSIO_PutByteTxBuffer_PrevTail   ; Overrun error count did not rollover.
;
        decf    SSIO_TxCntBufOver, F            ; Max overrun error count.
;
SSIO_PutByteTxBuffer_PrevTail
        movff   SSIO_TxTailPtrH, FSR0H          ; Get PREV tail pointer high byte.
        movff   SSIO_TxPrevTailPtrL, FSR0L      ; Get PREV tail pointer low byte.
        movwf   INDF0                           ; Overwrite data at last tail pointer.
        bra     SSIO_PutByteTxBuffer_Exit       ; No pointer or flag handling so just exit.
;
;   Save data at old tail even though buffer may already be full.
;
SSIO_PutByteTxBuffer_StoreByte
        movff   SSIO_TxTailPtrH, FSR0H          ; Get tail pointer high byte.
        movff   SSIO_TxTailPtrL, FSR0L          ; Get tail pointer low byte.
        movff   SSIO_TxTailPtrL, SSIO_TxPrevTailPtrL    ; Save PREV tail pointer.
        movwf   POSTINC0                        ; Copy data, incr probable tail pointer.
;
;   Wrap tail pointer if needed.
;
        movlw   LOW (SSIO_TxBuffer+SSIO_TX_BUF_LEN) ; Find last address +1 of buffer.
        cpfseq  FSR0L                           ; Skip if matches tail pointer.
        bra     SSIO_PutByteTxBuffer1           ; No match, continue.
;
        lfsr    0, SSIO_TxBuffer                ; Reload tail pointer to buffer start.
;
;   Test if buffer is full.
;
SSIO_PutByteTxBuffer1
        movf    FSR0L, W                        ; Get updated probable tail pointer.
        cpfseq  SSIO_TxHeadPtrL                 ; Skip if it will equal head pointer.
        bra     SSIO_PutByteTxBuffer_SaveTail   ; Buffer not full, save new tail pointer.
;
        bsf     SSIO_Flags, SSIO_TxBufFull      ; New tail would == head, now buffer full.
;
SSIO_PutByteTxBuffer_SaveTail
        movff   FSR0L, SSIO_TxTailPtrL          ; Save new tail.
;
SSIO_PutByteTxBuffer_Exit
        bcf     SSIO_Flags, SSIO_TxBufEmpty     ; Flag buffer not empty.
;
;        btfss   SSIO_Flags, SSIO_VerifyFlag     ; Skip if verifying.
        bsf     PIE1, TXIE                      ; Re-enable transmit interrupt.
        return
;
;*******************************************************************************
;
;   Add a byte (in WREG) to tail of receive buffer.
;   
;   NOTE: This routine is time critical.  It barely runs in the time allocated
;   for a 4 MHz clock at 115.2K baud.  It runs fine at 57.6K baud.  Consideration
;   should be given to making this a high priority interrupt, so fast receives
;   can be accomodated.
;
;   NOTE: If we are storing a byte and the buffer is already full, don't store
;   at current tail pointer nor update tail.  Instead store at previous
;   tail pointer.  Continue to overwrite the last location in the buffer.
;   So when <CR> is received and stored, other routines will find it.
;
        GLOBAL  SSIO_PutByteRxBuffer
SSIO_PutByteRxBuffer
;
; NOTE: no disabling/re-enabling of RX interrupt because this routine called from RX ISR.
;
        banksel SSIO_Flags      
        btfss   SSIO_Flags, SSIO_RxBufFull      ; Skip if buffer full.
        bra     SSIO_PutByteRxBuffer_StoreByte  ; Else just store the byte at current tail.
;
;   Overrun error because storing new data and the buffer is already full.
;
        incfsz  SSIO_RxCntBufOver, F            ; Increment overrun error count.
        bra     SSIO_PutByteRxBuffer_LastTail   ; Overrun error count did not rollover.
;
        decf    SSIO_RxCntBufOver, F            ; Max overrun error count.
;
SSIO_PutByteRxBuffer_LastTail
        movff   SSIO_RxTailPtrH, FSR0H          ; Get PREV tail pointer high byte.
        movff   SSIO_RxPrevTailPtrL, FSR0L      ; Get PREV tail pointer low byte.
        movwf   INDF0                           ; Overwrite data at last tail pointer.
        bra     SSIO_PutByteRxBuffer_Exit       ; No pointer or flag handling so just exit.
;
;   Save data at tail.
;
SSIO_PutByteRxBuffer_StoreByte
        movff   SSIO_RxTailPtrH, FSR0H          ; Get tail pointer high byte.
        movff   SSIO_RxTailPtrL, FSR0L          ; Get tail pointer low byte.
        movff   SSIO_RxTailPtrL, SSIO_RxPrevTailPtrL    ; Save PREV tail pointer.
        movwf   POSTINC0                        ; Copy data, incr tail pointer.
;
;   Wrap tail pointer if needed.
;
        movlw   LOW (SSIO_RxBuffer+SSIO_RX_BUF_LEN) ; Last address +1 of buffer.
        cpfseq  FSR0L                           ; Skip if matches tail pointer.
        bra     SSIO_PutByteRxBuffer1           ; No match, continue.
;
        lfsr    0, SSIO_RxBuffer                ; Reload tail pointer to buffer start.
;
;   Test if buffer is full.
;
SSIO_PutByteRxBuffer1
        movf    FSR0L, W                        ; Get updated tail pointer.
        cpfseq  SSIO_RxHeadPtrL                 ; Skip if it now equals head pointer.
        bra     SSIO_PutByteRxBuffer_SaveTail   ; Buffer not full, save new tail pointer.
;
        bsf     SSIO_Flags, SSIO_RxBufFull      ; New tail == head, now buffer full.
;
SSIO_PutByteRxBuffer_SaveTail
        movff   FSR0L, SSIO_RxTailPtrL          ; Save new tail.
;
SSIO_PutByteRxBuffer_Exit
        bcf     SSIO_Flags, SSIO_RxBufEmpty     ; Flag buffer not empty.
        return
;
;*******************************************************************************
;
;   Remove and return (in WREG) the byte at head of transmit buffer.
;
        GLOBAL  SSIO_GetByteTxBuffer
SSIO_GetByteTxBuffer
;   
        banksel SSIO_Flags
        btfsc   SSIO_Flags, SSIO_TxBufEmpty     ; Skip if buffer not empty.
        bra     SSIO_GetByteTxBuffer_BufEmpty   ; Else buffer empty, just leave.
;
;   Get data at old head.
;
        movff   SSIO_TxHeadPtrH, FSR0H          ; Get head pointer high byte.
        movff   SSIO_TxHeadPtrL, FSR0L          ; Get head pointer low byte.
        movff   POSTINC0, SSIO_TempTxData       ; Copy data, incr head pointer.
;
;   Wrap head pointer if needed.
;
        movlw   LOW (SSIO_TxBuffer+SSIO_TX_BUF_LEN) ; Last address +1 of buffer.
        cpfseq  FSR0L                           ; Skip if matches head pointer.
        bra     SSIO_GetByteTxBuffer1           ; No match, continue.
;
        lfsr    0, SSIO_TxBuffer                ; Reload head pointer buffer start.
;
SSIO_GetByteTxBuffer1
        movff   FSR0L, SSIO_TxHeadPtrL          ; Save new head pointer.
;
;   Test if buffer is empty.
;
        movf    FSR0L, W                        ; Get updated head pointer.
        cpfseq  SSIO_TxTailPtrL                 ; Skip if it equals tail pointer.
        bra     SSIO_GetByteTxBuffer2           ; Buffer not empty.
;
        bsf     SSIO_Flags, SSIO_TxBufEmpty     ; New head == tail, flag buffer empty.
        bcf     PIE1,TXIE                       ; Empty so disable Tx interrupt.
;
SSIO_GetByteTxBuffer2
        bcf     SSIO_Flags, SSIO_TxBufFull      ; Since removed byte buffer cannot be full.
        movf    SSIO_TempTxData, W              ; Return data from buffer.
        return
;
;   Nothing to do if trying to read from empty buffer.
;   This is not necessarily an error condition.
;
SSIO_GetByteTxBuffer_BufEmpty
;
;   NOTE: code unlikely to reach here, but just in case.
;
        retlw   0                               ; Buffer empty, return zero value.
;
;*******************************************************************************
;
;Remove and return (in WREG) the byte at head of receive buffer.
;
        GLOBAL  SSIO_GetByteRxBuffer
SSIO_GetByteRxBuffer
;
        bcf     PIE1, RCIE                      ; Disable Rx interrupt.
        banksel SSIO_Flags
        btfsc   SSIO_Flags, SSIO_RxBufEmpty     ; Skip if buffer not empty.
        bra     SSIO_GetByteRxBuffer_BufEmpty   ; Else buffer empty, just leave.
;
;   Get data at head.
;
        movff   SSIO_RxHeadPtrH, FSR0H          ; Get head pointer high byte.
        movff   SSIO_RxHeadPtrL, FSR0L          ; Get head pointer low byte.
        movff   POSTINC0, SSIO_TempRxData       ; Copy data, incr head pointer.
;
;   Wrap head pointer if needed.
;
        movlw   LOW (SSIO_RxBuffer+SSIO_RX_BUF_LEN) ; Last address +1 of buffer.
        cpfseq  FSR0L                           ; Skip if matches head pointer.
        bra     SSIO_GetByteRxBuffer1           ; No match, continue.
;
        lfsr    0, SSIO_RxBuffer                ; Reload head pointer buffer start.
;
SSIO_GetByteRxBuffer1
        movff   FSR0L, SSIO_RxHeadPtrL          ; Save new head pointer.
;
;   Test if buffer is empty.
;
        movf    FSR0L, W                        ; Get updated head pointer.
        cpfseq  SSIO_RxTailPtrL                 ; Skip if it equals tail pointer.
        bra     SSIO_GetByteRxBuffer2           ; Buffer not empty.
;
        bsf     SSIO_Flags, SSIO_RxBufEmpty     ; New head == tail, flag buffer empty.
;
SSIO_GetByteRxBuffer2
        bcf     SSIO_Flags, SSIO_RxBufFull      ; Since removed byte buffer cannot be full.
        movf    SSIO_TempRxData, W              ; Return data from buffer.
        bsf     PIE1, RCIE                      ; Re-enable Rx interrupt.
        return
;
;   Nothing to do if trying to read from empty buffer.
;   This is not necessarily an error condition.
;
SSIO_GetByteRxBuffer_BufEmpty
        bsf     PIE1, RCIE                      ; Re-enable Rx interrupt.
        retlw   0                               ; Buffer empty, return zero value.
        end