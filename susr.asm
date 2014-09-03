        TITLE "SUSR - SRTX to User Application interface"
;
;*******************************************************************************
; tinyRTX Filename: susr.asm (System USeR interface)
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
; Revision History:
;  31Oct03  SHiggins@tinyRTX.com  Created to isolate user vars and routine calls.
;  24Feb04  SHiggins@tinyRTX.com  Add trace.
;  29Jul14  SHiggins@tinyRTX.com  Moved UAPP_Timer1Init to MACRO to save stack.
;  30Jul14  SHiggins@tinyRTX.com  Reduce from 4 tasks to 3 to reduce stack needs.
;  12Aug14  SHiggins@tinyRTX.com  Converted from PIC16877 to PIC18F452.
;  02Sep14	SHiggins@tinyRTX.com	Removed smTrace calls in SUSR_TaskI2C.
;									Although smTrace has int protection, it does not
;									seem to be sufficient, and
;									calling smTrace in an ISR such as TaskI2C clobbers
;									both the trace buffer and RAM.  Unresolved.
;  03Sep14  SHiggins@tinyRTX.com	Rename SUSR_TaskI2C to SUSR_ISR_I2C, remove SUSR_UdataSec.
;                                   New SUSR_TaskI2C now invokes UI2C_MsgTC74ProcessData.
;
;*******************************************************************************
;
        errorlevel -302	
        #include <p18f452.inc>
        #include <slcd.inc>
        #include <si2c.inc>
        #include <strc.inc>
        #include <uapp.inc>
        #include <ulcd.inc>
        #include <uadc.inc>
        #include <ui2c.inc>
;
;*******************************************************************************
;
; SUSR: System to User Redirection.
;
; These routines provide the interface from the SRTX (System Real Time eXecutive)
;   and SISD (Interrupt Service Routine Director) to user application code.
;
SUSR_CodeSec    CODE
;
; User initialization at Power-On Reset Phase A.
;  Application time-critical initialization, no interrupts.
;
        GLOBAL  SUSR_POR_PhaseA
SUSR_POR_PhaseA
        nop
        return
;
; User initialization at Power-On Reset Phase B.
;  Application non-time critical initialization.
;
        GLOBAL  SUSR_POR_PhaseB
SUSR_POR_PhaseB
        call    UAPP_POR_Init           ; User app POR Init. (Enables global interrupts.)
;
;  UAPP_POR_Init enabled global interrupts.
;
        call    UADC_Init               ; User ADC hardware init.
        call    SLCD_Init               ; System LCD init.
        call    ULCD_Init               ; User LCD init.
        call    UI2C_Init               ; User I2C hardware init.
        return
;
; User initialization of timebase timer and corresponding interrupt.
;
        GLOBAL  SUSR_Timebase
SUSR_Timebase
		UAPP_Timer1Init         ; Re-init Timer1 so new int in 100ms. (Enables Timer1 interrupt.)
;
; UAPP_Timer1Init enabled Timer1 interrupts.
;
        return
;
; User interface to Task1.
;
        GLOBAL  SUSR_Task1
SUSR_Task1
        smTraceL STRC_TSK_BEG_1
        banksel PORTB
        movlw   0x01
        xorwf   PORTB, F                ; Toggle LED 1.
        call    UADC_Trigger            ; Initiate new A/D conversion. (Enables ADC interrupt.)
;
; UADC_Trigger enabled ADC interrupts.
;
        smTraceL STRC_TSK_END_1
        return
;
; User interface to Task2.
;
        GLOBAL  SUSR_Task2
SUSR_Task2
        smTraceL STRC_TSK_BEG_2
        banksel PORTB
        movlw   0x02
        xorwf   PORTB, F                ; Toggle LED 2.
        call    ULCD_RefreshLine1       ; Update LCD data buffer with scrolling message.
        call    SLCD_RefreshLine1       ; Send LCD data buffer to LCD.
        smTraceL STRC_TSK_END_2
        return
;
; User interface to Task3.
;
        GLOBAL  SUSR_Task3
SUSR_Task3
        smTraceL STRC_TSK_BEG_3
        banksel PORTB
        movlw   0x04
        xorwf   PORTB, F                ; Toggle LED 3.
        call    UI2C_MsgTC74			; Use I2C to get raw temperature from TC74.
        smTraceL STRC_TSK_END_3
        return
;
; User interface to TaskADC.
;
        GLOBAL  SUSR_TaskADC
SUSR_TaskADC
        smTraceL STRC_TSK_BEG_ADC
        call    UADC_RawToASCII         ; Convert A/D result to ASCII msg for display.
        call    ULCD_RefreshLine2       ; Update LCD data buffer with A/D and temperature.
        call    SLCD_RefreshLine2       ; Send LCD data buffer to LCD.
        smTraceL STRC_TSK_END_ADC
        return
;
; User handling when I2C event interrupt occurs.
;
        GLOBAL  SUSR_ISR_I2C
SUSR_ISR_I2C
;;        smTraceL STRC_ISR_BEG_I2C     ; Calls to smTrace during ints currently not supported.
        call    SI2C_Tbl_HwState        ; Service I2C event.
;;        smTraceL STRC_ISR_END_I2C     ; Calls to smTrace during ints currently not supported.
        return
;
; User interface to TaskI2C.
;
        GLOBAL  SUSR_TaskI2C
SUSR_TaskI2C
        smTraceL STRC_TSK_BEG_I2C
        call    UI2C_MsgTC74ProcessData	; Process data from TC74 message.
        smTraceL STRC_TSK_END_I2C
        return
;
; User handling when I2C message completed.
;
        GLOBAL  SUSR_TaskI2C_MsgDone
SUSR_TaskI2C_MsgDone
        goto    UI2C_MsgDone
;
        end