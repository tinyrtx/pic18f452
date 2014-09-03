        LIST
;*******************************************************************************
; tinyRTX Filename: sisd.asm (System Interrupt Service Director)
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
;   17Oct03 SHiggins@tinyRTX.com 	Created from scratch.
;   23Jul14 SHiggins@tinyRTX.com 	Move save/restore FSR from SISD_Director to 
;									to SISD_Interrupt
;   14Aug14 SHiggins@tinyRTX.com    Converted from PIC16877 to PIC18F452.
;   02Sep14 SHiggins@tinyRTX.com    Save/restore BSR.
;   								Remove AD_COMPLETE_TASK and I2C_COMPLETE_TASK options.
;									Both now have some interrupt handling and a task.
;	03Sep14 SHiggins@tinyRTX.com    SISD_Director_CheckI2C now calls SUSR_ISR_I2C.
;
;*******************************************************************************
;
        errorlevel -302	
	    #include    <p18f452.inc>
	    #include    <srtx.inc>
	    #include    <srtxuser.inc>
        #include 	<strc.inc>
	    #include    <susr.inc>
;
;*******************************************************************************
;
; SISD service variables.
;
; Interrupt Service Routine context save/restore variables.
;
; LINKNOTE: SISD_UdataShrSec must be placed in data space shared across all banks.
;			This is because to save/restore STATUS register properly, we can't control
;			RP1 and RP0 bits.  So any values in RP1 and RP0 must be valid.  In order to
;			allow this we need memory which accesses the same across all banks.
;
SISD_UdataShrSec    UDATA_ACS
;
SISD_TempW          res     1   ; Access Bank; temp copy of W.
SISD_TempSTATUS     res     1   ; Access Bank; temp copy of STATUS.
SISD_TempBSR		res     1   ; Access Bank; temp copy of BSR.
SISD_TempPCLATH     res     1   ; Access Bank; temp copy of PCLATH.
SISD_TempPCLATU     res     1   ; Access Bank; temp copy of PCLATU.
SISD_TempFSR0H      res     1   ; Access Bank; temp copy of FSR.
SISD_TempFSR0L      res     1   ; Access Bank; temp copy of FSR.
;
;*******************************************************************************
;
SISD_ResetVectorSec	CODE            ; Reset vector address.
SISD_ResetEntry
        goto    SRTX_Init           ; Initialize SRTX and then application.
;
SISD_IntVectorSec   CODE            ; Interrupt vector address.
SISD_InterruptEntry
		goto	SISD_Interrupt		; Handle interrupt.
									; This is small because if we ever decide to
									; use prioritized interrupts, that handler
									; is placed immediately after this one.
;
SISD_IntCodeSec     CODE            ; Interrupt handler, too big to fit at vector address.
SISD_Interrupt
        movwf   SISD_TempW          	; Access RAM; preserve W without changing STATUS.
		movff	STATUS, SISD_TempSTATUS	; Access RAM; preserve STATUS.
		movff	BSR, SISD_TempBSR		; Access RAM; preserve BSR.
		movff	PCLATH, SISD_TempPCLATH	; Access RAM; preserve PCLATH.
		movff	PCLATU, SISD_TempPCLATU	; Access RAM; preserve PCLATU.
		movff	FSR0L, SISD_TempFSR0L	; Access RAM; preserve FSR0L.
		movff	FSR0H, SISD_TempFSR0H	; Access RAM; preserve FSR0H.
;
; Now we can use the internal registers (W, STATUS, PCLATH and FSR).
; GOTO is used here to save stack space, and SISD_Director never called elsewhere.
;
        bra     SISD_Director       ; Service interrupt director.
;
; SISD_Director does a GOTO here when it completes.
;
SISD_InterruptExit
;
; Now we restore the internal registers (W, STATUS, PCLATH and FSR), taking special care not to disturb
; any of them.
;
		movff	SISD_TempFSR0H, FSR0H 	; Access RAM; restore FSR0H.
		movff	SISD_TempFSR0L, FSR0L 	; Access RAM; restore FSR0L.
		movff	SISD_TempPCLATU, PCLATU	; Access RAM; restore PCLATU.
		movff	SISD_TempPCLATH, PCLATH	; Access RAM; restore PCLATH.
		movff	SISD_TempBSR, BSR		; Access RAM; restore BSR.
		movff	SISD_TempSTATUS, STATUS	; Access RAM; restore STATUS.
        swapf   SISD_TempW, F       	; Access RAM; restore W without changing STATUS.
        swapf   SISD_TempW, W
;
       	retfie                      	; Return from interrupt exception. (Return plus GIE.)
;
;*******************************************************************************
;
SISD_CodeSec	CODE
;
;*******************************************************************************
;
; SISD: System Interrupt Service Director.
;
; 3 possible sources of interrupts:
;   a) Timer1 expires. (Initiate A/D conversion, schedule new timer int in 100ms.)
;   b) A/D conversion completed. (Convert reading to ASCII.)
;   c) I2C event completed. (Multiple I2C events to transmit ASCII.)
;
;*******************************************************************************
;
; Each routine invoked by this routine must conclude with a MANDATORY return statement.
; This allows us to save a stack slot by using GOTO's here and still operate correctly.
;
SISD_Director
;
; Test for Timer1 rollover.
;
        banksel PIR1
        btfss   PIR1, TMR1IF            ; Skip if Timer1 interrupt flag set.
        bra     SISD_Director_CheckADC  ; Timer1 int flag not set, check other ints.
        bcf     PIR1, TMR1IF            ; Clear Timer1 interrupt flag.
        call    SUSR_Timebase           ; User re-init of timebase interrupt.
        call    SRTX_Scheduler          ; SRTX scheduler when timebase interupt, must RETURN at end.
        bra     SISD_Director_Exit      ; Only execute single interrupt handler.
;
; Test for completion of A/D conversion.
;
SISD_Director_CheckADC
        btfss   PIR1, ADIF              ; Skip if A/D interrupt flag set.
        bra     SISD_Director_CheckI2C  ; A/D int flag not set, check other ints.
        bcf     PIR1, ADIF              ; Clear A/D interrupt flag.
        banksel PIE1
        bcf     PIE1, ADIE              ; Disable A/D interrupts.
        banksel SRTX_Sched_Cnt_TaskADC
        incfsz  SRTX_Sched_Cnt_TaskADC, F   ; Increment task schedule count.
        bra     SISD_Director_Exit          ; Task schedule count did not rollover.
        decf    SRTX_Sched_Cnt_TaskADC, F   ; Max task schedule count.
        bra     SISD_Director_Exit          ; Only execute single interrupt handler.
;
; Test for completion of I2C event.
;
SISD_Director_CheckI2C
        btfss   PIR1, SSPIF             ; Skip if I2C interrupt flag set.
        bra     SISD_Director_Exit      ; I2C int flag not set, check other ints.
        bcf     PIR1, SSPIF             ; Clear I2C interrupt flag.
        call    SUSR_ISR_I2C            ; User ISR handling when I2C event.
        bra     SISD_Director_Exit      ; Only execute single interrupt handler.
;
; This point only reached if unknown interrupt occurs, any error handling can go here.
;
SISD_Director_Exit
        bra     SISD_InterruptExit      ; Return to SISD interrupt handler.
;
        end