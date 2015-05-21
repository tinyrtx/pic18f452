        LIST
;*******************************************************************************
; tinyRTX Filename: uadc.asm (User Analog to Digital Conversion routines)
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
;   27Jan04 SHiggins@tinyRTX.com  Split out from main user application.  For PICdem2plus demo board.
;   30Jan04 SHiggins@tinyRTX.com  Updated to use 1023 steps to full range, not 1024.
;   29Jul14 SHiggins@tinyRTX.com  Move UADC_SetupDelay to inline in UADC_Trigger, reducing stack.
;   13Aug14 SHiggins@tinyRTX.com  Converted from PIC16877 to PIC18F452.
;   18Aug14 SHiggins@tinyRTX.com  Minor optimizations.
;   21Aug14 SHiggins@tinyRTX.com  Replaced FXM1616U with SM16_16x16u.
;   26Apr15 Stephen_Higgins@KairosAutonomi.com
;               Added support for PIC18F2620, removed UADC_ADCON0_Temp.
;               Removed unnecessary banksel's for SFR's in access RAM.
;   06May15 Stephen_Higgins@KairosAutonomi.com
;               Support UCFG_PROC == UCFG_18F2620.
;
;*******************************************************************************
;
        errorlevel -302 
;
        #include    <ucfg.inc>  ; Configure board and proc, #include <proc.inc>
        #include    <sm16.inc>
        #include    <sbcd.inc>
        #include    <ulcd.inc>
;
;*******************************************************************************
;
; User ADC defines.
;
    IF UCFG_PROC==UCFG_18F452
;
;   18F452 specified.
;   *****************
;
#define UADC_ADCON0_VAL  0x01
;
; A/D conversion clock is 4MHz/2; A/D channel = 0; no conversion active; A/D on.
;
; bit 7 : ADCS1 : 0 : Clock Conversion Select (with ADCON1/ADCS2 = 0, Fosc/2)
; bit 6 : ADCS0 : 0 : Clock Conversion Select (with ADCON1/ADCS2 = 0, Fosc/2)
; bit 5 : CHS2  : 0 : Channel Select, 0b000 -> AN0
; bit 4 : CHS1  : 0 : Channel Select, 0b000 -> AN0
; bit 3 : CHS0  : 0 : Channel Select, 0b000 -> AN0
; bit 2 : GO    : 0 : A/D Conversion Status
; bit 1 : dc    : 0 : Unimplemented, read as 0
; bit 0 : ADON  : 1 : A/D converter module is powered up
;
#define UADC_ADCON1_VAL  0x8e
;
; Vdd on Vref+; AN0 is analog, AN7-AN1 are discretes.
;
; bit 7 : ADFM  : 1 : Right-justified 10-bit A/D result
; bit 6 : ADCS2 : 0 : Clock Conversion Select (with ADCON1/ADCS2:ADCS1 = 00, Fosc/2)
; bit 5 : dc    : 0 : Unimplemented, read as 0
; bit 4 : dc    : 0 : Unimplemented, read as 0
; bit 3 : PCFG3 : 1 : A/D Port Configuration Control, 0b1110 -> AN0 analog, AN7-AN1 discretes
; bit 2 : PCFG2 : 1 : A/D Port Configuration Control, 0b1110 -> AN0 analog, AN7-AN1 discretes
; bit 1 : PCFG1 : 1 : A/D Port Configuration Control, 0b1110 -> AN0 analog, AN7-AN1 discretes
; bit 0 : PCFG0 : 0 : A/D Port Configuration Control, 0b1110 -> AN0 analog, AN7-AN1 discretes
;
    ENDIF
;
    IF UCFG_PROC==UCFG_18F2620
;
;   18F2620 specified.
;   ******************
;
#define UADC_ADCON0_VAL  0x01
;
; A/D channel = 0; no conversion active; A/D on.
;
; bit 7 : dc    : 0 : Unimplemented, read as 0
; bit 6 : dc    : 0 : Unimplemented, read as 0
; bit 5 : CHS3  : 0 : Channel Select, 0b0000 -> AN0
; bit 4 : CHS2  : 0 : Channel Select, 0b0000 -> AN0
; bit 3 : CHS1  : 0 : Channel Select, 0b0000 -> AN0
; bit 2 : CHS0  : 0 : Channel Select, 0b0000 -> AN0
; bit 1 : GO    : 0 : A/D Conversion Status
; bit 0 : ADON  : 1 : A/D converter module is powered up
;
#define UADC_ADCON1_VAL  0x0e
;
; Vdd on Vref+, Vss on Vref- 
; AN0 is analog, AN12-AN8 and AN4-AN1 are discretes. (No AN7-AN5 on 28-pin chips.)
;
; bit 7 : dc    : 0 : Unimplemented, read as 0
; bit 6 : dc    : 0 : Unimplemented, read as 0
; bit 5 : VCFG1 : 0 : Voltage Reference Configuration, Vss = Vref-
; bit 4 : VCFG2 : 0 : Voltage Reference Configuration, Vdd = Vref+
; bit 3 : PCFG3 : 1 : A/D Port Configuration Control, 0b1110 -> AN0 analog, AN12-AN1 discretes
; bit 2 : PCFG2 : 1 : A/D Port Configuration Control, 0b1110 -> AN0 analog, AN12-AN1 discretes
; bit 1 : PCFG1 : 1 : A/D Port Configuration Control, 0b1110 -> AN0 analog, AN12-AN1 discretes
; bit 0 : PCFG0 : 0 : A/D Port Configuration Control, 0b1110 -> AN0 analog, AN12-AN1 discretes
;
#define UADC_ADCON2_VAL  0x84
;
; Right-justified result; Tad = 4 Tosc (need at Tad > .7us, so 4 Tosc = 1.0us at Tosc = 250ns)
; A/D acquisition is manual (could be set to 20 Tad, which is 20us). 
;
; bit 7 : ADFM  : 1 : 10-bit result is right-justified
; bit 6 : dc    : 0 : Unimplemented, read as 0
; bit 5 : ACQT2 : 0 : A/D Acquisition Time, 0b000 -> manual aquisition
; bit 4 : ACQT1 : 0 : A/D Acquisition Time, 0b000 -> manual aquisition
; bit 3 : ACQT0 : 0 : A/D Acquisition Time, 0b000 -> manual aquisition
; bit 6 : ADCS2 : 1 : Clock Conversion Select, 0b100 -> Tad = 4 Tosc (Fosc/4)
; bit 7 : ADCS1 : 0 : Clock Conversion Select, 0b100 -> Tad = 4 Tosc (Fosc/4)
; bit 6 : ADCS0 : 0 : Clock Conversion Select, 0b100 -> Tad = 4 Tosc (Fosc/4)
;
    ENDIF
;
;*******************************************************************************
;
; User ADC service variables.
;
UADC_UdataSec       UDATA
;
UADC_DelayTimer     res     1   ; Countdown delay timer.
UADC_Channel        res     1   ; A/D channel select.
UADC_ResultRawHi    res     1   ; Raw A/D result, high byte.
UADC_ResultRawLo    res     1   ; Raw A/D result, low byte.
UADC_ResultScaledHi res     1   ; Scaled A/D result, high byte.
UADC_ResultScaledLo res     1   ; Scaled A/D result, low byte.
;
;*******************************************************************************
;
; Init ADC registers.
;
UADC_CodeSec        CODE
;
        GLOBAL  UADC_Init
UADC_Init
;
; Ensure RA0 set as input to allow it to function as analog input.
;
        bsf     TRISA, 0    ; TRISA bit 0 set to allow AN0 to function as analog input.
;
; Program ADC registers per selected processor.
;
        movlw   UADC_ADCON0_VAL
        movwf   ADCON0
        movlw   UADC_ADCON1_VAL
        movwf   ADCON1
;
    IF UCFG_PROC==UCFG_18F2620
        movlw   UADC_ADCON2_VAL
        movwf   ADCON2
    ENDIF
;
        return
;
;*******************************************************************************
;
        GLOBAL  UADC_Trigger
UADC_Trigger
;
; Delay for A/D acquisition time.
; (STRICTLY SPEAKING, THIS IS ONLY NEEDED IF CHANGING ACTIVE A/D PORT SELECTION.)
;
; Delay 20us for A/D sample setup.  Assumes 4MHz clock.
; (7*3) = 21 cycles * (1us/cycle) = 21 us.
;
        movlw   D'6'                    ; Init uses 3 cycles.
        banksel UADC_DelayTimer
        movwf   UADC_DelayTimer         ; Delay timer.
;
UADC_SetupDelay_Loop                    ; Loop uses 3 cycles each iteration.
        decfsz  UADC_DelayTimer, F      ; Delay timer.
        bra     UADC_SetupDelay_Loop    ; Loop until timer expires.
;
; Trigger A/D conversion.
;
        bsf     PIE1, ADIE              ; Enable A/D interrupts. (BEFORE setting GO flag!)
        bsf     ADCON0, GO              ; Trigger A/D conversion.
        return
;
;*******************************************************************************
;
        GLOBAL  UADC_RawToASCII
UADC_RawToASCII
;
; Save completed A/D result bytes.
;
        movf    ADRESH, W           ; Get A/D result high byte.
        banksel UADC_ResultRawHi    ; Bank 0.
        movwf   UADC_ResultRawHi    ; Bank 0, high byte of A/D result.
;
        movf    ADRESL, W           ; Get A/D result low byte.
        movwf   UADC_ResultRawLo    ; Bank 0, low byte of A/D result.
;
; AD_ResultRawHi:AD_ResultRawLo contains right-justified 10-bit result, upper 6 bits are 0.
; 0x0000 = 0.00 Vdc, 0x03ff = 5.00 Vdc.
; Zeros in bit 15 through bit 10; result msb = bit 9, lsb = bit 0.
;
; Convert raw A/D to engineering units.
;
; Assuming 1023 counts = 0x3ff = 5 volts.
; N(computer counts) = E(volts) * (1023 counts/5.0 volts) so N = E * 204.6
; (With N = E * 204.6, E = N/204.6, or E(volts) = N * 0.00489 (each N is 4.89mV) )
;
; For display purposes, we desire E = N * .001 (each N is 1.0 mV), or N = E * 1000.
; Therefore we rescale from N = E * 204.6 to N = E * 1000, so mult by 1000/204.6 (or 4.89).
; Since we cannot do integer multiply by 4.89 directly, we use an extra 8 bits of precision
; in the multiplier and then discard the extra 8 bits in the result.
;
; Begin with N = E * 204.6.
; Multiply by ((1000/204.6)*256) = 1251, now N = E * 1000 * 256.
; Drop lower 8 bits of result is like dividing by 256, now N = E * 1000.
;
        movff   UADC_ResultRawHi, AARGB0    ; AARGB0-B1 = raw A/D result.
        movff   UADC_ResultRawLo, AARGB1
;
        movlw   0x04                    ; BARGB0-B1 = 1251.
        banksel BARGB0
        movwf   BARGB0
        movlw   0xe3
        movwf   BARGB1
;
        call    SM16_16x16u             ; 16 x 16 unsigned multiply, AARG <- AARG x BARG.
                                        ; AARGB0-AARGB3 = raw A/D * 1251.
                                        ; To divide by 256 we ignore AARGB3 result byte.
;
        movff   AARGB1, UADC_ResultScaledHi ; AARGB1-B2 = scaled A/D result.
        movff   AARGB2, UADC_ResultScaledLo 
;
; Convert from engineering units to BCD.
;
        movff   UADC_ResultScaledHi, BARGB0 ; BARGB0-B1 = scaled A/D result.
        movff   UADC_ResultScaledLo, BARGB1
;
        call    e2bcd16u                ; 16 bit unsigned engineering unit to BCD conversion.
                                        ; AARGB0-B2 = scaled A/D result in BCD.
                                        ; Result is 5 nibbles, right-justified, high nibble is 0.
;
; Convert from 5-digit BCD to 6 ASCII chars, add decimal point, 3 decimal places.
;
        movff   AARGB0, BARGB0          ; BARGB0-B2 = scaled A/D result in BCD.
        movff   AARGB1, BARGB1          
        movff   AARGB2, BARGB2          
;
        call    bcd2a5p3                ; AARGB0-B5 = scaled A/D result in ASCII with dec pt.
;
; AARGB0 (leading zero) is ignored as we know that range is from 00.000 to 04.999.                                       
;
        lfsr    0, ULCD_VoltAscii0  ; Indirect pointer gets dest start address.
        movff   AARGB1, POSTINC0    ; Char 0 (volts ones) to dest ASCII buffer.
        movff   AARGB2, POSTINC0    ; Char 1 (volts decimal point) to dest ASCII buffer.
        movff   AARGB3, POSTINC0    ; Char 2 (volts tenths) to dest ASCII buffer.
        movff   AARGB4, POSTINC0    ; Char 3 (volts hundredths) to dest ASCII buffer.
        movff   AARGB5, POSTINC0    ; Char 4 (volts thousandths) to dest ASCII buffer.
;
        return
        end