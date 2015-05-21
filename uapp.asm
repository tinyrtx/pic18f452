        LIST
;*******************************************************************************
; tinyRTX Filename: uapp.asm (User APPlication)
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
;  15Oct03  SHiggins@tinyRTX.com  Created from scratch.
;  31Oct03  SHiggins@tinyRTX.com  Split out USER interface calls.
;  27Jan04  SHiggins@tinyRTX.com  Split out UADC, updated comments.
;  30Jan04  SHiggins@tinyRTX.com  Refined initialization.
;  29Jul14  SHiggins@tinyRTX.com  Moved UAPP_Timer1Init to MACRO to save stack.
;  13Aug14  SHiggins@tinyRTX.com  Converted from PIC16877 to PIC18F452.
;  14Apr15  Stephen_Higgins@KairosAutonomi.com
;               Converted from PIC18F452 to PIC18F2620.
;  29Apr15  Stephen_Higgins@KairosAutonomi.com
;               Added support for 2010 PICDEM2+ demo board (no 4 Mhz crystal).
;  05May15  Stephen_Higgins@KairosAutonomi.com
;               Added support for Kairos Autonomi 280B board.
;  14May15  Stephen_Higgins@KairosAutonomi.com  
;               Substitute #include <ucfg.inc> for <p18f452.inc>.
;  20May15  Stephen_Higgins@KairosAutonomi.com  
;               Fix UAPP_Timer1Init by adding terminating return.
;
;*******************************************************************************
;
; Complete PIC18F452 (40-pin device) pin assignments for PICDEM 2 Plus (2002) Demo Board
;   OR PICDEM 2 Plus (2010) Demo Board:
;
;  1) MCLR*/Vpp         = Reset/Programming connector(1): (active low, with debounce H/W)
;  2) RA0/AN0           = Analog In: Potentiometer Voltage 
;  3) RA1/AN1           = Discrete Out: LCD E (SLCD_CTRL_E)
;  4) RA2/AN2/Vref-     = Discrete Out: LCD RW (SLCD_CTRL_RW)
;  5) RA3/AN3/Vref+     = Discrete Out: LCD RS (SLCD_CTRL_RS)
;  6) RA4/TOCKI         = Discrete In:  Pushbutton S2 (active low, no debounce H/W)
;  7) RA5/AN4/SS*       = No Connect: (configured as Discrete In)
;  8) RE0/RD*/AN5       = No Connect: (configured as Discrete In)
;  9) RE1/WR*/AN6       = No Connect: (configured as Discrete In)
; 10) RE2/CS*/AN7       = No Connect: (configured as Discrete In)
; 11) Vdd               = Programming connector(2) (+5 VDC) 
; 12) Vss               = Programming connector(3) (Ground) 
;
;   External 4 Mhz crystal installed in Y2, PICDEM 2 Plus (2002).
;
; 13) OSC1/CLKIN        = 4 MHz clock in (4 MHz/4 = 1 MHz = 1us instr cycle)
;
;   External 4 Mhz crystal NOT installed in Y2, PICDEM 2 Plus (2010) PN 02-01630-1.
;   Note that Jumper J7 should not be connected.
;
; 13) OSC1/CLKIN        = No Connect.
;
; 14) OSC2/CLKOUT       = (non-configurable output)
; 15) RC0/T1OSO/T1CKI   = No Connect: (configured as Discrete In) (possible future Timer 1 OSO)
; 16) RC1/T1OSI         = No Connect: (configured as Discrete In) (possible future Timer 1 OSI)
; 17) RC2/CCP1          = Discrete Out: Peizo Buzzer (when J9 in place) (TEMPORARILY DISCRETE IN)
; 18) RC3/SCK/SCL       = I2C SCL: MSSP implementation of I2C requires pin as Discrete In. (Not used for SPI.)
; 19) RD0/PSP0          = Discrete Out/Discrete In: LCD data bit 4
; 20) RD1/PSP1          = Discrete Out/Discrete In: LCD data bit 5
; 21) RD2/PSP2          = Discrete Out/Discrete In: LCD data bit 6
; 22) RD3/PSP3          = Discrete Out/Discrete In: LCD data bit 7
; 23) RC4/SDI/SDA       = I2C SDA: MSSP implementation of I2C requires pin as Discrete In. (Not used for SPI.)
; 24) RC5/SDO           = No Connect: (configured as Discrete In) (Not used for SPI.)
; 25) RC6/TX/CK         = USART TX: RS-232 driver, USART control of this pin requires pin as Discrete In.
; 26) RC7/RX/DT         = USART RX: RS-232 driver, USART control of this pin requires pin as Discrete In.
; 27) RD4/PSP4          = No Connect: (configured as Discrete In)
; 28) RD5/PSP5          = No Connect: (configured as Discrete In)
; 29) RD6/PSP6          = No Connect: (configured as Discrete In)
; 30) RD7/PSP7          = No Connect: (configured as Discrete In)
; 31) Vss               = Programming connector(3) (Ground)
; 32) Vdd               = Programming connector(2) (+5 VDC)
; 33) RB0/INT           = Discrete Out: LED RB0 (when J6 in place)
;                       = Discrete In: RB0/INT also Pushbutton S3 (active low, with debounce H/W)
; 34) RB1               = Discrete Out: LED RB1 (when J6 in place)
; 35) RB2               = Discrete Out: LED RB2 (when J6 in place)
; 36) RB3/PGM           = Discrete Out: LED RB3 (when J6 in place)
; 37) RB4               = No Connect: (configured as Discrete In)
; 38) RB5               = No Connect: (configured as Discrete In)
; 39) RB6/PGC           = Programming connector(5) (PGC) ICD2 control of this pin requires pin as Discrete In.
; 40) RB7/PGD           = Programming connector(4) (PGD) ICD2 control of this pin requires pin as Discrete In.
;
;*******************************************************************************
;
; Complete PIC18F2620 (28-pin device) pin assignments for PICDEM 2 Plus (2002) Demo Board
;   OR PICDEM 2 Plus (2010) Demo Board:
;
;  1) MCLR*/Vpp         = Reset/Programming connector(1): (active low, with debounce H/W)
;  2) RA0/AN0           = Analog In: Potentiometer Voltage 
;  3) RA1/AN1           = Discrete Out: LCD E (SLCD_CTRL_E)
;  4) RA2/AN2/Vref-     = Discrete Out: LCD RW (SLCD_CTRL_RW)
;  5) RA3/AN3/Vref+     = Discrete Out: LCD RS (SLCD_CTRL_RS)
;  6) RA4/TOCKI         = Discrete In:  Pushbutton S2 (active low, no debounce H/W)
;  7) RA5/AN4/SS*       = No Connect: (configured as Discrete In)
;  8) Vss               = Programming connector(3) (Ground)
;
;   External 4 Mhz crystal installed in Y2, PICDEM 2 Plus (2002).
;
;  9) OSC1/CLKIN        = 4 MHz clock in (4 MHz/4 = 1 MHz = 1us instr cycle)
;
;   External 4 Mhz crystal NOT installed in Y2, PICDEM 2 Plus (2010) PN 02-01630-1.
;   Note that Jumper J7 should not be connected.
;
;  9) OSC1/CLKIN        = No Connect.
;
; 10) OSC2/CLKOUT       = (non-configurable output)
; 11) RC0/T1OSO/T1CKI   = No Connect: (configured as Discrete In) (possible future Timer 1 OSO)
; 12) RC1/T1OSI         = No Connect: (configured as Discrete In) (possible future Timer 1 OSI)
; 13) RC2/CCP1          = Discrete Out: Peizo Buzzer (when J9 in place) (TEMPORARILY DISCRETE IN)
; 14) RC3/SCK/SCL       = I2C SCL: MSSP implementation of I2C requires pin as Discrete In. (Not used for SPI.)
; 15) RC4/SDI/SDA       = I2C SDA: MSSP implementation of I2C requires pin as Discrete In. (Not used for SPI.)
; 16) RC5/SDO           = No Connect: (configured as Discrete In) (Not used for SPI.)
; 17) RC6/TX/CK         = USART TX: RS-232 driver, USART control of this pin requires pin as Discrete In.
; 18) RC7/RX/DT         = USART RX: RS-232 driver, USART control of this pin requires pin as Discrete In.
; 19) Vss               = Programming connector(3) (Ground)
; 20) Vdd               = Programming connector(2) (+5 VDC)
; 21) RB0/INT           = Discrete Out: LED RB0 (when J6 in place)
;                       = Discrete In: RB0/INT also Pushbutton S3 (active low, with debounce H/W)
; 22) RB1               = Discrete Out: LED RB1 (when J6 in place)
; 23) RB2               = Discrete Out: LED RB2 (when J6 in place)
; 24) RB3/PGM           = Discrete Out: LED RB3 (when J6 in place)
; 25) RB4               = No Connect: (configured as Discrete In)
; 26) RB5               = No Connect: (configured as Discrete In)
; 27) RB6/PGC           = Programming connector(5) (PGC) ICD2 control of this pin requires pin as Discrete In.
; 28) RB7/PGD           = Programming connector(4) (PGD) ICD2 control of this pin requires pin as Discrete In.
;
;*******************************************************************************
;
        errorlevel -302 
;
        #include    <ucfg.inc>  ; Configure board and proc, #include <proc.inc>
;
;*******************************************************************************
;
    IF UCFG_BOARD==UCFG_PD2P_2002
;
;   UCFG_PD2P_2002 specified.
;   *************************
;
; Hardware: PICdem 2 Plus 2002 board with 4 MHz external crystal.
;           TC74 digital temperature meter with I2C bus clocked at 100 kHz.
;
; Functions:
;  1) Read 1 A/D channel, convert A/D signal to engineering units and ASCII.
;  2) Read TC74 temperature value using I2C bus, convert to ASCII.
;  3) If 40-pin part, send ASCII text and commands to LCD display using 4-bit bus.
;  4) Send ASCII text to RS-232 port.  Receive and echo RS-232 bytes.
;
;   User CONFIG. Valid values are found in <"processor".inc>, e.g. <p18f2620.inc>.
;
;       UCFG_18F452 specified.
;       **********************
;
        IF UCFG_PROC==UCFG_18F452
        CONFIG  OSC = RCIO          ; RC oscillator w/ OSC2 configured as RA6
        CONFIG  OSCS = OFF          ; Oscillator system clock switch option is disabled (main oscillator is source)
        CONFIG  BOR = ON            ; Brown-out Reset enabled
        CONFIG  BORV = 20           ; VBOR set to 2.0V
        CONFIG  CCP2MUX = ON        ; CCP2 input/output is multiplexed with RC1
        CONFIG  STVR = ON           ; Stack full/underflow will cause Reset
        ENDIF
;
;       UCFG_18F2620 specified.
;       ***********************
;
        IF UCFG_PROC==UCFG_18F2620
        CONFIG  OSC = RCIO6         ; External RC oscillator, port function on RA6
        CONFIG  FCMEN = OFF         ; Fail-Safe Clock Monitor disabled
        CONFIG  IESO = OFF          ; Oscillator Switchover mode disabled
        CONFIG  BOREN = SBORDIS     ; Brown-out Reset enabled in hardware only (SBOREN is disabled)
        CONFIG  BORV = 3            ; Minimum setting
        CONFIG  CCP2MX = PORTC      ; CCP2 input/output is multiplexed with RC1
        CONFIG  PBADEN = ON         ; PORTB<4:0> pins are configured as analog input channels on Reset
        CONFIG  LPT1OSC = OFF       ; Timer1 configured for higher power operation
        CONFIG  MCLRE = ON          ; MCLR pin enabled; RE3 input pin disabled
        CONFIG  STVREN = ON         ; Stack full/underflow will cause Reset
        CONFIG  XINST = OFF         ; Instruction set extension and Indexed Addressing mode disabled (Legacy mode)
        ENDIF
;
;       Common to all processors.
;       *************************
;
        CONFIG  PWRT = OFF          ; PWRT disabled
        CONFIG  WDT = OFF           ; WDT disabled (control is placed on the SWDTEN bit)
        CONFIG  WDTPS = 128         ; 1:128
        CONFIG  LVP = OFF           ; Single-Supply ICSP disabled
        CONFIG  CP0 = OFF           ; Block 0 (000200-001FFFh) not code-protected
        CONFIG  CP1 = OFF           ; Block 1 (002000-003FFFh) not code-protected
        CONFIG  CP2 = OFF           ; Block 2 (004000-005FFFh) not code-protected
        CONFIG  CP3 = OFF           ; Block 3 (006000-007FFFh) not code-protected
        CONFIG  CPB = OFF           ; Boot block (000000-0001FFh) not code-protected
        CONFIG  CPD = OFF           ; Data EEPROM not code-protected
        CONFIG  WRT0 = OFF          ; Block 0 (000200-001FFFh) not write-protected
        CONFIG  WRT1 = OFF          ; Block 1 (002000-003FFFh) not write-protected
        CONFIG  WRT2 = OFF          ; Block 2 (004000-005FFFh) not write-protected
        CONFIG  WRT3 = OFF          ; Block 3 (006000-007FFFh) not write-protected
        CONFIG  WRTC = OFF          ; Configuration registers (300000-3000FFh) not write-protected
        CONFIG  WRTB = OFF          ; Boot Block (000000-0001FFh) not write-protected
        CONFIG  WRTD = OFF          ; Data EEPROM not write-protected
        CONFIG  EBTR0 = OFF         ; Block 0 (000200-001FFFh) not protected from table reads executed in other blocks
        CONFIG  EBTR1 = OFF         ; Block 1 (002000-003FFFh) not protected from table reads executed in other blocks
        CONFIG  EBTR2 = OFF         ; Block 2 (004000-005FFFh) not protected from table reads executed in other blocks
        CONFIG  EBTR3 = OFF         ; Block 3 (006000-007FFFh) not protected from table reads executed in other blocks
        CONFIG  EBTRB = OFF         ; Boot Block (000000-0001FFh) not protected from table reads executed in other blocks
;
; User APP defines.
;
#define UAPP_OSCCON_VAL  0x00
;
;   Use primary oscillator/clock input pin
;
; bit 7 : N/A   : 0 : (unimplemented, don't care)
; bit 6 : N/A   : 0 : (unimplemented, don't care)
; bit 5 : N/A   : 0 : (unimplemented, don't care)
; bit 4 : N/A   : 0 : (unimplemented, don't care)
; bit 3 : N/A   : 0 : (unimplemented, don't care)
; bit 2 : N/A   : 0 : (unimplemented, don't care)
; bit 1 : N/A   : 0 : (unimplemented, don't care)
; bit 0 : SCS0  : 0 : System Clock Select, use primary oscillator/clock input pin
;
#define UAPP_PORTA_VAL  0x00
;
; PORTA cleared so any bits later programmed as output initialized to 0.
;
;   NOTE: ADCON1 must have ??? to ensure RA3:RA0 and RA5 not set as analog inputs.
;       On Power-On Reset they are configured as analog inputs.
;
;   NOTE: CMCON must have ??? to ensure the comparators are off and use RA3:RA0 as discrete in.
;
; bit 7 : OSC1/CLKIN/RA7            : 0 : Using OSC1 (don't care)
; bit 6 : OSC2/CLKOUT/RA6           : 0 : Using OSC2 (don't care)
; bit 5 : RA5/AN4/SS*/HLVDIN/C2OUT  : 0 : DiosPro TX TTL: (unused, configured as Discrete In)
; bit 4 : RA4/T0KI/C1OUT            : 0 : DiosPro RX TTL: (unused, configured as Discrete In)
; bit 3 : RA3/AN2/Vref+             : 0 : RA3 input (don't care) 
; bit 2 : RA2/AN2/Vref-/CVref       : 0 : (unused, configured as RA2 input) (don't care)
; bit 1 : RA1/AN1                   : 0 : (unused, configured as RA1 input) (don't care)
; bit 0 : RA0/AN0                   : 0 : (unused, configured as RA0 input) (don't care)
;
#define UAPP_TRISA_VAL  0x3f
;
; Set all PORTA to inputs.
;
; bit 7 : TRISA7 : 0 : Using OSC1, overridden by CONFIG1H (don't care)
; bit 6 : TRISA6 : 0 : Using OSC2, overridden by CONFIG1H (don't care)
; bit 5 : DDRA5  : 1 : Disrete In
; bit 4 : DDRA4  : 1 : Disrete In
; bit 3 : DDRA3  : 1 : Disrete In
; bit 2 : DDRA2  : 1 : Disrete In
; bit 1 : DDRA1  : 1 : Disrete In
; bit 0 : DDRA0  : 1 : Disrete In
;
#define UAPP_PORTB_VAL  0x00
;
; PORTB cleared so any bits later programmed as output initialized to 0.
;
; bit 7 : RB7/PGD               : 0 : Discrete In: (don't care)
; bit 6 : RB6/PGC               : 0 : Discrete In: (don't care)
; bit 5 : RB5/KB11/PGM          : 0 : Discrete In: (don't care)
; bit 4 : RB4/KB10/AN11         : 0 : Discrete In: (don't care)
; bit 3 : RB3/AN9/CCP2          : 0 : Discrete Out: Init to 0.
; bit 2 : RB2/INT2/AN8          : 0 : Discrete Out: Init to 0.
; bit 1 : RB1/INT1/AN10         : 0 : Discrete Out: Init to 0.
; bit 0 : RB0/INT0/FLT0/AN12    : 0 : Discrete Out: Init to 0. 
;
#define UAPP_TRISB_VAL  0xf0
;
; Set TRISB RB0-RB3 to outputs for LED's.
; Set TRISB RB4-RB7 to inputs.  PGC and PGD need to be configured as high-impedance inputs.
;
; bit 7 : DDRB7  : 1 : Discrete In
; bit 6 : DDRB6  : 1 : Discrete In
; bit 5 : DDRB5  : 1 : Discrete In
; bit 4 : DDRB4  : 1 : Discrete In
; bit 3 : DDRB3  : 0 : Discrete Out
; bit 2 : DDRB2  : 0 : Discrete Out
; bit 1 : DDRB1  : 0 : Discrete Out
; bit 0 : DDRB0  : 0 : Discrete Out
;
#define UAPP_PORTC_VAL  0x00
;
; PORTC cleared so any bits later programmed as output initialized to 0.
;
; bit 7 : RC7/RX/DT         : 0 : Discrete In: (don't care)
; bit 6 : RC6/TX/CK         : 0 : Discrete In: (don't care)
; bit 5 : RC5/SDO           : 0 : Discrete In: (don't care)
; bit 4 : RC4/SDI/SDA       : 0 : Discrete In: (don't care)
; bit 3 : RC3/SCK/SCL       : 0 : Discrete In: (don't care)
; bit 2 : RC2/CCP1          : 0 : Discrete In: (don't care)
; bit 1 : RC1/T1OSI         : 0 : Discrete In: (don't care)
; bit 0 : RC0/T1OSO/T1CKI   : 0 : Discrete In: (don't care)
;
#define UAPP_TRISC_VAL  0xff
;
; Set TRISC to all inputs.  SDA and SCL must be configured as inputs for I2C.
;
; bit 7 : DDRC7  : 1 : Discrete In, USART RX: RS-232 driver, USART control of this pin requires pin as Discrete In.
; bit 6 : DDRC6  : 1 : Discrete In, USART TX: RS-232 driver, USART control of this pin requires pin as Discrete In.
; bit 5 : DDRC5  : 1 : Discrete In, No Connect (configured as Discrete In) (Not used for SPI.)
; bit 4 : DDRC4  : 1 : Discrete In, I2C SDA: MSSP implementation of I2C requires pin as Discrete In. (Not used for SPI.)
; bit 3 : DDRC3  : 1 : Discrete In, I2C SCL: MSSP implementation of I2C requires pin as Discrete In. (Not used for SPI.)
; bit 2 : DDRC2  : 1 : Discrete In, Discrete Out: Peizo Buzzer (when J9 in place) (TEMPORARILY DISCRETE IN)
; bit 1 : DDRC1  : 1 : Discrete In, No Connect
; bit 0 : DDRC0  : 1 : Discrete In, No Connect
;
;   PORTD and PORTE defines in case we have 40-pin part.
;
#define UAPP_PORTD_VAL  0x00
#define UAPP_TRISD_VAL  0xf8
#define UAPP_PORTE_VAL  0x00
#define UAPP_TRISE_VAL  0x07
;
#define UAPP_T1CON_VAL  0x30
;
; 1:8 pre-scaler; T1 oscillator disabled; T1SYNC* ignored;
; TMR1CS internal clock Fosc/4; Timer1 off.
;
; bit 7 : RD16    : 0 : Read/write Timer1 in two 8-bit operations
; bit 6 : T1RUN   : 0 : Device clock is derived from another source
; bit 5 : T1CKPS1 : 1 : Timer1 Input Clock Prescale Select, 0b11 -> 1:8 Prescale value
; bit 4 : T1CKPS0 : 1 : Timer1 Input Clock Prescale Select, 0b11 -> 1:8 Prescale value
; bit 3 : T1OSCEN : 0 : T1 oscillator disabled
; bit 2 : T1SYNC  : 0 : IT1SYNC* ignored
; bit 1 : TMR1CS  : 0 : Internal clock (Fosc/4)
; bit 0 : TMR1ON  : 0 : Timer1 disabled
;
#define UAPP_TMR1L_VAL  0x2c
#define UAPP_TMR1H_VAL  0xcf
;
; 4 Mhz Fosc/4 is base clock = 1 Mhz = 1.0 us per clock.
; 1:8 prescale = 1.0 * 8 = 8.0 us per clock.
; 12,500 counts * 8.0us/clock = 100,000 us/rollover = 100ms/rollover.
; Timer preload value = 65,536 - 12,500 = 53,036 = 0xcf2c.
;
    ENDIF
;
;*******************************************************************************
;
    IF UCFG_BOARD==UCFG_PD2P_2010
;
;   UCFG_PD2P_2010 specified.
;   *************************
;
; Hardware: PICdem 2 Plus 2010 board with no external crystal.
;           TC74 digital temperature meter with I2C bus clocked at 100 kHz.
;
; Functions:
;  1) Read 1 A/D channel, convert A/D signal to engineering units and ASCII.
;  2) Read TC74 temperature value using I2C bus, convert to ASCII.
;  3) If 40-pin part, send ASCII text and commands to LCD display using 4-bit bus.
;  4) Send ASCII text to RS-232 port.  Receive and echo RS-232 bytes.
;
;   User CONFIG. Valid values are found in <"processor".inc>, e.g. <p18f2620.inc>.
;
;       UCFG_18F452 specified.
;       **********************
;
        IF UCFG_PROC==UCFG_18F452
        CONFIG  OSC = RCIO          ; RC oscillator w/ OSC2 configured as RA6 (MUST INSTALL 4MHz crystal)
        CONFIG  OSCS = OFF          ; Oscillator system clock switch option is disabled (main oscillator is source)
        CONFIG  BOR = ON            ; Brown-out Reset enabled
        CONFIG  BORV = 20           ; VBOR set to 2.0V
        CONFIG  CCP2MUX = ON        ; CCP2 input/output is multiplexed with RC1
        CONFIG  STVR = ON           ; Stack full/underflow will cause Reset
        ENDIF
;
;       UCFG_18F2620 specified.
;       ***********************
;
        IF UCFG_PROC==UCFG_18F2620
        CONFIG  OSC = INTIO67       ; Internal oscillator block, port function on RA6 and RA7
        CONFIG  FCMEN = OFF         ; Fail-Safe Clock Monitor disabled
        CONFIG  IESO = OFF          ; Oscillator Switchover mode disabled
        CONFIG  BOREN = SBORDIS     ; Brown-out Reset enabled in hardware only (SBOREN is disabled)
        CONFIG  BORV = 3            ; Minimum setting
        CONFIG  CCP2MX = PORTC      ; CCP2 input/output is multiplexed with RC1
        CONFIG  PBADEN = ON         ; PORTB<4:0> pins are configured as analog input channels on Reset
        CONFIG  LPT1OSC = OFF       ; Timer1 configured for higher power operation
        CONFIG  MCLRE = ON          ; MCLR pin enabled; RE3 input pin disabled
        CONFIG  STVREN = ON         ; Stack full/underflow will cause Reset
        CONFIG  XINST = OFF         ; Instruction set extension and Indexed Addressing mode disabled (Legacy mode)
        ENDIF
;
;       Common to all processors.
;       *************************
;
        CONFIG  PWRT = OFF          ; PWRT disabled
        CONFIG  WDT = OFF           ; WDT disabled (control is placed on the SWDTEN bit)
        CONFIG  WDTPS = 128         ; 1:128
        CONFIG  LVP = OFF           ; Single-Supply ICSP disabled
        CONFIG  CP0 = OFF           ; Block 0 (000800-003FFFh) not code-protected
        CONFIG  CP1 = OFF           ; Block 1 (004000-007FFFh) not code-protected
        CONFIG  CP2 = OFF           ; Block 2 (008000-00BFFFh) not code-protected
        CONFIG  CP3 = OFF           ; Block 3 (00C000-00FFFFh) not code-protected
        CONFIG  CPB = OFF           ; Boot block (000000-0007FFh) not code-protected
        CONFIG  CPD = OFF           ; Data EEPROM not code-protected
        CONFIG  WRT0 = OFF          ; Block 0 (000800-003FFFh) not write-protected
        CONFIG  WRT1 = OFF          ; Block 1 (004000-007FFFh) not write-protected
        CONFIG  WRT2 = OFF          ; Block 2 (008000-00BFFFh) not write-protected
        CONFIG  WRT3 = OFF          ; Block 3 (00C000-00FFFFh) not write-protected
        CONFIG  WRTC = OFF          ; Configuration registers (300000-3000FFh) not write-protected
        CONFIG  WRTB = OFF          ; Boot Block (000000-0007FFh) not write-protected
        CONFIG  WRTD = OFF          ; Data EEPROM not write-protected
        CONFIG  EBTR0 = OFF         ; Block 0 (000800-003FFFh) not protected from table reads executed in other blocks
        CONFIG  EBTR1 = OFF         ; Block 1 (004000-007FFFh) not protected from table reads executed in other blocks
        CONFIG  EBTR2 = OFF         ; Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks
        CONFIG  EBTR3 = OFF         ; Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks
        CONFIG  EBTRB = OFF         ; Boot Block (000000-0007FFh) not protected from table reads executed in other blocks
;
; User APP defines.
;
#define UAPP_OSCCON_VAL  0x66
;
;   4 Mhz clock; use internal oscillator block.
;
; bit 7 : IDLEN : 0 : Device enters Sleep mode on SLEEP instruction
; bit 6 : IRCF2 : 1 : Internal Oscillator Frequency Select, 0b110 -> 4 Mhz
; bit 5 : IRCF1 : 1 : Internal Oscillator Frequency Select, 0b110 -> 4 Mhz
; bit 4 : IRCF0 : 0 : Internal Oscillator Frequency Select, 0b110 -> 4 Mhz
; bit 3 : OSTS  : 0 : Oscillator Start-up Timer time-out is running; primary oscillator is not ready
; bit 2 : IOFS  : 1 : INTOSC frequency is stable
; bit 1 : SCS1  : 1 : System Clock Select, 0b1x -> use internal oscillator block
; bit 0 : SCS0  : 0 : System Clock Select, 0b1x -> use internal oscillator block
;
#define UAPP_PORTA_VAL  0x00
;
; PORTA cleared so any bits later programmed as output initialized to 0.
;
;   NOTE: ADCON1 must have ??? to ensure RA3:RA0 and RA5 not set as analog inputs.
;       On Power-On Reset they are configured as analog inputs.
;
;   NOTE: CMCON must have ??? to ensure the comparators are off and use RA3:RA0 as discrete in.
;
; bit 7 : OSC1/CLKIN/RA7            : 0 : Using OSC1 (don't care)
; bit 6 : OSC2/CLKOUT/RA6           : 0 : Using OSC2 (don't care)
; bit 5 : RA5/AN4/SS*/HLVDIN/C2OUT  : 0 : DiosPro TX TTL: (unused, configured as Discrete In)
; bit 4 : RA4/T0KI/C1OUT            : 0 : DiosPro RX TTL: (unused, configured as Discrete In)
; bit 3 : RA3/AN2/Vref+             : 0 : RA3 input (don't care) 
; bit 2 : RA2/AN2/Vref-/CVref       : 0 : (unused, configured as RA2 input) (don't care)
; bit 1 : RA1/AN1                   : 0 : (unused, configured as RA1 input) (don't care)
; bit 0 : RA0/AN0                   : 0 : (unused, configured as RA0 input) (don't care)
;
#define UAPP_TRISA_VAL  0x3f
;
; Set all PORTA to inputs.
;
; bit 7 : TRISA7 : 0 : Using OSC1, overridden by CONFIG1H (don't care)
; bit 6 : TRISA6 : 0 : Using OSC2, overridden by CONFIG1H (don't care)
; bit 5 : DDRA5  : 1 : Disrete In
; bit 4 : DDRA4  : 1 : Disrete In
; bit 3 : DDRA3  : 1 : Disrete In
; bit 2 : DDRA2  : 1 : Disrete In
; bit 1 : DDRA1  : 1 : Disrete In
; bit 0 : DDRA0  : 1 : Disrete In
;
#define UAPP_PORTB_VAL  0x00
;
; PORTB cleared so any bits later programmed as output initialized to 0.
;
; bit 7 : RB7/PGD               : 0 : Discrete In: (don't care)
; bit 6 : RB6/PGC               : 0 : Discrete In: (don't care)
; bit 5 : RB5/KB11/PGM          : 0 : Discrete In: (don't care)
; bit 4 : RB4/KB10/AN11         : 0 : Discrete In: (don't care)
; bit 3 : RB3/AN9/CCP2          : 0 : Discrete Out: Init to 0.
; bit 2 : RB2/INT2/AN8          : 0 : Discrete Out: Init to 0.
; bit 1 : RB1/INT1/AN10         : 0 : Discrete Out: Init to 0.
; bit 0 : RB0/INT0/FLT0/AN12    : 0 : Discrete Out: Init to 0. 
;
#define UAPP_TRISB_VAL  0xf0
;
; Set TRISB RB0-RB3 to outputs for LED's.
; Set TRISB RB4-RB7 to inputs.  PGC and PGD need to be configured as high-impedance inputs.
;
; bit 7 : DDRB7  : 1 : Discrete In
; bit 6 : DDRB6  : 1 : Discrete In
; bit 5 : DDRB5  : 1 : Discrete In
; bit 4 : DDRB4  : 1 : Discrete In
; bit 3 : DDRB3  : 0 : Discrete Out
; bit 2 : DDRB2  : 0 : Discrete Out
; bit 1 : DDRB1  : 0 : Discrete Out
; bit 0 : DDRB0  : 0 : Discrete Out
;
#define UAPP_PORTC_VAL  0x00
;
; PORTC cleared so any bits later programmed as output initialized to 0.
;
; bit 7 : RC7/RX/DT         : 0 : Discrete In: (don't care)
; bit 6 : RC6/TX/CK         : 0 : Discrete In: (don't care)
; bit 5 : RC5/SDO           : 0 : Discrete In: (don't care)
; bit 4 : RC4/SDI/SDA       : 0 : Discrete In: (don't care)
; bit 3 : RC3/SCK/SCL       : 0 : Discrete In: (don't care)
; bit 2 : RC2/CCP1          : 0 : Discrete In: (don't care)
; bit 1 : RC1/T1OSI         : 0 : Discrete In: (don't care)
; bit 0 : RC0/T1OSO/T1CKI   : 0 : Discrete In: (don't care)
;
#define UAPP_TRISC_VAL  0xff
;
; Set TRISC to all inputs.  SDA and SCL must be configured as inputs for I2C.
;
; bit 7 : DDRC7  : 1 : Discrete In, USART RX: RS-232 driver, USART control of this pin requires pin as Discrete In.
; bit 6 : DDRC6  : 1 : Discrete In, USART TX: RS-232 driver, USART control of this pin requires pin as Discrete In.
; bit 5 : DDRC5  : 1 : Discrete In, No Connect (configured as Discrete In) (Not used for SPI.)
; bit 4 : DDRC4  : 1 : Discrete In, I2C SDA: MSSP implementation of I2C requires pin as Discrete In. (Not used for SPI.)
; bit 3 : DDRC3  : 1 : Discrete In, I2C SCL: MSSP implementation of I2C requires pin as Discrete In. (Not used for SPI.)
; bit 2 : DDRC2  : 1 : Discrete In, Discrete Out: Peizo Buzzer (when J9 in place) (TEMPORARILY DISCRETE IN)
; bit 1 : DDRC1  : 1 : Discrete In, No Connect
; bit 0 : DDRC0  : 1 : Discrete In, No Connect
;
;   PORTD and PORTE defines in case we have 40-pin part.
;
#define UAPP_PORTD_VAL  0x00
#define UAPP_TRISD_VAL  0xf8
#define UAPP_PORTE_VAL  0x00
#define UAPP_TRISE_VAL  0x07
;
#define UAPP_T1CON_VAL  0x30
;
; 1:8 pre-scaler; T1 oscillator disabled; T1SYNC* ignored;
; TMR1CS internal clock Fosc/4; Timer1 off.
;
; bit 7 : RD16    : 0 : Read/write Timer1 in two 8-bit operations
; bit 6 : T1RUN   : 0 : Device clock is derived from another source
; bit 5 : T1CKPS1 : 1 : Timer1 Input Clock Prescale Select, 0b11 -> 1:8 Prescale value
; bit 4 : T1CKPS0 : 1 : Timer1 Input Clock Prescale Select, 0b11 -> 1:8 Prescale value
; bit 3 : T1OSCEN : 0 : T1 oscillator disabled
; bit 2 : T1SYNC  : 0 : IT1SYNC* ignored
; bit 1 : TMR1CS  : 0 : Internal clock (Fosc/4)
; bit 0 : TMR1ON  : 0 : Timer1 disabled
;
#define UAPP_TMR1L_VAL  0x2c
#define UAPP_TMR1H_VAL  0xcf
;
; 4 Mhz Fosc/4 is base clock = 1 Mhz = 1.0 us per clock.
; 1:8 prescale = 1.0 * 8 = 8.0 us per clock.
; 12,500 counts * 8.0us/clock = 100,000 us/rollover = 100ms/rollover.
; Timer preload value = 65,536 - 12,500 = 53,036 = 0xcf2c.
;
    ENDIF
;
;*******************************************************************************
;
    IF UCFG_BOARD==UCFG_DJPCB_280B
;
;   UCFG_DJPCB_280B specified.
;   **************************
;
; Hardware: Kairos Autonomi 280B circuit board.
;           Microchip PIC18F2620 processor with 10 MHz input resonator.
;
; Functions:
;  1) Read length of low pulse on pin 18 (AIN3), determine transmission mode.
;  2) Read contents of 8 digital inputs and send to 8 digital outputs.
;  3) Send ASCII text to RS-232 port.  Receive and echo RS-232 bytes.
;
; Complete PIC18F2620 (28-pin device) pin assignments for KA board 280B:
;
;  1) MCLR*/Vpp/RE3             = Reset/Programming connector(1): ATN TTL (active low)
;  2) RA0/AN0                   = Analog In: AIN0
;  3) RA1/AN1                   = Analog In: AIN1
;  4) RA2/AN2/Vref-/CVref       = Analog In: AIN2
;  5) RA3/AN3/Vref+             = Analog In: AIN3
;  6) RA4/T0KI/C1OUT            = RX TTL: (configured as Discrete In) (Not used for I2C.)
;  7) RA5/AN4/SS*/HLVDIN/C2OUT  = TX TTL: (configured as Discrete In) (Not used for I2C.)
;  8) Vss                       = Programming connector(3) (Ground)
;
;   External 10 Mhz ceramic oscillator installed in pins 10, 11; KA board 280B.
;
;  9) OSC1/CLKIN/RA7        = 10 MHz clock in (10 MHz * 4(PLL)/4 = 10 MHz = 0.1us instr cycle)
; 10) OSC2/CLKOUT/RA6       = (non-configurable output)
;
; 11) RC0/T1OSO/T13CKI      = Discrete Out: DOUT17
; 12) RC1/T1OSI/CCP2        = Discrete Out: DOUT16
; 13) RC2/CCP1              = Discrete Out: DOUT15
; 14) RC3/SCK/SCL           = Discrete Out: DOUT14 (Not used for SPI.) (Not used for I2C.)
; 15) RC4/SDI/SDA           = Discrete Out: DOUT13 (Not used for SPI.) (Not used for I2C.)
; 16) RC5/SDO               = Discrete Out: DOUT12 (Not used for SPI.)
; 17) RC6/TX/CK             = Discrete Out, USART TX (RS-232): DOUT11
;                               USART control of this pin requires pin as Discrete In.
; 18) RC7/RX/DT             = Discrete Out, USART TX (RS-232): DOUT10
;                               USART control of this pin requires pin as Discrete In.
; 19) Vss                   = Programming connector(3) (Ground)
; 20) Vdd                   = Programming connector(2) (+5 VDC)
; 21) RB0/INT0/FLT0/AN12    = Discrete In: DIN7
; 22) RB1/INT1/AN10         = Discrete In: DIN6
; 23) RB2/INT2/AN8          = Discrete In: DIN5
; 24) RB3/AN9/CCP2          = Discrete In: DIN4
; 25) RB4/KB10/AN11         = Discrete In: DIN3
; 26) RB5/KB11/PGM          = Discrete In: DIN2
; 27) RB6/KB12/PGC          = Discrete In, Programming connector(5) (PGC): DIN1
;                               ICD2 control of this pin requires pin as Discrete In.
; 28) RB7/KB13/PGD          = Discrete In, Programming connector(4) (PGD): DIN0
;                               ICD2 control of this pin requires pin as Discrete In.
;
;   User CONFIG. Valid values are found in <"processor".inc>, e.g. <p18f2620.inc>.
;
        CONFIG  OSC = HSPLL         ; HS oscillator, PLL enabled (Clock Frequency = 4 x Fosc1)
        CONFIG  FCMEN = OFF         ; Fail-Safe Clock Monitor disabled
        CONFIG  IESO = OFF          ; Oscillator Switchover mode disabled
        CONFIG  PWRT = OFF          ; PWRT disabled
        CONFIG  BOREN = SBORDIS     ; Brown-out Reset enabled in hardware only (SBOREN is disabled)
        CONFIG  BORV = 3            ; Minimum setting
        CONFIG  WDT = OFF           ; WDT disabled (control is placed on the SWDTEN bit)
        CONFIG  WDTPS = 32768       ; 1:32768
        CONFIG  CCP2MX = PORTC      ; CCP2 input/output is multiplexed with RC1
        CONFIG  PBADEN = ON         ; PORTB<4:0> pins are configured as analog input channels on Reset
        CONFIG  LPT1OSC = OFF       ; Timer1 configured for higher power operation
        CONFIG  MCLRE = ON          ; MCLR pin enabled; RE3 input pin disabled
        CONFIG  STVREN = ON         ; Stack full/underflow will cause Reset
        CONFIG  LVP = OFF           ; Single-Supply ICSP disabled
        CONFIG  XINST = OFF         ; Instruction set extension and Indexed Addressing mode disabled (Legacy mode)
        CONFIG  CP0 = OFF           ; Block 0 (000800-003FFFh) not code-protected
        CONFIG  CP1 = OFF           ; Block 1 (004000-007FFFh) not code-protected
        CONFIG  CP2 = OFF           ; Block 2 (008000-00BFFFh) not code-protected
        CONFIG  CP3 = OFF           ; Block 3 (00C000-00FFFFh) not code-protected
        CONFIG  CPB = OFF           ; Boot block (000000-0007FFh) not code-protected
        CONFIG  CPD = OFF           ; Data EEPROM not code-protected
        CONFIG  WRT0 = OFF          ; Block 0 (000800-003FFFh) not write-protected
        CONFIG  WRT1 = OFF          ; Block 1 (004000-007FFFh) not write-protected
        CONFIG  WRT2 = OFF          ; Block 2 (008000-00BFFFh) not write-protected
        CONFIG  WRT3 = OFF          ; Block 3 (00C000-00FFFFh) not write-protected
        CONFIG  WRTC = OFF          ; Configuration registers (300000-3000FFh) not write-protected
        CONFIG  WRTB = OFF          ; Boot Block (000000-0007FFh) not write-protected
        CONFIG  WRTD = OFF          ; Data EEPROM not write-protected
        CONFIG  EBTR0 = OFF         ; Block 0 (000800-003FFFh) not protected from table reads executed in other blocks
        CONFIG  EBTR1 = OFF         ; Block 1 (004000-007FFFh) not protected from table reads executed in other blocks
        CONFIG  EBTR2 = OFF         ; Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks
        CONFIG  EBTR3 = OFF         ; Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks
        CONFIG  EBTRB = OFF         ; Boot Block (000000-0007FFh) not protected from table reads executed in other blocks
;
; User APP defines.
;
#define UAPP_OSCCON_VAL  0x64
;
;   40 Mhz clock; use HS PLL with external 10 MHz resonator.
;
;   NOTE: Configuration register CONFIG1H (@0x300001) must be set to 0x06.
;       FOSC3:FOSC0: 0b0110 = HS oscillator, PLL enabled (Clock Frequency = 4 x Fosc1)
;
; bit 7 : IDLEN : 0 : Device enters Sleep mode on SLEEP instruction
; bit 6 : IRCF2 : 1 : Internal Oscillator Frequency Select, 0b110 -> 4 Mhz (not used)
; bit 5 : IRCF1 : 1 : Internal Oscillator Frequency Select, 0b110 -> 4 Mhz (not used)
; bit 4 : IRCF0 : 0 : Internal Oscillator Frequency Select, 0b110 -> 4 Mhz (not used)
; bit 3 : OSTS  : 0 : Oscillator Start-up Timer time-out is running; primary oscillator is not ready
; bit 2 : IOFS  : 1 : INTOSC frequency is stable
; bit 1 : SCS1  : 0 : System Clock Select, 0b00 -> use primary oscillator
; bit 0 : SCS0  : 0 : System Clock Select, 0b00 -> use primary oscillator
;
#define UAPP_PORTA_VAL  0x00
;
; PORTA cleared so any bits later programmed as output initialized to 0.
;
;   NOTE: ADCON1 must have ??? to ensure RA3:RA0 and RA5 not set as analog inputs.
;       On Power-On Reset they are configured as analog inputs.
;
;   NOTE: CMCON must have ??? to ensure the comparators are off and use RA3:RA0 as discrete in.
;
; bit 7 : OSC1/CLKIN/RA7            : 0 : Using OSC1 (don't care)
; bit 6 : OSC2/CLKOUT/RA6           : 0 : Using OSC2 (don't care)
; bit 5 : RA5/AN4/SS*/HLVDIN/C2OUT  : 0 : DiosPro TX TTL: (unused, configured as Discrete In)
; bit 4 : RA4/T0KI/C1OUT            : 0 : DiosPro RX TTL: (unused, configured as Discrete In)
; bit 3 : RA3/AN2/Vref+             : 0 : RA3 input (don't care) 
; bit 2 : RA2/AN2/Vref-/CVref       : 0 : (unused, configured as RA2 input) (don't care)
; bit 1 : RA1/AN1                   : 0 : (unused, configured as RA1 input) (don't care)
; bit 0 : RA0/AN0                   : 0 : (unused, configured as RA0 input) (don't care)
;
#define UAPP_TRISA_VAL  0x3f
;
; Set all PORTA to inputs.
;
; bit 7 : TRISA7 : 0 : Using OSC1, overridden by CONFIG1H (don't care)
; bit 6 : TRISA6 : 0 : Using OSC2, overridden by CONFIG1H (don't care)
; bit 5 : DDRA5  : 1 : Disrete In
; bit 4 : DDRA4  : 1 : Disrete In
; bit 3 : DDRA3  : 1 : Disrete In
; bit 2 : DDRA2  : 1 : Disrete In
; bit 1 : DDRA1  : 1 : Disrete In
; bit 0 : DDRA0  : 1 : Disrete In
;
#define UAPP_PORTB_VAL  0x00
;
; PORTB cleared so any bits later programmed as output initialized to 0.
;
; bit 7 : RB7/PGD               : 0 : Discrete In: (don't care)
; bit 6 : RB6/PGC               : 0 : Discrete In: (don't care)
; bit 5 : RB5/KB11/PGM          : 0 : Discrete In: (don't care)
; bit 4 : RB4/KB10/AN11         : 0 : Discrete In: (don't care)
; bit 3 : RB3/AN9/CCP2          : 0 : Discrete In: (don't care)
; bit 2 : RB2/INT2/AN8          : 0 : Discrete In: (don't care)
; bit 1 : RB1/INT1/AN10         : 0 : Discrete In: (don't care)
; bit 0 : RB0/INT0/FLT0/AN12    : 0 : Discrete In: (don't care)
;
#define UAPP_TRISB_VAL  0xff
;
; Set TRISB RB0-RB7 to inputs.  PGC and PGD need to be configured as high-impedance inputs.
;
; bit 7 : DDRB7  : 1 : Disrete In
; bit 6 : DDRB6  : 1 : Disrete In
; bit 5 : DDRB5  : 1 : Disrete In
; bit 4 : DDRB4  : 1 : Disrete In
; bit 3 : DDRB3  : 1 : Disrete Out
; bit 2 : DDRB2  : 1 : Disrete Out
; bit 1 : DDRB1  : 1 : Disrete Out
; bit 0 : DDRB0  : 1 : Disrete Out
;
#define UAPP_PORTC_VAL  0x00
;
; PORTC cleared so any bits later programmed as output initialized to 0.
;
; bit 7 : RC7/RX/DT         : 0 : Discrete In: (don't care)
; bit 6 : RC6/TX/CK         : 0 : Discrete In: (don't care)
; bit 5 : RC5/SDO           : 0 : Discrete Out
; bit 4 : RC4/SDI/SDA       : 0 : Discrete Out
; bit 3 : RC3/SCK/SCL       : 0 : Discrete Out
; bit 2 : RC2/CCP1          : 0 : Discrete Out
; bit 1 : RC1/T1OSI         : 0 : Discrete Out
; bit 0 : RC0/T1OSO/T1CKI   : 0 : Discrete Out
;
#define UAPP_TRISC_VAL  0xc0
;
; Set TRISC to inputs for only USART RX/TX.  All others are outputs.
;
; bit 7 : DDRC7  : 1 : Discrete In, USART RX: RS-232 driver, USART control of this pin requires pin as Discrete In.
; bit 6 : DDRC6  : 1 : Discrete In, USART TX: RS-232 driver, USART control of this pin requires pin as Discrete In.
; bit 5 : DDRC5  : 0 : Discrete Out: DOUT12 (Not used for SPI.)
; bit 4 : DDRC4  : 0 : Discrete Out: DOUT13 (Not used for SPI.) (Not used for I2C.)
; bit 3 : DDRC3  : 0 : Discrete Out: DOUT14 (Not used for SPI.) (Not used for I2C.)
; bit 2 : DDRC2  : 0 : Discrete Out: DOUT15
; bit 1 : DDRC1  : 0 : Discrete Out: DOUT16
; bit 0 : DDRC0  : 0 : Discrete Out: DOUT17
;
#define UAPP_T1CON_VAL  0x30
;
; 1:8 pre-scaler; T1 oscillator disabled; T1SYNC* ignored;
; TMR1CS internal clock Fosc/4; Timer1 off.
;
; bit 7 : RD16    : 0 : Read/write Timer1 in two 8-bit operations
; bit 6 : T1RUN   : 0 : Device clock is derived from another source
; bit 5 : T1CKPS1 : 1 : Timer1 Input Clock Prescale Select, 0b11 -> 1:8 Prescale value
; bit 4 : T1CKPS0 : 1 : Timer1 Input Clock Prescale Select, 0b11 -> 1:8 Prescale value
; bit 3 : T1OSCEN : 0 : T1 oscillator disabled
; bit 2 : T1SYNC  : 0 : IT1SYNC* ignored
; bit 1 : TMR1CS  : 0 : Internal clock (Fosc/4)
; bit 0 : TMR1ON  : 0 : Timer1 disabled
;
#define UAPP_TMR1L_VAL  0xdc
#define UAPP_TMR1H_VAL  0x0b
;
; 40 Mhz Fosc/4 is base clock = 10 Mhz = 0.1 us per clock.
; 1:8 prescale = 1.0 * 8 = 0.8 us per clock.
; 62,500 counts * 0.8us/clock = 50,000 us/rollover = 50ms/rollover.
; Timer preload value = 65,536 - 62,500 = 3,036 = 0x0bdc.
;
    ENDIF
;
;*******************************************************************************
;
;  User application RAM variable definitions. (currently unused)
;
;;;UAPP_UdataSec   UDATA
;;;
;;;UAPP_Temp       res     1   ; General purpose scratch register (unused).
;
;*******************************************************************************
;
; User application Power-On Reset initialization.
;
UAPP_CodeSec    CODE
;
        GLOBAL  UAPP_POR_Init_PhaseA
UAPP_POR_Init_PhaseA
;
        movlw   UAPP_OSCCON_VAL
        movwf   OSCCON              ; Configure Fosc. Note relation to CONFIG1H.
        return
;
;*******************************************************************************
;
; User application Power-On Reset initialization.
;
        GLOBAL  UAPP_POR_Init_PhaseB
UAPP_POR_Init_PhaseB
;
        movlw   UAPP_PORTA_VAL  ; Clear initial data values in port.
        movwf   PORTA
;
        movlw   UAPP_TRISA_VAL  ; Set port bits function and direction.
        movwf   TRISA
;
        movlw   UAPP_PORTB_VAL  ; Clear initial data values in port.
        movwf   PORTB
;
        movlw   UAPP_TRISB_VAL  ; Set port bits function and direction.
        movwf   TRISB

        movlw   UAPP_PORTC_VAL  ; Clear initial data values in port.
        movwf   PORTC
;
        movlw   UAPP_TRISC_VAL  ; Set port bits function and direction.
        movwf   TRISC
;
;   40-pin parts need setup of PORTD and PORTE.
;
    IF UCFG_PROC==UCFG_18F452
        movlw   UAPP_PORTD_VAL  ; Clear initial data values in port.
        movwf   PORTD
;
        movlw   UAPP_TRISD_VAL  ; Set port bits function and direction.
        movwf   TRISD
;
        movlw   UAPP_PORTE_VAL  ; Clear initial data values in port.
        movwf   PORTE
;
        movlw   UAPP_TRISE_VAL  ; Set port bits function and direction.
        movwf   TRISE
    ENDIF
;
; PIE1 changed: ADIE, RCIE, TXIE, SSPIE, CCP1IE, TMR2IE, TMR1IE disabled.
;
        movlw   (0<<ADIE)|(0<<RCIE)|(0<<TXIE)|(0<<SSPIE)|(0<<CCP1IE)|(0<<TMR2IE)|(0<<TMR1IE)
        movwf   PIE1
;
; PIR1 changed: ADIF, RCIF, TXIF, SSPIF, CCP1IF, TMR2IF, TMR1IF cleared.
;
        movlw   (0<<ADIF)|(0<<RCIF)|(0<<TXIF)|(0<<SSPIF)|(0<<CCP1IF)|(0<<TMR2IF)|(0<<TMR1IF)
        movwf   PIR1
;
; PIE2 untouched; EEIE, BCLIE disabled.
; PIR2 untouched; EEIR, BCLIF remain cleared.
;
; IPEN cleared so disable priority levels on interrupts (PIC16 compatiblity mode.)
; RI set; TO set; PD set; POR set; BOR set; subsequent hardware resets will clear these bits.
;
        movlw   (0<<IPEN)|(1<<NOT_RI)|(1<<NOT_TO)|(1<<NOT_PD)|(1<<NOT_POR)|(1<<NOT_BOR)        
        movwf   RCON
;
; INTCON changed: GIE, PEIE enabled; TMR0IE, INT0IE, RBIE disabled; TMR0IF, INT0IF, RBIF cleared.
;
        movlw   (1<<GIE)|(1<<PEIE)|(0<<TMR0IE)|(0<<INT0IE)|(0<<RBIE)|(0<<TMR0IF)|(0<<INT0IF)|(0<<RBIF)
        movwf   INTCON
        return
;

;*******************************************************************************
;
; Init Timer1 module to generate timer interrupt every 100ms.
;
        GLOBAL  UAPP_Timer1Init
UAPP_Timer1Init
;
        movlw   UAPP_T1CON_VAL  
        movwf   T1CON                   ; Initialize Timer1 but don't start it.
;
        movlw   UAPP_TMR1L_VAL          ; Timer1 preload value, low byte.
        movwf   TMR1L
        movlw   UAPP_TMR1H_VAL          ; Timer1 preload value, high byte.
        movwf   TMR1H
;
        bsf     PIE1, TMR1IE            ; Enable Timer1 interrupts.
        bsf     T1CON,TMR1ON            ; Turn on Timer1 module.
;
        return
        end
