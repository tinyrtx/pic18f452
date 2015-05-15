        LIST
;*******************************************************************************
; tinyRTX Filename: ucfg.inc (User ConFiGuration)
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
;   06May15  Stephen_Higgins@KairosAutonomi.com  Created from scratch.
;
;*******************************************************************************
		NOLIST
;
;   Define all supported board configurations.
;
#define UCFG_PD2P_2002      0x01    ; Microchip PICDEM2PLUS 2002 (has ext 4MHz)
#define UCFG_PD2P_2010      0x02    ; Microchip PICDEM2PLUS 2010 (no ext 4MHz)
#define UCFG_DJPCB_280B     0x10    ; Kairos Autonomi 280B: Utility Module I/O
;
;   Select board being used.
;
#define UCFG_BOARD  UCFG_PD2P_2010  ; Microchip PICDEM2PLUS 2010 (no ext 4MHz)
;
;   Define all supported processors.
;
#define UCFG_18F452     0x01        ; 40 pins.
#define UCFG_18F2620    0x02        ; 28 pins.
;
;   Setting processor being used done below.
;
;*******************************************************************************
;
;    Output message of configuration of board and procesor.
;
    IFDEF __18F452
        #include <p18f452.inc>
        #define UCFG_PROC   UCFG_18F452     ; Allows logical expressions.
        MESSG "Processor defined: 18F452"
    ENDIF       
    IFDEF __18F2620
        #include <p18f2620.inc>
        #define UCFG_PROC   UCFG_18F2620    ; Allows logical expressions.
        MESSG "Processor defined: 18F2620"
    ENDIF       
;           
    IF UCFG_BOARD==UCFG_PD2P_2002
        MESSG "Board defined: Microchip PICDEM2PLUS 2002"
    ENDIF       
    IF UCFG_BOARD==UCFG_PD2P_2010
        MESSG "Board defined: Microchip PICDEM2PLUS 2010"
    ENDIF       
    IF UCFG_BOARD==UCFG_DJPCB_280B
        MESSG "Board defined: UCFG_DJPCB_280B"
    ENDIF       
;
        LIST