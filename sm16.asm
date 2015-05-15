        LIST
;*******************************************************************************
; tinyRTX Filename: sm16.inc (System Math 16-bit library.)
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
; 23Sep03  SHiggins@tinyRTX.com  Created from scratch.
; 10Jun14  SHiggins@tinyRTX.com  Renamed from ma16 to sm16 for conformance.
; 21Aug14  SHiggins@tinyRTX.com  Removed LOOPCOUNT, added SM16_16x16u.
;   14May15 Stephen_Higgins@KairosAutonomi.com  
;               Substitute #include <ucfg.inc> for <p18f452.inc>.
;
;*******************************************************************************
;
        errorlevel -302 
;
        #include    <ucfg.inc>  ; Configure board and proc, #include <proc.inc>
;
;*******************************************************************************
;
SM16_UdataSec   UDATA
;
        GLOBAL  AARGB7
        GLOBAL  AARGB6
        GLOBAL  AARGB5
        GLOBAL  AARGB4
        GLOBAL  AARGB3
        GLOBAL  AARGB2
        GLOBAL  AARGB1
        GLOBAL  AARGB0
        GLOBAL  AARG
AARGB7  res 1
AARGB6  res 1
AARGB5  res 1
AARGB4  res 1
AARGB3  res 1
AARGB2  res 1
AARGB1  res 1
AARGB0  res 1
AARG    res 1
;
        GLOBAL  BARGB3
        GLOBAL  BARGB2
        GLOBAL  BARGB1
        GLOBAL  BARGB0
        GLOBAL  BARG
BARGB3  res 1
BARGB2  res 1
BARGB1  res 1
BARGB0  res 1
BARG    res 1
;
        GLOBAL  TEMPB3
        GLOBAL  TEMPB2
        GLOBAL  TEMPB1
        GLOBAL  TEMPB0
        GLOBAL  TEMP
TEMPB3  res 1
TEMPB2  res 1
TEMPB1  res 1
TEMPB0  res 1
TEMP    res 1
;
;*******************************************************************************
;
;       16x16 Bit Unsigned Fixed Point Multiply 16x16 -> 32
;       Input:  16 bit unsigned fixed point multiplicand in AARGB0:1
;               16 bit unsigned fixed point multiplier in BARGB0:1
;       Output: 32 bit unsigned fixed point product in AARGB0:3
;
SM16_CodeSec    CODE
;
        global  SM16_16x16u
SM16_16x16u
;
        movff   AARGB0, TEMPB0
        movff   AARGB1, TEMPB1
;
        banksel BARGB0
        movf    BARGB0, W
        mulwf   TEMPB0          ; AARGB0 * BARGB0 -> PRODH:PRODL.
        movff   PRODH, AARGB0
        movff   PRODL, AARGB1
;
        movf    BARGB1, W
        mulwf   TEMPB1          ; AARGB1 * BARGB1 -> PRODH:PRODL.
        movff   PRODH, AARGB2
        movff   PRODL, AARGB3
;
        movf    BARGB0, W
        mulwf   TEMPB1          ; AARGB1 * BARGB0 -> PRODH:PRODL.
        movf    PRODL, W        ; Add cross products to existing results.
        addwf   AARGB2, F
        movf    PRODH, W
        addwfc  AARGB1, F
        clrf    WREG
        addwfc  AARGB0, F
;
        movf    BARGB1, W
        mulwf   TEMPB0          ; AARGB0 * BARGB1 -> PRODH:PRODL.
        movf    PRODL, W        ; Add cross products to existing results.
        addwf   AARGB2, F
        movf    PRODH, W
        addwfc  AARGB1, F
        clrf    WREG
        addwfc  AARGB0, F
;
        return
        end
