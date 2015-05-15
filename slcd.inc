        LIST
;*******************************************************************************
; tinyRTX Filename: slcd.inc (System Liquid Crystal Display driver)
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
;   20Oct03 SHiggins@tinyRTX.com  Created from scratch.
;   14May15 Stephen_Higgins@KairosAutonomi.com
;               Minimal slcd.asm if board or chip doesn't support it.
;
;*******************************************************************************
        NOLIST
;
; SLCD routines.
;
    IF (UCFG_BOARD==UCFG_PD2P_2002 || UCFG_BOARD==UCFG_PD2P_2010) && UCFG_PROC==UCFG_18F452
;
;   (UCFG_PD2P_2002 OR UCFG_PD2P_2010 specified) AND UCFG_18F452 specified
;   **********************************************************************
;
;   NOTE: This condition must hold in order to use SLCD services.  That is,
;       the board must have an LCD and the chip has 40-pins.
;
        EXTERN      SLCD_Init
        EXTERN      SLCD_RefreshLine1
        EXTERN      SLCD_RefreshLine2
;
    ENDIF
;
; SLCD data structures.
;
        EXTERN      SLCD_BufferLine1
        EXTERN      SLCD_BufferLine2
;
        LIST
