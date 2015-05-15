        LIST
;**********************************************************************************************
; tinyRTX Filename: uapp.inc (User APPlication interface)
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
;  31Oct03  SHiggins@tinyRTX.com  Created from scratch.
;  29Jul14  SHiggins@tinyRTX.com  Moved UAPP_Timer1Init to MACRO to save stack.
;  29Apr15  Stephen_Higgins@KairosAutonomi.com
;               Added UAPP_POR_Init_PhaseA.
;   06May15 Stephen_Higgins@KairosAutonomi.com
;               Moved UAPP_Timer1Init from MACRO to allow conditional assembly.
;
;*********************************************************************************************
        NOLIST
;
        EXTERN      UAPP_POR_Init_PhaseA
        EXTERN      UAPP_POR_Init_PhaseB
        EXTERN      UAPP_Timer1Init
;
        LIST
