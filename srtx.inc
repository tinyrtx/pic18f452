        LIST
;*******************************************************************************
; tinyRTX Filename: srtx.inc (System Real Time eXecutive)
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
;   16Oct03  SHiggins@tinyRTX.com   Created from scratch.
;   15Aug14  SHiggins@tinyRTX.com   Added SRTX_ComputedBraRCall and SRTX_ComputedGotoCall.
;   17Apr15  Stephen_Higgins@KairosAutonomi.com
;               Added SRTX_Sched_Cnt_TaskSIO.
;   20May15 Stephen_Higgins@KairosAutonomi.com  
;               Moved  SRTX_ComputedBraRCall and SRTX_ComputedGotoCall to SUTL
;               for consistency with C version.
;
;*******************************************************************************
        NOLIST
;
        EXTERN      SRTX_Sched_Cnt_TaskADC
        EXTERN      SRTX_Sched_Cnt_TaskI2C
        EXTERN      SRTX_Sched_Cnt_TaskSIO
;
        EXTERN      SRTX_Init
        EXTERN      SRTX_Scheduler
;
        LIST
