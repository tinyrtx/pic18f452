        LIST
;*******************************************************************************
; tinyRTX Filename: ssio.inc (System Serial I/O communication services)
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
;   17Apr15  Stephen_Higgins@KairosAutonomi.com
;                                   Created from si2c.inc.
;
;*******************************************************************************
        NOLIST
;
; SSIO routines.
;
        EXTERN  SSIO_PutByteIntoTxHW
        EXTERN  SSIO_GetByteFromRxHW
        EXTERN  SSIO_InitFlags
        EXTERN  SSIO_InitTxBuffer
        EXTERN  SSIO_InitRxBuffer
        EXTERN  SSIO_PutByteTxBuffer
        EXTERN  SSIO_PutByteRxBuffer
        EXTERN  SSIO_GetByteTxBuffer
        EXTERN  SSIO_GetByteRxBuffer
;
        LIST
