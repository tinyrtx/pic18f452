        LIST
;*******************************************************************************
; tinyRTX Filename: si2c.inc (System Inter-IC (I2C) communication services)
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
;   11Nov03  SHiggins@tinyRTX.com  Created from scratch.
;   14Apr15  Stephen_Higgins@KairosAutonomi.com
;               Added I2C_* as they are not in p18f2620.cin
;   14May15 Stephen_Higgins@KairosAutonomi.com  
;               Substitute #include <ucfg.inc> for <p18f452.inc>.
;
;*******************************************************************************
        NOLIST
;
; SI2C routines.
;
        EXTERN  SI2C_Tbl_HwState
        EXTERN  SI2C_Msg_Write
        EXTERN  SI2C_Msg_Wrt_Rd
;
; SI2C data structures.
;
        EXTERN  SI2C_Flags
        EXTERN  SI2C_AddrSlave
        EXTERN  SI2C_DataCntXmt
        EXTERN  SI2C_DataByteXmt00
        EXTERN  SI2C_DataByteXmt01
        EXTERN  SI2C_DataByteXmt02
        EXTERN  SI2C_DataByteXmt03
        EXTERN  SI2C_DataCntRcv
        EXTERN  SI2C_DataByteRcv00
        EXTERN  SI2C_DataByteRcv01
        EXTERN  SI2C_DataByteRcv02
        EXTERN  SI2C_DataByteRcv03
;
; For readability.  Sometimes not in device.inc files.
;
I2C_READ         EQU  H'0002'
I2C_START        EQU  H'0003'
I2C_STOP         EQU  H'0004'
I2C_DATA         EQU  H'0005'
;
        LIST
