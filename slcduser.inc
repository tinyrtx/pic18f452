        LIST
;**********************************************************************************************
; Filename: slcduser.inc (System Liquid Crystal Display - USER configuration)
;   SLCD configuration is defined by these #defines.
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
;   23Oct03 SHiggins@tinyRTX.com  Created from scratch.
;   14May15 Stephen_Higgins@KairosAutonomi.com  
;               Add #define SLCD_BUFFER_LINE_SIZE.
;
;*********************************************************************************************
        NOLIST
;
; SLCD configuration is defined by these #defines.
; User should set them to match hardware configuration.
; NOTE: User is also responsible for proper hardware initialization so
;   hardware addressed by these #defines may be used.
;
    IF UCFG_BOARD==UCFG_PD2P_2002
#define     SLCD_CTRL_TRIS      TRISA
#define     SLCD_CTRL_PORT      PORTA
#define     SLCD_CTRL_E         1
#define     SLCD_CTRL_RW        2
#define     SLCD_CTRL_RS        3
    ENDIF
;
    IF UCFG_BOARD==UCFG_PD2P_2010
#define     SLCD_CTRL_TRIS      TRISD
#define     SLCD_CTRL_PORT      PORTD
#define     SLCD_CTRL_PWR       7
#define     SLCD_CTRL_E         6
#define     SLCD_CTRL_RW        5
#define     SLCD_CTRL_RS        4
    ENDIF
;
#define     SLCD_DATA_TRIS      TRISD
#define     SLCD_DATA_PORT      PORTD
#define     SLCD_DATA_BITSUSED  0x0f
;
#define     SLCD_BUFFER_LINE_SIZE   0x10
;
        LIST
