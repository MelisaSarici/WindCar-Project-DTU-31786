;*********************************************************************************
; Copyright (c) 2014 released Microchip Technology Inc.  All rights reserved.
; Microchip licenses to you the right to use, modify, copy and distribute
; Software only when embedded on a Microchip microcontroller or digital signal
; controller that is integrated into your product or third party product
; (pursuant to the sublicense terms in the accompanying license agreement).
;
; You should refer to the license agreement accompanying this Software for
; additional information regarding your rights and obligations.
;
; SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
; EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
; MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
; IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
; CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
; OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
; INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
; CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
; SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
; (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
;*********************************************************************************
;*******************************************************************
; ReadADC0 and ReadSignedADC0
;  
;Description:        
;  Read Channel 0 of ADC, scale it using qK and put results in qADValue.
;  Do not call this routine until conversion is completed.
;
;  ReadADC0 range is qK*(0.0 ->0.9999).
;  ReadSignedADC0 range is qK*(-1.0 ->0.9999).
;
;  Scaling constant, qK, must be set elsewhere such that
;         iResult = 2 * qK * ADCBUF0
;  The factor of 2 is designed to allow qK to be given in 1.15.
;
;
;Functional prototype:
; Calculates unsigned value 0 -> 2*qK
; void ReadADC0( int16_t ADC_Result,READ_ADC_PARM_T* pParm )
; Calculates signed value -2*qK -> 2*qK
; void ReadSignedADC0( int16_t ADC_Result,READ_ADC_PARM_T* pParm )
;
;On Entry:   ReadADCParm structure must contain qK. ADC channel 0
;            must contain signed fractional value.
;
;On Exit:    ReadADCParm will contain qADValue
;
;Parameters: 
; Input arguments: None
;
; Return:
;   Void
;
; SFR Settings required:
;         CORCON.SATA  = 0
;     If there is any chance that Accumulator will overflow must set
;         CORCON.SATDW = 1  
;
; Support routines required: None
;
; Local Stack usage: None
;
; Registers modified: w0,w4,w5
;
;
;*******************************************************************
;
.include "general.inc"
.include "ReadADC.inc"

; Register usage
.equ ADC_Result,w0      ; ADC Value
.equ ParmBaseW,w1      ; Base of parm structure
.equ Work0W,   w4
.equ Work1W,   w5

;=================== CODE =====================

        .section  .text
        .global   _ReadADC0
        .global   ReadADC0

; Inputs:
;  ADC_Value

; Outputs:
;  Read_ADC_Parm.qADvalue
_ReadADC0:
ReadADC0:

    ;; iResult = 2 * qK * ADCBUF0

    mov.w     [ParmBaseW+ADC_qK],Work0W
    mov.w	  ADC_Result, Work1W

    ;; change from signed fractional to fractional, i.e. convert
    ;; from -1->.9999 to 0 -> 0.9999
    btg       Work1W,#15
    lsr.w     Work1W,Work1W

    mpy       Work0W*Work1W,A
    sac       A,#-1,Work0W
    mov.w     Work0W,[ParmBaseW+ADC_qADValue]
    return


            .global   _ReadSignedADC0
            .global   ReadSignedADC0

; Inputs:
;  ADC_Value

; Outputs:
;  Read_ADC_Parm.qADvalue
_ReadSignedADC0:
ReadSignedADC0:

    ;; iResult = 2 * qK * ADCBUF0

    mov.w     [ParmBaseW+ADC_qK],Work0W
    mov.w	  ADC_Result, Work1W

    mpy       Work0W*Work1W,A
    sac       A,#-1,Work0W
    mov.w     Work0W,[ParmBaseW+ADC_qADValue]

    return

.end
