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
; ******************************************************************************
; MeasCompCurr
;
;Description:
;  Read Channels 1 & 2 of ADC, scale them as signed fractional values
;  using qKa, qKb and put the results qIa and qIb of MeasCurrParm.
;
;  Specifically the offset is used to correct the raw ADC by
;         CorrADC = CurrentIn - Offset
;
;  Do not call this routine until conversion is completed.
;
;  Scaling constant, qKa and qKb, must be set elsewhere such that
;         qIa = qKa * CorrADC1
;         qIb = qKb * CorrADC2
;
;Functional prototypes:
; void MeasCompCurr(int,int,MEAS_CURR_PARM_T *MeasCurrParm);
; void InitMeasCompCurr(int Offset_a,int Offset_b,MEAS_CURR_PARM_T *MeasCurrParm);
;
;On Start:   Must call InitMeasCompCurr.
;
;On Entry:   MeasCurrParm structure must contain qKa & qKb. ADC conversion results
;            must contain signed fractional value.
;
;On Exit:    MeasCurrParm will contain qIa & qIb.
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
; Registers modified: w0,w1,w2,w4,w5
;
;
;*******************************************************************

          .include "general.inc"
          .include "meascurr.inc"

; Register usage
.equ Current_Ia,w0      ; ADC Buffer Register
.equ Current_Ib,w1      ; ADC Buffer Register
.equ MeasCurrParm,w2    ; Base of MeasCurrParm structure

;=================== CODE =====================
            .section  .text
            .global   _MeasCompCurr
            .global   MeasCompCurr
; Inputs:
;  Ia
;  Ib
;  MeasCurrParm.Offseta
;  MeasCurrParm.Offseta
;  MeasCurrParm.qKa
;  MeasCurrParm.qKb

; Outputs:
;  MeasCurrParm.qIa
;  MeasCurrParm.qIb

_MeasCompCurr:
MeasCompCurr:

    ;; CorrADC1 = Current_Ia - iOffsetHa
    ;; qIa = 2 * qKa * CorrADC1
    mov.w     [MeasCurrParm+ADC_Offseta],w5
    sub.w     w5,Current_Ia,w5                      ; w5 = Current_Ia - Offset
    mov.w     [MeasCurrParm+ADC_qKa],w4
    mpy       w4*w5,A
    sac       A,#0,w4
    mov.w     w4,[MeasCurrParm+ADC_qIa]

    ;; CorrADC1 = Current_Ib - iOffsetHb
    ;; qIa = 2 * qKa * CorrADC1
    mov.w     [MeasCurrParm+ADC_Offsetb],w5
    sub.w     w5,Current_Ib,w5                      ; w5 = Current_Ib - Offset
    mov.w     [MeasCurrParm+ADC_qKb],w4
    mpy       w4*w5,A
    sac       A,#0,w4
    mov.w     w4,[MeasCurrParm+ADC_qIb]

    return

; Inputs:
;  Offseta
;  Offsetb

; Outputs:
;  MeasCurrParm.Offseta
;  MeasCurrParm.Offsetb

        .global   _InitMeasCompCurr
        .global   InitMeasCompCurr

_InitMeasCompCurr:
InitMeasCompCurr:

    mov.w     w0,[MeasCurrParm+ADC_Offseta]
    mov.w     w1,[MeasCurrParm+ADC_Offsetb]
    return

.end
