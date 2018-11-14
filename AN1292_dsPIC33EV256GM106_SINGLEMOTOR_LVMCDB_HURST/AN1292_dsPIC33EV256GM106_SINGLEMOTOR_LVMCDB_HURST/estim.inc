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
; Variables for Speed estimation

.equ Estim_qDeltaT,             0   ;
.equ Estim_qRho,			    2   ;
.equ Estim_qRhoStateVar_L,      4  	;
.equ Estim_qRhoStateVar_H,      6  	;
.equ Estim_qOmegaMr,		    8   ;
.equ Estim_qLastIalpha,         10	;
.equ Estim_qLastIbeta,          12	;
.equ Estim_qDIalpha,		    14	; 
.equ Estim_qDIbeta,             16	;
.equ Estim_qEsa,		    	18	; 
.equ Estim_qEsb,		    	20	; 
.equ Estim_qEsd,		    	22	; 
.equ Estim_qEsq,		    	24	;
.equ Estim_qDiCounter,          26	;
.equ Estim_qVIndalpha,          28	;
.equ Estim_qVIndbeta,	    	30	;
.equ Estim_qEsdf,		    	32	;
.equ Estim_qEsdStateVar_L,      34	;
.equ Estim_qEsdStateVar_H,      36	;
.equ Estim_qEsqf,		    	38	;
.equ Estim_qEsqStateVar_L,      40	;
.equ Estim_qEsqStateVar_H,      42	;
.equ Estim_qKfilterEsdq,        44	;
.equ Estim_qVelEstim,	    	46	;
.equ Estim_qVelEstimFilterK,    48	;
.equ Estim_qVelEstimStateVar_L, 50	; 	
.equ Estim_qVelEstimStateVar_H, 52  ;
.equ Estim_qLastValpha,         54  ;
.equ Estim_qLastVbeta,          56	;
.equ Estim_qDIlimitLS,          58  ;
.equ Estim_qDIlimitHS,          60  ;
.equ Estim_qLastIalphaHS,       62  ;
.equ Estim_qLastIbetaHS,        78	;
.equ Estim_qRhoOffset,          94  ;

.extern	_EstimParm

; Motor parameters

.equ Motor_qRs,			0
.equ Motor_qLsDt,		2
.equ Motor_qLsDtBase,	4
.equ Motor_qInvKfi,		6
.equ Motor_qInvKfiBase,	8

.extern	_MotorEstimParm

; EOF
