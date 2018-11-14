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

;=================== CODE =====================
            .global   _Q15SQRT
            .global   Q15SQRT

_Q15SQRT:
Q15SQRT:
    mov.w w0,w7
    clr.w w0
    cpsgt.w w0,w7
    nop
    nop
    nop
    mov.d w8,[w15++]
    ff1l w7,w3
    sub.w w3,#2,w1
    sl w7,w1,w2
    mov.w #0x8000,w0
    sub.w w2,w0,w5
    mov.w w5,w4
    sl w5,#1,w5
    mov.w #0x4000,w6
    mul.ss w4,w6,w6
    mul.ss w4,w5,w8
    mov.w #0xf000,w0
    mul.ss w0,w9,w2
    add.w w2,w6,w6
    addc.w w3,w7,w7
    mul.ss w9,w5,w8
    mov.w #0x800,w0
    mul.ss w0,w9,w2
    add.w w2,w6,w6
    addc.w w3,w7,w7
    mul.ss w9,w5,w8
    mov.w #0xfb00,w0
    mul.ss w0,w9,w2
    add.w w2,w6,w6
    addc.w w3,w7,w7
    mul.ss w9,w5,w8
    mov.w #0x380,w0
    mul.ss w0,w9,w2
    add.w w2,w6,w6
    addc.w w3,w7,w7
    mul.ss w9,w5,w8
    mov.w #0xfd60,w0
    mul.ss w0,w9,w2
    add.w w2,w6,w6
    addc.w w3,w7,w7
    mul.ss w9,w5,w8
    mov.w #0x210,w0
    mul.ss w0,w9,w2
    add.w w2,w6,w6
    addc.w w3,w7,w7
    mul.ss w9,w5,w8
    mov.w #0xfe53,w0
    mul.ss w0,w9,w2
    add.w w2,w6,w6
    addc.w w3,w7,w7
    lsr w6,#15,w6
    sl w7,#1,w0
    ior.w w6,w0,w6
    asr w7,#15,w7
    mov.w #0x8000,w0
    add.w w0,w6,w6
    addc.w w7,#0,w7
    lsr w1,#1,w2
    subr.w w2,#16,w0
    lsr w6,w2,w6
    sl w7,w0,w0
    ior.w w6,w0,w6
    asr w7,w2,w7
    btst.c w1,#0
    bra nc, Sqrt_else
    mov.w #0x5a82,w4
    mul.ss w6,w4,w0
    lsr w0,#15,w0
    sl w1,#1,w1
    ior.w w0,w1,w6
Sqrt_else:
    mov.w w6,w0
    mov.d [--w15],w8
    return


    .global _Delay
    .global Delay


_Delay:
Delay:
REPEAT w0
NOP
return


.end
