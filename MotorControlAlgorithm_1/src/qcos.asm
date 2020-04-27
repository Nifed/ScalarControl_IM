;***********************************************************************; 
;*                                                                     *;
;* File Name     : qcos.asm                                            *;
;*                                                                     *;
;* Originator    : Advanced Embeeded Control (AEC)                     *;
;*                 Texas Instruments Inc.                              *;
;*                                                                     *;
;* Description   : This file contain source code for fixed point COS   *;
;*                function implemented using Tylor series              *;
;*                                                                     *;
;* Date          : 21/1/2002                                           *;
;***********************************************************************;
;* Tylor Series Approximation in the first quardrant:                  *;
;*                                                                     *;
;* sin(x) = 3.1406625*x+0.02026367*x^2-5.325196*x^3+0.5446778*x^4      *;
;* +1.800293*x^5, where 'x' is the normalized radians                  *;
;*                                                                     *;
;***********************************************************************;
  
                     
        .def _qcos  ;External ref
    
K5      .set 0x6480             ; Scaled to Q13
K4      .set 0x52FF             ; Scaled to Q20
K3      .set 0xAACC             ; Scaled to Q12
K2      .set 0x45B8             ; Scaled to Q15
K1      .set 0x7338             ; Scaled to Q14

_qcos:
        SETC   SXM,OVM 
        MOV    ACC,AL<<16       ; AH = AL
        ABS    ACC              ; ACC = abs(x) in Q15
        NEG    ACC              ; ACC=-x
        ADD    AH,#04000h       ; ACC=(PI/2 - x)

        CLRC   TC
        ABSTC  ACC               ;abs(PI/2-x)and store sign in TC
        MOV    T,AH              ;T=abs(PI/2-x) in Q15 and in first quardrant           

; Comment Next 2-instructions, if this works
        MPY     ACC,T,#K1           

;       MOV     AR5,#K1         ; AR5=K1    
;       MPY     ACC,T,AR5       ; ACC=K1*x in Q29       
        
        ADD     ACC,#K2<<14     ; ACC=K1*x+K2 in Q29
        
        QMPYL   ACC,XT,ACC      ; ACC=(K1*x+K2)*x in Q28
        ADD     AH,#K3          ; ACC=(K1*x+K2)*x+K3 in Q28

        QMPYL   ACC,XT,ACC      ; ACC=((K1*x+K2)*x+K3)*x in Q27
        ADD     ACC,#K4<<7      ; ACC=((K1*x+K2)*x+K3)*x+K4 in Q27
        
        QMPYL   ACC,XT,ACC      ; ACC=(((K1*x+K2)*x+K3)*x+K4)*x in Q26
        ADD     ACC,#K5<<13     ; ACC=(((K1*x+K2)*x+K3)*x+K4)*x+K5 in Q26
        QMPYL   ACC,XT,ACC      ; ACC=((((K1*x+K2)*x+K3)*x+K4)*x+K5)*x in Q25
        
        LSL     ACC,6
        ABS     ACC             ; Saturate to 0x7fff
        NEGTC   ACC             ; ACC=-sin(x), if TC=1
        
        MOV     AL,AH    
        CLRC 	OVM   
        LRETR
       
   