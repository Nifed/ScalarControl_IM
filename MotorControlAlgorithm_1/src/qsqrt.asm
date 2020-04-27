;***********************************************************************; 
;*                                                                     *;
;* File Name     : qsqrt.asm                                           *;
;*                                                                     *;
;* Originator    : Advanced Embeeded Control (AEC)                     *;
;*                 Texas Instruments Inc.                              *;
;*                                                                     *;
;* Description   : This file contain source code for fixed point SQRT  *;
;*                function implemented using Tylor series              *;
;*                                                                     *;
;***********************************************************************;
;* Tylor Series Approximation of SQRT:                                 *;
;* Algorithm     :                                                     *;
;* The following equation approximates the sqrt(x), where 0.5<= x <=1.0*;
;*  0.5*sqrt(x)= 0.7274475*x-0.672455*x^2+0.553406*x^3-0.2682495*x^4   *;
;*              +0.0560605*x^5+0.1037903                               *;
;*                                                                     *;
;* To determine the sqrt of an input value outside the range [0.5, 1.0]*;
;*  the input should be scaled to a number within the range.           *;
;***********************************************************************;

        .def _qsqrt

a0      .set    06a48h          ; 0.1037903  scaled by 2^18
a1      .set    05d1dh          ; 0.7274475 scaled by 2^15 
a2      .set    0a9edh          ; -0.672455 scaled by 2^15
a3      .set    046d6h          ; 0.553406 scaled by 2^15
a4      .set    0bb54h          ; -0.2682495 scaled by 2^16 
a5      .set    00e5ah          ; 0.0560605 scaled by 2^16 

SQRT2  .set    05a82h           ;(1/sqrt(2)) in Q15 format
                                ; Also sqrt(2) in Q14 format

_qsqrt
                                    ; ACC=X in Q16 format
            SETC    SXM             ; Set the sign ext. mode
            MPY     P,T,#0          ; P=0 
            LSR64   ACC:P,#1        ; X=X/2

            CSB     ACC             ; Count sign bits, T=E
            LSL64   ACC:P,T         ; ACC=x=X/(2^(E-15) in  Q31 format

            TBIT    @T,#0           ; TC=odd/even shift indicator
            MOV     AR4,T           ; AH=n             
            MOVL    XT,ACC          ; T=x in Q31
            MOV     AH,AR4          ; AH=n
            LSR     AH,#1           ; AH=n/2
            MOV     AR4,AH          ; AR4=n/2
   
            MPY     ACC,T,#a5       ; ACC=x*a5 in Q31 
            ADD     ACC,#a4<<15     ; ACC=a4+x*a5 in Q31
 
            QMPYL   ACC,XT,@ACC     ; ACC=x*(a4+x*a5) in Q30
            ADD     ACC, #a3<<15    ; ACC=a3+x*(a4+x*a5) in Q30
            
            QMPYL   ACC,XT,@ACC     ; ACC=x*(a3+(a4+x*a5)) in Q29
            ADD     ACC, #a2<<14    ; ACC=a2+x*(a3+(a4+x*a5)) in Q29
            
            QMPYL   ACC,XT,@ACC     ; ACC=x*(a2+x*(a3+(a4+x*a5))) in Q28
            ADD     ACC, #a1<<13    ; ACC=a1+x*(a2+x*(a3+(a4+x*a5))) in Q28 
            
            QMPYL   ACC,XT,@ACC     ; ACC=x*(a1+x*(a2+x*(a3+(a4+x*a5)))) in Q27
            ADD     ACC, #a0<<9     ; ACC=a0+x*(a1+x*(a2+x*(a3+(a4+x*a5)))) in Q27
                                    ; ACC=0.5sqrt(s*x) in Q27
                                    ; ACC=sqrt(s*x) in Q26

;*********** De-normalise the result ****************************   
        
            MOVH    T,ACC<<5        ; ACC=sqrt(s*x) in Q15
            MPY     P,T,#SQRT2      ; P=sqrt(s*x)*(1/sqrt(2)) in Q30 format
                                    
            LSL     ACC,#4          ; ACC=sqrt(s*x) in Q30  
            MOVL    P,ACC,NTC       ; P=sqrt(s*x) in Q30, if n is odd
            MOVL    ACC,P

            MOV     T,AR4
            LSRL    ACC,T           ; ACC=sqrt(x) in Q30
            MOVH    AL,ACC<<2       ; AL=sqrt(x)    
            LRETR
                
            
                                                                                                                                            
                                

            


            

            
            
            
            
            
            
            
            
            
            
            
            
                                                                                    
    



