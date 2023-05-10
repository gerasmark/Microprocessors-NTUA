.include "m328PBdef.inc"
    
.equ FOSC_MHZ=16
    
.equ DEL_mS=500
    
.equ DEL_NU= FOSC_MHZ*DEL_mS

.org 0x0
rjmp reset
    
.org 0x2
rjmp ISR0
       
  reset:
	ldi r23,(1<< ISC01)|(1<<ISC00)
	sts EICRA, r23
    
	ldi r23, (1<<INT0)
	out EIMSK, r23
    
	sei

  
main:
   ldi r24, LOW(RAMEND)
   out SPL, r24
   ldi r24, HIGH (RAMEND)
   out SPH, r24
   ser r26
   out DDRC, r26
   clr R26
   out DDRB, R26

   
loop1:
    clr r26
loop2:
    out PORTC, r26
    
    ldi r24, low(16*600)
    ldi r25, high(16*600)
    rcall delay_mS
    
   inc r26
   
   cpi r26, 0x1F
   breq loop1
   rjmp loop2
   
delay_mS:
    
    ldi r23, 249
loop_inn:
    dec r23
    nop
    brne loop_inn
    
    sbiw r24, 1
    brne delay_mS
    
    ret
    
ISR0:
    push r23
    push r24
    in r24, SREG
    push r24 
    in r16, PINB
    com r16
    clr r26
    ldi r25,0x06
loop:
    mov r24,r16
    andi r24,0x01
    cpi r24,0x00
    breq next
    inc r26
next:
    ror r16
    dec r25
    cpi r25,0x00
    breq nextt
    jmp loop
nextt: ldi r19,0x00
lights:
    cpi r26,0
    breq end
    sec
    rol r19
    dec r26
    rjmp lights    
end:
    out PORTC,r19
    ldi r24, low(16*500)
    ldi r25, high(16*500)
    rcall delay_mS
    pop r23
    pop r24
    out SREG, r24 
    pop r24
    
    reti    
   



