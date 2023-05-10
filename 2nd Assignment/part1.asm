.include "m328PBdef.inc"
    
.equ FOSC_MHZ=16
    
.equ DEL_mS=500
    
.equ DEL_NU= FOSC_MHZ*DEL_mS

.org 0x0
rjmp reset
    
.org 0x4
rjmp ISR1
      
  ;not ours   

  reset:
	ldi r23,(1<< ISC11)|(1<<ISC10)
	sts EICRA, r23
    
	ldi r23, (1<<INT1)
	out EIMSK, r23
    
	sei

  
main:
   ldi r24, LOW(RAMEND)
   out SPL, r24
   ldi r24, HIGH (RAMEND)
   out SPH, r24
   
   ser r26
   out DDRB, r26
   ser r28
   out DDRC, r28
   clr r20
   out PORTC, r20

   
loop1:
    clr r26
loop2:
    out PORTB, r26
    
    ldi r24, low(500*16)
    ldi r25, high(500*16)
    rcall delay_mS
    
   inc r26
   
   cpi r26, 16
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
    
ISR1:
     
     
    
    push r23
    push r24
    in r24, SREG
    push r24
start: 
	ldi r16, (1 << INTF1)
	out EIFR, r16
	ldi r24, low(5*16)
	ldi r25, high(5*16)
	rcall delay_mS

	in r16, EIFR
	cpi r16,0
	brne start
    rjmp routine
    
    
  routine: 
    in r23, PIND
    com r23
    andi r23, 0x80
    cpi r23, 0x00
    brne endint
    in r24, PORTC
    cpi r24, 0x1F
    breq int_full
    inc r24
    out PORTC, r24
   endint:
    pop r23
    pop r24
    out SREG, r24 
    pop r24
    
    reti
    
  int_full:
	 
    ldi r23, 0x00
    out PORTC, r23
    pop r23
    pop r24
    out SREG, r24 
    pop r24
    
    reti    
