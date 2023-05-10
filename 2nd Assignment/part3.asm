.include "m328PBdef.inc"
    
.equ FOSC_MHZ=16
    
.equ DEL_mS=500
    
.equ DEL_NU= FOSC_MHZ*DEL_mS ;to interrupt se petaei se shmeia tou kwdika analoga th timh tou pb0.

.org 0x0
rjmp reset
    
.org 0x4
rjmp ISR1

  reset:
	ldi r23,(1<< ISC11)|(1<<ISC10)
	sts EICRA, r23
    
	ldi r23, (1<<INT1)
	out EIMSK, r23
    
	sei

    ser r26
    out DDRB, r26    
    ldi r24, LOW(RAMEND)
   out SPL, r24
   ldi r24, HIGH (RAMEND)
   out SPH, r24
    clt
main:   
   rjmp main 

ISR1:
    
    brts all_lights
    rjmp lights_on    
      
all_lights:
    ser r20
    out PORTB, r20
    ldi r24, low(16*500)
    ldi r25, high(16*500)
    rcall delay_mS
    ldi r20, 0x01
    out PORTB, r20
    ldi r24, low(16*3500)
    ldi r25, high(16*3500)
    rcall delay_mS
    clr r20
    out PORTB, r20
    clt
    reti
 
lights_on:
    set
    ldi r20, 0x01
    out PORTB, r20
    ldi r24, low(16*4000)
    ldi r25, high(16*4000)
    rcall delay_mS
    clr r20
    out PORTB, r20
    clt
    reti

    
delay_mS:  
    sei
    ldi r23, 249
loop_inn:
    dec r23
    nop
    brne loop_inn
    
    sbiw r24, 1
    brne delay_mS
    
    ret
    