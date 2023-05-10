.include "m328PBdef.inc"
    
.equ FOSC_MHZ=16
    
.equ DEL_mS=500
    
.equ DEL_NU= FOSC_MHZ*DEL_mS ;

.org 0x0
rjmp reset
.org 0x4
rjmp ISR1   
.org 0x1A
rjmp ISR_TIMER1_OVF

reset:
    ldi r23,(1<< ISC11)|(1<<ISC10)
    sts EICRA, r23
    ldi r23, (1<<INT1)
    out EIMSK, r23
    sei

    ldi r24, (1<<TOIE1) ; ???????????? ???????? ???????????? ??? ??????? TCNT1
    sts TIMSK1, r24 ; ??? ??? timer1
    ser r26
    out DDRB, r26   
    clr r26
    out DDRC, r26 
    ldi r24, LOW(RAMEND)
    out SPL, r24
    ldi r24, HIGH (RAMEND)
    out SPH, r24
    clr r16
    out PORTB,r16
    ldi r24 ,(1<<CS12) | (0<<CS11) | (1<<CS10) ; CK/1024
    sts TCCR1B, r24

check: 
    in r17, PINC
    com r17
    andi r17, 0x20
    cpi r17, 0x00
    breq check
delay:
    ldi r24, low(5*16)
    ldi r25, high(5*16)
    rcall delay_mS
    in r17, PINC
    com r17
    andi r17, 0x20
    cpi r17, 0x00
    brne delay

    in r18,PORTB
    cpi r18, 0x01
    brne first

    ldi r24, HIGH(57722) 
    sts TCNT1H, r24  
    ldi r24, LOW(57722)
    sts TCNT1L, r24
    ser r20
    out PORTB, r20
wait1:    
    in r17,PORTB
    cpi r17, 0x00
    brne wait1

    ldi r24, HIGH(10847) 
    sts TCNT1H, r24  
    ldi r24, LOW(10847)
    sts TCNT1L, r24 
    ldi r20, 0x01
    out PORTB, r20
    rjmp check

first:
    ldi r24, HIGH(3035) ; 
    sts TCNT1H, r24 ; 
    ldi r24, LOW(3035)
    sts TCNT1L, r24 
    ldi r20, 0x01
    out PORTB, r20
    rjmp check

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
    sei
    in r17,PORTB
    cpi r17, 0x01
    breq all_lights
  
lights_on:
    ldi r24, HIGH(3035)  
    sts TCNT1H, r24  
    ldi r24, LOW(3035)
    sts TCNT1L, r24 
    ldi r20, 0x01
    out PORTB, r20
    pop r23
    pop r24
    out SREG, r24 
    pop r24
    reti 
     
all_lights:
    ldi r24, HIGH(57722) 
    sts TCNT1H, r24  
    ldi r24, LOW(57722)
    sts TCNT1L, r24
    ser r20
    out PORTB, r20
wait2:    
    in r17,PORTB
    cpi r17, 0x00
    brne wait2

    ldi r24, HIGH(10847) 
    sts TCNT1H, r24  
    ldi r24, LOW(10847)
    sts TCNT1L, r24 
    ldi r20, 0x01
    out PORTB, r20
    pop r23
    pop r24
    out SREG, r24 
    pop r24
    reti

ISR_TIMER1_OVF:
    clr r20
    out PORTB, r20
    reti


delay_mS:
    
    ldi r23, 249
loop_inn:
    dec r23
    nop
    brne loop_inn
    
    sbiw r24, 1
    brne delay_mS
    
    ret



