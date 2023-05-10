.include "m328PBdef.inc"
.equ FOSC_MHZ=16

ldi r24 ,(1<<WGM11) | (0<<WGM10) | (1<<COM1A1)
sts TCCR1A, r24
ldi r24 ,(1<<WGM12) | (1<<WGM13) | (1<<CS11) ;prescale 8
sts TCCR1B, r24

ser r26
out DDRB, r26
clr r26
out DDRD, r26 
ldi r16,0x00
sts OCR1AH,r16
ldi r16,0x80
sts OCR1AL,r16  ;duty cycle 50%
    
main:   
ldi r21,0x00
sts ICR1H, r21
ldi r21,0x00
sts ICR1L, r21
in r20, PIND
com r20
cpi r20,0
breq main
cpi r20,0x01
breq d0
cpi r20,0x02
breq d1
cpi r20,0x04
breq d2
cpi r20,0x08
breq d3
    
d0:
ldi r16,0x1F
sts OCR1AH,r16
ldi r16,0x40
sts OCR1AL,r16
ldi r21,0x3E
sts ICR1H, r21
ldi r21,0x7F
sts ICR1L, r21
d00:
in r20, PIND
com r20
cpi r20,0x01
breq d00
rjmp main
    
d1:
ldi r16,0x0F
sts OCR1AH,r16
ldi r16,0xA0
sts OCR1AL,r16
ldi r21,0x1F
sts ICR1H, r21 
ldi r21,0x3F
sts ICR1L, r21
d11:
in r20, PIND
com r20
cpi r20,0x02
breq d11
rjmp main
    
d2:
ldi r16,0x07
sts OCR1AH,r16
ldi r16,0xD0
sts OCR1AL,r16
ldi r21,0x0F
sts ICR1H, r21
ldi r21,0x9F
sts ICR1L, r21    
d22:
in r20, PIND
com r20
cpi r20,0x04
breq d22
rjmp main
    
d3:
ldi r16,0x07
sts OCR1AH,r16
ldi r16,0xBE
sts OCR1AL,r16
ldi r21,0x0F
sts ICR1H, r21
ldi r21,0x7C
sts ICR1L, r21
d33:
in r20, PIND
com r20
cpi r20,0x08
breq d33
rjmp main


