.include "m328PBdef.inc"

ldi r24 ,(1<<WGM10) | (1<<COM1A1) 
sts TCCR1A, r24
ldi r24 ,(1<<WGM12) | (1<<CS11) 
sts TCCR1B, r24
ser r24
out DDRB,r24
ldi ZH,HIGH(Array *2)
ldi ZL,LOW(Array *2) 
adiw zl, 6
lpm 
mov r18,r0
ldi r21,0x00
sts OCR1AH,r21
sts OCR1AL,r18
loop:
    in r17, PIND
    com r17
    cpi r17,0x00
    breq loop
delay:
    ldi r24, low(5*16)
    ldi r25, high(5*16)
    rcall delay_mS
    in r20, PIND
    com r20
    andi r20,0x06
    cpi r20, 0x00
    brne delay

    mov r16,r17
    andi r17, 0x02
    cpi r17, 0x02
    breq raise
    andi r16, 0x04
    cpi r16, 0x04
    breq lower
    rjmp loop

raise:
    cpi r18,0xFB
    breq loop
    sbiw zl,1
    lpm 
    mov r18,r0
    ldi r21,0x00
    sts OCR1AH,r21
    sts OCR1AL,r18
    rjmp loop
    
lower:
    cpi r18,0x1A
    breq loop
    adiw zl,1
    lpm 
    mov r18,r0
    ldi r21,0x00
    sts OCR1AH,r21
    sts OCR1AL,r18
    rjmp loop  



Array:
.DW 0xE6FB, 0xBDD2, 0x94A9, 0x6C80
.DW 0x4357, 0x332E, 0x001A

delay_mS:
    
    ldi r23, 249
loop_inn:
    dec r23
    nop
    brne loop_inn
    
    sbiw r24, 1
    brne delay_mS
    
    ret