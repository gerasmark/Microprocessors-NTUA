.include "m328PBdef.inc"

main:
    rcall wait_x_msec
    jmp main

wait_x_msec:
    ldi r24,0x5E
    ldi r25, 0x01
    sbiw r24,1
    breq jump1
loop1:
    rcall wait1m
    sbiw r24,1
    brne loop1
    rcall waitm
    ret

wait4:
    ret

wait1m:
    ldi r26, 98
    
loop:
    rcall wait4
    dec r26
    brne loop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    ret

waitm:
    ldi r26, 98
    
loop2:
    rcall wait4
    dec r26
    brne loop2
    nop 
    nop
    ret
jump1:
    ldi r26, 98
    
loop3:
    rcall wait4
    dec r26
    brne loop3
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    ret



