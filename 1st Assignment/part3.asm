.include "m328PBdef.inc"
.def temp=r16
.def led=r17
.def data=r18
.def position=r19

    clr temp
    out DDRD, temp 
    ldi led,0x01
    set
    clr position
    out PORTD , led
    jmp left
left1:
    set
    rcall wait_x_msec
    rcall wait_x_msec
left:
    inc position
    rcall wait_x_msec
    lsl led
    out PORTD , led
    cpi position , 7
    breq right1
    brts left

right1:
    clt
    rcall wait_x_msec
    rcall wait_x_msec

right:
    dec position
    rcall wait_x_msec
    lsr led
    out PORTD , led
    cpi position , 0
    breq left1
    brtc right

wait_x_msec:
    ldi r24,0xD0
    ldi r25, 0x07
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







