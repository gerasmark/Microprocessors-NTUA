.include "m328PBdef.inc"
.def A=r16
.def B=r17
.def C=r18
.def D=r19
.def temp=r20
.def f0=r21
.def f1=r22
.def count=r23

ldi A,0x55
ldi B,0x43
ldi C,0x22
ldi D,0x02
ldi count,0x06

start:
    mov f0,A
    com f0
    mov temp,B
    com temp
    and f0,temp
    and temp,D
    or f0,temp
    com f0
    mov f1,A
    or f1,C
    mov temp,D
    com temp
    or temp,B
    and f1,temp
    ldi temp,0x02
    add A,temp
    inc temp
    add B,temp
    inc temp
    add C,temp
    inc temp
    add D,temp
    dec count
    brne start



