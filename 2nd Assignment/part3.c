#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
ISR (INT1_vect){
    sei();
    if (PORTB==0x01){
        PORTB=0xFF;
        _delay_ms(500);
        PORTB=0x01;
        _delay_ms(3500);
        PORTB=0x00;
    
    }
    
    else{
        PORTB=0x01;
        _delay_ms(4000);
        PORTB=0x00;
        }
    reti();
}


int main(void) {
    EICRA=(1<< ISC11)|(1<< ISC10);
    EIMSK=(1<<INT1);
    sei();
    DDRB=0xFF;
    PORTB=0x00;
    while (1) {
}
}

