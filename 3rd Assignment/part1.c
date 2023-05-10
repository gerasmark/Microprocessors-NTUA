#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

ISR (INT1_vect){
    EIFR= (1 << INTF1);
    _delay_ms(5);
    while (EIFR==1) {
        EIFR= (1 << INTF1);
    _delay_ms(5);
    }

    sei();
     if (PORTB==0x01){
        TCNT1=57722;
        PORTB=0xFF;
        while(PORTB==0xFF);
        TCNT1=10847;  
        PORTB=0x01;
        return;
    } 
    else{
        TCNT1=3035;
        PORTB=0x01;
        return;
        }

}

ISR (TIMER1_OVF_vect){ 
    PORTB=0x00;
    
}


int main(void) {
    DDRB=0xFF;
    EICRA=(1<< ISC11)|(1<< ISC10);
    EIMSK=(1<<INT1);
    TIMSK1=(1<<TOIE1);
    TCCR1B=(1<<CS12) | (0<<CS11) | (1<<CS10);
    sei();
    PORTB=0x00;
    
   while(1){ 

       if(PINC==0x5F){
        while(PINC!=0x7F)_delay_ms(5);
        if (PORTB==0x01){
            TCNT1=57722;
            PORTB=0xFF; 
            while (PORTB==0xFF);
            TCNT1=10847;
            PORTB=0x01; }
        else {
            TCNT1=3035;
            PORTB=0x01;
        }
        }
    
               
   }   
            
}
