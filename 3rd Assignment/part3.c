#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

int main(void) {
    TCCR1A = (0<<WGM10) | (1<<WGM11) |(1<<COM1A1) ;
    TCCR1B = (1<<WGM12) | (1<<CS11)|(1<<WGM13);
    DDRB=0b00111111;
    OCR1AL = 0x80;
    while(1) {
        ICR1=0x0000;
        if((PIND&0b00000001)==0){
            while((PIND&0b00000001)==0)
                ICR1=0x3E7F;
                OCR1A =ICR1 /2;        
            }
        else if ((PIND&0b00000010)==0){
            while((PIND&0b00000010)==0)
                ICR1=0x1F3F;  
                OCR1A =ICR1 /2;
            }
        else if ((PIND&0b00000100)==0){
            while((PIND&0b00000100)==0)
                ICR1=0x0F9F; 
                OCR1A =ICR1 /2;
            }
        else if ((PIND&0b00001000)==0){
            while((PIND&0b00001000)==0)
                ICR1=0x0F7C; 
                OCR1A =ICR1 /2;
            }
        }
    
}
