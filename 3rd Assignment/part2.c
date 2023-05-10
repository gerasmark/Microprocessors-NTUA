#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

int main(void) {
    TCCR1A = (1<<WGM10) | (1<<COM1A1) ;
    TCCR1B = (1<<WGM12) | (1<<CS11) ;
    DDRB=0b00111111;
    int data[] = { 0xFB,0xE6,0xD2, 0xBD,0xA9,0x94,0x80, 0x6C,0x57,0x43,0x33 ,0x2E,0x1A };
    int i=6;
    OCR1A = data[i];
    while(1) {
        if(PIND==0b11111101){
            while(PIND==0b11111101)_delay_ms(5);
            if (i==0);
            else {
                i--;
                OCR1A = data[i];
            }
        }
   
        else if (PIND==0b11111011){
            while(PIND==0b11111011)_delay_ms(5);
            if (i==12);
            else {
                i++;
                OCR1A = data[i];
            }
        }
    
}
}