#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

static volatile float adc;
static volatile uint8_t adc1;
static volatile uint8_t adc2;
static volatile uint8_t adc3;

void Write_2_Nibbles(uint8_t in) {
    uint8_t temp = in;
    uint8_t p = PIND;
    p &= 0x0F;
    in &= 0xF0;
    in |= p;
    PORTD = in;
    PORTD |= 0x08;
    PORTD &= 0xF7;
    
    in = temp;
    in &= 0x0F;
    in = in << 4;
    in |= p;
    PORTD = in;
    PORTD |= 0x08;
    PORTD &= 0xF7;
    
    return;
}

void LCD_data(uint8_t c) {
    PORTD |= 0x04;
    Write_2_Nibbles(c);
    _delay_us(100);
    return;
}

void LCD_command(uint8_t c) {
    PORTD &= 0xFB;
    Write_2_Nibbles(c);
    _delay_us(100);
    return;
}

void LCD_init(void) {
    _delay_ms(40);
    
    PORTD = 0x30;
    PORTD |= 0x08;
    PORTD &= 0xF7;
    _delay_us(100);
    
    PORTD = 0x30;
    PORTD |= 0x08;
    PORTD &= 0xF7;
    _delay_us(100);
    
    PORTD = 0x20;
    PORTD |= 0x08;
    PORTD &= 0xF7;
    _delay_us(100);
    
    LCD_command(0x28);
    LCD_command(0x0C);
    LCD_command(0x01);
    _delay_us(5000);
    
    LCD_command(0x06);
}



int main(void) {
    float decimal;
    TCCR1A = (0<<WGM10) | (1<<WGM11) |(1<<COM1A1) ;
    TCCR1B = (1<<WGM12) | (1<<CS11)|(1<<WGM13);
    DDRB=0b00111111;
    OCR1AL = 0x80;
    DDRD = 0xFF;
    DDRC = 0x00;
    ADMUX = (1 << REFS0) | (1 << MUX1);
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
    ADCSRB = 0x00;
    DIDR0 = ~(1 << ADC2D);
    while(1) {
        ICR1=0x0000;
        if((PINB&0b00000100)==0){
            while((PINB&0b00000100)==0);
                ICR1=399;
                OCR1A =ICR1*0.2;   
                ADCSRA |= (1 << ADSC);
                while ((ADCSRA & (1 << ADSC)) == (1 << ADSC));

                adc = ADC;
                adc = (adc * 5) / 1024;

                adc1 = (uint8_t)adc;
                decimal = adc - adc1;
                adc2 = (uint8_t)(decimal * 10);
                adc3 = (uint8_t)(((decimal * 10) - adc2) * 10); 


                adc1 |= 0x30;
                adc2 |= 0x30;
                adc3 |= 0x30;

                LCD_init();
                _delay_ms(2);

                LCD_data('2');
                LCD_data('0');
                LCD_data('%');
                LCD_command(0b11000000);
                LCD_data(adc1);
                LCD_data('.');
                LCD_data(adc2);
                LCD_data(adc3);

        
        
        
        _delay_ms(100);
                
            }
        else if ((PINB&0b00001000)==0){
            while((PINB&0b00001000)==0);
                ICR1=399;  
                OCR1A =ICR1*0.4;
                ADCSRA |= (1 << ADSC);
                while ((ADCSRA & (1 << ADSC)) == (1 << ADSC));

                adc = ADC;
                adc = (adc * 5) / 1024;

                adc1 = (uint8_t)adc;
                decimal = adc - adc1;
                adc2 = (uint8_t)(decimal * 10);
                adc3 = (uint8_t)(((decimal * 10) - adc2) * 10); 


                adc1 |= 0x30;
                adc2 |= 0x30;
                adc3 |= 0x30;

                LCD_init();
                _delay_ms(2);

                LCD_data('4');
                LCD_data('0');
                LCD_data('%');
                LCD_command(0b11000000);
                LCD_data(adc1);
                LCD_data('.');
                LCD_data(adc2);
                LCD_data(adc3);
            }
        else if ((PINB&0b00010000)==0){
            while((PINB&0b0010000)==0);
                ICR1=399; 
                OCR1A =ICR1*0.6;
                ADCSRA |= (1 << ADSC);
                while ((ADCSRA & (1 << ADSC)) == (1 << ADSC));

                adc = ADC;
                adc = (adc * 5) / 1024;

                adc1 = (uint8_t)adc;
                decimal = adc - adc1;
                adc2 = (uint8_t)(decimal * 10);
                adc3 = (uint8_t)(((decimal * 10) - adc2) * 10); 


                adc1 |= 0x30;
                adc2 |= 0x30;
                adc3 |= 0x30;

                LCD_init();
                _delay_ms(2);

                LCD_data('6');
                LCD_data('0');
                LCD_data('%');
                LCD_command(0b11000000);
                LCD_data(adc1);
                LCD_data('.');
                LCD_data(adc2);
                LCD_data(adc3);
            }
        else if ((PINB&0b00100000)==0){
            while((PINB&0b00100000)==0);
                ICR1=399; 
                OCR1A =ICR1*0.8;
                ADCSRA |= (1 << ADSC);
                while ((ADCSRA & (1 << ADSC)) == (1 << ADSC));

                adc = ADC;
                adc = (adc * 5) / 1024;

                adc1 = (uint8_t)adc;
                decimal = adc - adc1;
                adc2 = (uint8_t)(decimal * 10);
                adc3 = (uint8_t)(((decimal * 10) - adc2) * 10); 


                adc1 |= 0x30;
                adc2 |= 0x30;
                adc3 |= 0x30;

                LCD_init();
                _delay_ms(2);

                LCD_data('8');
                LCD_data('0');
                LCD_data('%');
                LCD_command(0b11000000);
                LCD_data(adc1);
                LCD_data('.');
                LCD_data(adc2);
                LCD_data(adc3);
            }
        }
    
}
