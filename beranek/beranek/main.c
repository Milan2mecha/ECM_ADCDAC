/*
 * beranek.c
 *
 * Created: 02.12.2021 20:06:57
 * Author : Milan
 vývody od LSB PB2-7, PD0, PD2-PD4 (D2-D8, D-10-13)
 PWM vývod PD1 (D9)
 vypnutí SMA pøi A3 = 0
 ADC ext reference pøi A4 = 0
 DAC na I2C MCP4726 - 12bit (byl by lepší MCP4716 10bit ale nemám ho)
 */ 
#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/twi.h>
#include "util/delay.h"
#include <compat/twi.h>

#define F_SCL 400000L

volatile uint16_t ADCout[8];
uint8_t i = 0;
uint16_t measure = 0;
volatile uint16_t SMAsapmles [32];




void TWIbegin()
{
  //TWCR = (1<<TWEN); //povolení TWI
  TWSR = 0x00;    //bez pøedìlièky
  TWBR = ((F_CPU/F_SCL)-16)/2;    //400kHz

}
void TWIStart(void)
{
    TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN); //odešlu start
    PORTD = 0x05;
    while (!(TWCR & (1<<TWINT))); //èekám na flag odeslání
  
}
void TWIStop(void)
{
  TWCR = (1<<TWEN)|(1<<TWSTO);   //pošlu stop
}
void TWIWrite(uint8_t data)
{
  TWDR = data;
  TWCR = (1<<TWINT)|(1<<TWEN); //nastavení pro odeslání
  while (!(TWCR & (1<<TWINT))); //èekám na flag odeslání
}


uint8_t TWIGetStatus(void)
{
  uint8_t status;
  //mask status
  status = TWSR & 0xF8; //status TWI
  return status;
}
void adcRead(uint8_t mux) {
  uint8_t ADCHD = 0;
  uint8_t ADCLD = 0;
  ADMUX = mux;                                          // mux
  if (PINC & 0x08)
  {
      ADMUX |= (1 << REFS0);                                // AVcc reference
  } 
  else
  {
    ADMUX &= ~(1 << REFS0);                                // AVcc reference
  }
  ADMUX |= (1 << ADLAR);                                // zarovnat vlevo
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // 128 pøeddìlièka pro nižší šum
  ADCSRA |= (1 << ADEN);                                // zapnutí
  ADCSRA |= (1 << ADSC); // start
  while(ADCSRA & (1 << ADSC));                          // tu se to sekne
  ADCHD = ADCH;
  ADCLD = ADCL;
  ADCout[mux] = (ADCH*0x04) + (ADCLD/0x40);
}

void adcMeaBegin(uint8_t mux)
{
  //výchozích 32 hodnot pro klouzavý prùmìr
  for(uint8_t z = 0; z<32; z++)
  {
    adcRead(mux);
    SMAsapmles[z]=ADCout[mux];
  }
}
uint16_t adcMeasure(uint8_t mux)
{
  //Klouzavý prùmìr

  uint16_t vystup = 0;
  uint32_t SMA = 0;
  adcRead(mux);
  SMAsapmles[i] = ADCout[mux];
  for(uint8_t z = 0; z<32; z++)
  {
    SMA += SMAsapmles[z];
  }
  i++;
  if(i>32)
  {
    i=0;
  }
  //vypnutí SMA PB4 na GND
  if (PINC & 0x04)
  {
    vystup = SMA/32;
  }
  else
  {
    vystup = SMAsapmles[i-1];
  }
  return vystup;
}
void PWMinit()
{
  TCCR1A |= (1 << WGM11); //fast pwm
  TCCR1B |= (1 << WGM12);  //fast pwm
  TCCR1B |= (1 << WGM13); //fast pwm
  TCCR1A |= (1<<COM1A1); //non-inverting
  TCCR1B |= (1<<CS10);  //clock bez prescaleru
  ICR1 = 0x03ff; //10-bit rozsah
}
void writeDAC(uint16_t data){

  TWIStart();
 if (TWIGetStatus() != 0x08)
  {
    PORTD = 0x05;
  }
  TWIWrite(0b11000010);
  
  if (TWIGetStatus() != 0x18)
  {
    PORTD = 0x05;
  }
  TWIWrite(0x40);
  if (TWIGetStatus() != 0x28)
  {
    PORTD = 0x05;
  }
  TWIWrite(data>>2);
  if (TWIGetStatus() != 0x28)
  {
    PORTD = 0x05;
  }
  TWIWrite((data<<6)&0xF0);
  if (TWIGetStatus() != 0x28)
  {
    PORTD = 0x05;
  }
  TWIStop();
}
int main(void)
{
  DDRD = 0xFC;
  DDRB = 0x0F;
  PORTC = 0xCF;
  TWIbegin();
  adcMeaBegin(0);
  PWMinit();
    while (1) 
    {
    measure = adcMeasure(0);
    PORTD = (measure << 2);
    PORTB = (((measure >> 6) & 0x01) | ((measure >> 5) & 0x1C));
    OCR1A = measure;
    writeDAC(measure);
    }
}