/*
* Team Id: eYRC-841
* Author List: Harsh Lunia, Chethan M and Mahesh Joseph Sadashiv.
* Filename: buzzer_adc_func.c
* Theme: Feeder-Weeder
* Functions: unsigned char ADC_Conversion(unsigned char Ch),
             void buzzer_on (void),
			 void buzzer_off (void),
			 void buzz().
* Global Variables: unsigned char ADC_Value.
*/

#include<avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>
#include<math.h> //included to support power function

unsigned char ADC_Value; // this variable stores the analog value recorded by one of the PORTA pins

/*
  * Function Name:ADC_Conversion
  * Input: unsigned char Ch - channel value
  * Output: unsigned char - the analog value recorded 
  * Logic: This function converts the ADC signals to readable value
  * Example Call: ADC_Conversion(3)
  */

unsigned char ADC_Conversion(unsigned char Ch)
{
	unsigned char a;
	Ch = Ch & 0x07;
	ADMUX= 0x20| Ch;
	ADCSRA = ADCSRA | 0x40;	//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for ADC conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10;      //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	return a;
}

/*
  * Function Name:buzzer_on
  * Input: None
  * Output: None
  * Logic: This function sets PINC3 for switching on the buzzer
  * Example Call: buzzer_on()
  */

void buzzer_on (void)
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore | 0x08;
	PORTC = port_restore;
}

/*
  * Function Name:buzzer_off
  * Input: None
  * Output: None
  * Logic: This function clears PINC3 and turns off the buzzer
  * Example Call: buzzer_off()
  */

void buzzer_off (void)
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore & 0xF7;
	PORTC = port_restore;
}

/*
  * Function Name:buzz
  * Input: None
  * Output: None
  * Logic: This function switches ON the buzzer for 1 second and offs it
  * Example Call: buzz()
  */

void buzz()
{
	buzzer_on();
	_delay_ms(100);
	buzzer_off();
}
