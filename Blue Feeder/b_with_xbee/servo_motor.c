/*
* Team Id: eYRC-841
* Author List: Chethan M
* Filename: color_sensor.c
* Theme: Feeder-Weeder
* Functions: void feedOut(),
             unsigned char  Convert_Angle(unsigned char  k),
			 ISR(_VECTOR(9))        --      [Interrupt Service Routine for servo motor].
 
* Global Variables: volatile unsigned char Timer_Count;
                    unsigned char Servo_Angle,
                    unsigned char Servo_Value,
                    unsigned char k,
                    volatile unsigned char count.
*/

#define F_CPU 7372800
#include <avr/io.h>
#include <avr/delay.h>
#include <util/delay.h>
#include <avr/interrupt.h>

volatile unsigned char Timer_Count;  // Counts the number of timer overflows 
unsigned char Servo_Angle    = 10;   // the mapped angle value by which we want to rotate the servo motor
unsigned char Servo_Value    = 0;    // stores the number of timer overflows required for servo motor to rotate by specific angle
unsigned char k              = 0;    // a variable used in calculating servo value


unsigned char  Convert_Angle(unsigned char  k);

/*
  * Function Name:feedOut
  * Input: None 
  * Output: None
  * Logic: This function operates the servo motor for dispensing fertilizer. 
  * Example Call: feedOut()
  */

void feedOut()
{
	Servo_Value=Convert_Angle(100);
	_delay_ms(75);
	Servo_Value=Convert_Angle(0);
	_delay_ms(400);

}

/*
  * Function Name:ISR
  * Input: _VECTOR(9)
  * Output: None
  * Logic: This interrupt service routine is called on timer overflow.
  * Example Call: N.A
  */

ISR(_VECTOR(9))
{
	TIFR=0X01;
	Timer_Count++;

	if( Timer_Count < 125 )
	TCNT0=0XEB;

	if( Timer_Count == Servo_Value )
	PORTD &=~(1<<7);

	if( Timer_Count >= 125 )
	{
		TCNT0=0X97;
		TCCR0=0X04;
	}

	if( Timer_Count == 130 )
	{
		PORTD = 0X80;
		TCCR0 = 0X02;
		TCNT0 = 0XEB;
		Timer_Count = 0;
	}
}

/*
  * Function Name: Convert_Angle
  * Input: unsigned char  k -- mapped angle value 
  * Output: unsigned char   -- the required number of timer overflow
  * Logic: This function calculates the number of timer overflows required to rotate the servo motor by required angle. 
  * Example Call: Convert_Angle(180)
  */

unsigned char  Convert_Angle(unsigned char  k)
{
	unsigned char timer_value;
	int temp;
	temp = k*5;
	timer_value = temp/9;
	timer_value = timer_value+25;
	_delay_ms(3);
	return timer_value;
}
