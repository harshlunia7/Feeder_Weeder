/*
* Team Id: eYRC-841
* Author List: Harsh Lunia
* Filename: ir_prox.c
* Theme: Feeder-Weeder
* Functions: int ir_check(void).
* Global Variables: unsigned char flag2,
                    unsigned char Front_IR_Sensor.
*/ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define		IR_THRESHOLD_MAX  105
#define     IR_THRESHOLD_MIN  100


unsigned char flag2 = 0;          // this variable just keeps a check of whether the object has been detected or not.
unsigned char Side_IR_Sensor=0;  // this variable stores the Analog value recorded for side IR-sensor 

/*
  * Function Name:ir_check
  * Input: None
  * Output: int - confirmation of whether the object is beside the bot or not.
  * Logic: This function records the value registered by side ir sensors and if the same is within a pre-calculated range, a confirmation of plant on the side will be passed back.
  * Example Call: ir_check()
  */

void ir_check(void)
{
	Side_IR_Sensor = ADC_Conversion(1);    //Getting data of Side IR Proximity Sensor Sensor

	if((Side_IR_Sensor > IR_THRESHOLD_MIN) && (Side_IR_Sensor < IR_THRESHOLD_MAX))
	{
		stop();
		_delay_ms(4000);
		forward();
	}
}
