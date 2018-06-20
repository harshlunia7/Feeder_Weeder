/*
* Team Id: eYRC-841
* Author List: Harsh Lunia
* Filename: color_sensor.c
* Theme: Feeder-Weeder
* Functions: void timer1_init_color(void),
             ISR(TIMER1_CAPT_vect)        --      [Interrupt Service Routine for timer1 input capture mode],
			 void filter_red(void),
			 void filter_green(void),
			 void filter_blue(void),
			 void filter_clear(void),
			 void color_sensor_scaling_20(void),
			 void color_sensor_scaling_2(void),
			 void red_read(void),
			 void green_read(void),
			 void blue_read(void),
			 void init_color_sensor(void),
             void color_check(void).	
 
* Global Variables: volatile unsigned long int pulse,
                    volatile unsigned long int red,
					volatile unsigned long int blue,
					volatile unsigned long int green.
*/

#define F_CPU 7372800
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h> //included to support power function

volatile unsigned long int pulse = 0; //to keep the track of the number of pulses generated by the color sensor
volatile unsigned long int red;       // variable to store the pulse count when read_red function is called
volatile unsigned long int blue;      // variable to store the pulse count when read_blue function is called
volatile unsigned long int green;     // variable to store the pulse count when read_green function is called

/*
  * Function Name:timer1_init_color
  * Input: None
  * Output: None
  * Logic: Initialize pins for enabling timer1 in input capture mode
  * Example Call: timer1_init_color()
  */ 

void timer1_init_color(void)
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0xE3; //setup
 TCNT1L = 0xE1;
 OCR1AH = 0x1C;
 OCR1AL = 0x1F;
 OCR1BH = 0x1C;
 OCR1BL = 0x1F;
 ICR1H  = 0x1C;
 ICR1L  = 0x1F;
 TCCR1A = 0x00;
 TCCR1B = 0x05; //start Timer, INTERRUPT FOR COLOR SENSOR CONFIGURED FOR FALLING EDGE THROUGH BIT6 OF TCCR1B (ICES1 BIT)
}

/*
  * Function Name:ISR
  * Input: TIMER1_CAPT_vect
  * Output: None
  * Logic: Increment the pulse count on receiving pulse from the color sensor
  */ 

ISR(TIMER1_CAPT_vect)// INPUT CAPTURE ROUTINE
{
	pulse++; //increment on receiving pulse from the color sensor
}

 /*
  * Function Name:filter_red
  * Input: None
  * Output: None
  * Logic: Initialize the pins for reading pulses corresponding to red color
  * Example Call: filter_red()
  */ 

void filter_red(void)    //Used to select red filter
{
	//Filter Select - red filter
	PORTC = PORTC & 0xBF; //set S2 low
	PORTC = PORTC & 0x7F; //set S3 low
}

 /*
  * Function Name:filter_green()
  * Input: None
  * Output: None
  * Logic: Initialize the pins for reading pulses corresponding to green color
  * Example Call: filter_green()
  */ 

void filter_green(void)	//Used to select green filter
{
	//Filter Select - green filter
	PORTC = PORTC | 0x40; //set S2 High
	PORTC = PORTC | 0x80; //set S3 High
}

/*
  * Function Name:filter_blue()
  * Input: None
  * Output: None
  * Logic: Initialize the pins for reading pulses corresponding to blue color
  * Example Call: filter_blue()
  */ 

void filter_blue(void)	//Used to select blue filter
{
	//Filter Select - blue filter
	PORTC = PORTC & 0xBF; //set S2 low
	PORTC = PORTC | 0x80; //set S3 High
}

/*
  * Function Name:filter_clear()
  * Input: None
  * Output: None
  * Logic: Initialize the pins for no specific filter
  * Example Call: filter_blue()
  */ 

void filter_clear(void)	//select no filter
{
	//Filter Select - no filter
	PORTC = PORTC | 0x40; //set S2 High
	PORTC = PORTC & 0x7F; //set S3 Low
}

/*
  * Function Name:color_sensor_scaling_20
  * Input: None
  * Output: None
  * Logic: This function is used to select the scaled down version of the original frequency of the output generated by the color sensor(20%)
  * Example Call: color_sensor_scaling_20()
  */

void color_sensor_scaling_20()		
{
	//Output Scaling 20% from datasheet
	PORTC = PORTC | 0x10; //set S0 high
	PORTC = PORTC & 0xDF; //set S1 LOW
}

/*
  * Function Name:color_sensor_scaling_2
  * Input: None
  * Output: None
  * Logic: This function is used to select the scaled down version of the original frequency of the output generated by the color sensor(2%)
  * Example Call: color_sensor_scaling_2()
  */

void color_sensor_scaling_2()	
{
	//Output Scaling 2% from datasheet
	PORTC = PORTC & 0xEF; //set S0 low
	PORTC = PORTC | 0x20; //set S1 high
}

/*
  * Function Name:red_read
  * Input: None
  * Output: None
  * Logic: This function is used to select red filter 
           and calculate the corresponding pulse count generated,
           which will be more if the color is red. It will be very less if its blue or green.
  * Example Call: red_read()
  */

void red_read(void) // function to select red filter and display the count generated by the sensor on LCD. The count will be more if the color is red. The count will be very less if its blue or green.
{
	//Red
	filter_red(); //select red filter
	pulse=0; //reset the count to 0
	_delay_ms(100); //capture the pulses for 100 ms or 0.1 second
	red = pulse;  //store the count in variable called red
}

/*
  * Function Name:green_read
  * Input: None
  * Output: None
  * Logic: This function is used to select green filter 
           and calculate the corresponding pulse count generated,
           which will be more if the color is green. It will be very less if its red or blue.
  * Example Call: green_read()
  */

void green_read(void) // function to select green filter and display the count generated by the sensor on LCD. The count will be more if the color is green. The count will be very less if its blue or red.
{
	//Green
	filter_green(); //select green filter
	pulse=0; //reset the count to 0
	_delay_ms(100); //capture the pulses for 100 ms or 0.1 second
	green = pulse;  //store the count in variable called green
}

  /*
  * Function Name:blue_read
  * Input: None
  * Output: None
  * Logic: This function is used to select blue filter 
           and calculate the corresponding pulse count generated,
           which will be more if the color is blue. It will be very less if its red or green.
  * Example Call: blue_read()
  */ 

void blue_read(void) // function to select blue filter and display the count generated by the sensor on LCD. The count will be more if the color is blue. The count will be very less if its red or green.
{
	//Blue
	filter_blue(); //select blue filter
	pulse=0; //reset the count to 0
	_delay_ms(100); //capture the pulses for 100 ms or 0.1 second
	blue = pulse;  //store the count in variable called blue
}

/*
  * Function Name:init_color_sensor
  * Input: None
  * Output: None
  * Logic: Initialize pins for enabling input capture interrupt and set scaling for color sensor
  * Example Call: init_color_sensor()
  */ 

void init_color_sensor(void)
{
 cli();          //Clears the global interrupts
 timer1_init_color();
 TIMSK = TIMSK | 0x20;   //timer1 interrupt sources
 sei();          //Enables the global interrupts
 color_sensor_scaling_2();
}


/*
  * Function Name:color_check
  * Input: None
  * Output: None
  * Logic: Select a color's filter and read the corresponding pulses, compare the different pulses and decide the color
  * Example Call: color_check()
  */ 

void color_check(void)
{
	init_color_sensor();
	
	red_read(); 
	_delay_ms(500);
	green_read(); 
	_delay_ms(500);
	blue_read(); 
	_delay_ms(500);
	
	cli();
	timer1_init();
	sei();
	
	if((red>green) && (red>blue))  color_result = 0;
	else if(green>blue)            color_result = 1;
	else                           color_result = 2;
}



