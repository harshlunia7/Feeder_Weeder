/*
* Team Id: eYRC-841
* Author List: Harsh Lunia 
* Filename: rgb_led.c
* Theme: Feeder-Weeder
* Functions: void glow_red(),
             void glow_green(),
			 void glow_blue(),
			 void rgb_off(),
			 void glow(unsigned int rgb_color).
*/
#define F_CPU 7372800
#include <avr/io.h>
#include <avr/delay.h>
#include <util/delay.h>

/*
  * Function Name:glow_red
  * Input: None
  * Output: None
  * Logic: This function sets PINC0 connected to red pin of RGB led and switches it on
  * Example Call: glow_red()
  */

void glow_red()
{
	unsigned char PortCRestore = 0;
	PortCRestore = PORTC ;
	PortCRestore = PortCRestore & 0xF8;
	PortCRestore = PortCRestore | 0x01;
	PORTC = PortCRestore;
}

/*
  * Function Name:glow_green
  * Input: None
  * Output: None
  * Logic: This function sets PINC1 connected to green pin of RGB led and switches it on
  * Example Call: glow_green()
  */

void glow_green()
{
	unsigned char PortCRestore = 0;
	PortCRestore = PORTC ;
	PortCRestore = PortCRestore & 0xF8;
	PortCRestore = PortCRestore | 0x02;
	PORTC = PortCRestore;
}

/*
  * Function Name:glow_blue
  * Input: None
  * Output: None
  * Logic: This function sets PINC2 connected to blue pin of RGB led and switches it on
  * Example Call: glow_blue()
  */

void glow_blue()
{
	unsigned char PortCRestore = 0;
	PortCRestore = PORTC ;
	PortCRestore = PortCRestore & 0xF8;
	PortCRestore = PortCRestore | 0x04;
	PORTC = PortCRestore;
}

/*
  * Function Name:rgb_off
  * Input: None
  * Output: None
  * Logic: This function clears PINC 0-2 connected to red, blue and green pins of RGB led and switches all the of them off
  * Example Call: rgb_off()
  */

void rgb_off()
{
	unsigned char PortCRestore = 0;
	PortCRestore = PORTC ;
	PortCRestore = PortCRestore & 0xF8;
	PORTC = PortCRestore;
}

/*
  * Function Name:glow
  * Input: unsigned int -- code for color 
  * Output: None
  * Logic: This function takes in value to decide which color to glow in RGB LED for 1 sec; 0 - red, 1 - green, 2 - blue; 
  * Example Call: glow(1)
  */

void glow(unsigned int rgb_color)
{
	if(rgb_color == 0) 
	{
		glow_red();
		_delay_ms(1000);
		rgb_off();
	}
	else if(rgb_color == 1)
	{
		glow_green();
		_delay_ms(1000);
		rgb_off();
	}
	else if(rgb_color == 2)
	{
		glow_blue();
		_delay_ms(1000);
		rgb_off();
	}
}