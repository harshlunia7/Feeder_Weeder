/*
* Team Id: eYRC-841
* Author List: Harsh Lunia, Chethan M, Mahesh Joseph Sadashiv and Faiz Rahman.
* Filename: pin_config_func.c
* Theme: Feeder-Weeder
* Functions: void color_sensor_pin_config(void),
             void adc_pin_config (void),
			 void timer1_init(void),
			 void set_servo(),
			 void adc_init(),
			 void motion_pin_config (void),
			 void uart0_init(),
			 void left_encoder_pin_config(void),
			 void right_encoder_pin_config(void),
			 void buzzer_pin_config (void),
			 void right_position_encoder_interrupt_init(void),
			 void left_position_encoder_interrupt_init(void),
			 void port_init(void),
			 void init_devices (void),
			 void rgb_led_pin_config(void).
			 
*/

#include <avr/io.h>
#include <avr/delay.h>
#include <util/delay.h>
#include<avr/interrupt.h>

/*
  * Function Name:rgb_led_pin_config
  * Input: None
  * Output: None
  * Logic: This function sets up the  pins of the bot for configuring RGB LED.
           PINC 0-2 are for red, green and blue leads of RGB LED respectively. 
  * Example Call: rgb_led_pin_config()
  */

void rgb_led_pin_config(void)
{
	DDRC = DDRC | 0x07;
	PORTC = PORTC & 0xF8;
}

/*
  * Function Name:color_sensor_pin_config
  * Input: None
  * Output: None
  * Logic: This function sets up the  pins of the bot for configuring color sensor and it to send data.
           PINC 4-7 are for S0 - S3
		   PIND6 is for OUT 
  * Example Call: color_sensor_pin_config()
  */ 

void color_sensor_pin_config(void)
{
	DDRC  = DDRC | 0xFF; // CONFIGURING PINC 4-7 PINS AS OUTPUT FOR S0,S1,S2,S3.
	PORTC = PORTC | 0x00;// INITIALLY SET TO ZERO AS IN FIREBIRD V CODE
	DDRD= DDRD | 0x32; // PIND6 CONFIGURED AS OUT OF COLOR SENSOR WHILE OTHERS ARE KEPT AT THEIR DEFAULT VALUES
}

/*
  * Function Name:adc_pin_config
  * Input: None
  * Output: None
  * Logic: This function sets up the PINs of PORTA for analog reading.
  * Example Call: adc_pin_config()
  */

void adc_pin_config (void)
{
	DDRA = 0x00;   //set PORTA direction as input
	PORTA = 0x00;  //set PORTA pins floating
}

/*
  * Function Name:timer1_init
  * Input: None
  * Output: None
  * Logic: This function sets up timer1 - 16 bit for PWM generation.
  * Example Call: timer1_init()
  */

void timer1_init(void)
{
	TCCR1B = 0x00; //stop
	TCNT1H = 0xFF; //setup
	TCNT1L = 0x01;
	OCR1AH = 0x00;
	OCR1AL = 0xFF;
	OCR1BH = 0x00;
	OCR1BL = 0xFF;
	ICR1H  = 0x00;
	ICR1L  = 0xFF;
	TCCR1A = 0xA1;
	TCCR1B = 0x0D; //start Timer
}

/*
  * Function Name:set_servo
  * Input: None
  * Output: None
  * Logic: This function sets up the  pins of the bot for configuring servo motor and it also sets timer0 - 8bit for operating servo motor.
           PIND 7 for Servo control.
  * Example Call: set_servo()
  */ 

void set_servo()
{
	DDRD  = DDRD | 0X80;
	TCCR0 = 0X02;
	TCNT0 = 0XEB;
	TIMSK = 0X01;
}

/*
  * Function Name:adc_init
  * Input: None
  * Output: None
  * Logic: This function sets up for analog to digital conversion.
  * Example Call: adc_init()
  */

void adc_init()
{
	ADCSRA = 0x00;
	ADMUX = 0x20;
	ACSR = 0x80;
	ADCSRA = 0x86;
}

/*
  * Function Name:motion_pin_config
  * Input: None
  * Output: None
  * Logic: This function sets up motion pins and enables velocity control using PWM 
  * Example Call: motion_pin_config()
  */ 

void motion_pin_config (void)
{
	DDRB = DDRB | 0x0F;   //set direction of the PORTB3 to PORTB0 pins as output
	PORTB = PORTB & 0xF0; // set initial value of the PORTB3 to PORTB0 pins to logic 0
	DDRD = DDRD | 0x30;   //Setting PD4 and PD5 pins as output for PWM generation
	PORTD = PORTD | 0x30; //PD4 and PD5 pins are for velocity control using PWM
}

/*
  * Function Name:uart0_init
  * Input: None
  * Output: None
  * Logic: This function initializes  the UART  register of the bot  for xbee communication.
  * Example Call: uart0_init()
  */

void uart0_init()
{
	UCSRB = 0x00; //disable while setting baud rate
	UCSRA = 0x00;
	UCSRC = 0x86;
	UBRRL = 0x2F; //set baud rate lo  //67 is for 16MHz 9600 baudrate
	UBRRH = 0x00; //set baud rate hi
	UCSRB = 0x98;
}

/*
* Function Name:left_encoder_pin_config
* Input: None
* Output: None
* Logic: This function sets up pins for enabling left position encoder.
* Example Call: left_encoder_pin_config()
*/

 void left_encoder_pin_config(void)
{
	DDRD = DDRD & 0xFB;
	PORTD = PORTD | 0x04;
}

/*
* Function Name:right_encoder_pin_config
* Input: None
* Output: None
* Logic: This function sets up pins for enabling right position encoder.
* Example Call: right_encoder_pin_config()
*/

void right_encoder_pin_config(void)
 {
	 DDRD = DDRD & 0xF7;
	 PORTD = PORTD | 0x08;
 }


/*
  * Function Name:buzzer_pin_config
  * Input: None
  * Output: None
  * Logic: This function sets up pins for enabling buzzer 
  * Example Call: buzzer_pin_config()
  */ 

void buzzer_pin_config (void)
{
	DDRC = DDRC | 0x08;		//Setting PORTC 3 as output
	PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}

/*
* Function Name:right_position_encoder_interrupt_init
* Input: None
* Output: None
* Logic: This function sets up pins for enabling interrupt for right position encoder.
* Example Call: right_position_encoder_interrupt_init()
*/

void right_position_encoder_interrupt_init(void)
{
	MCUCR = MCUCR | 0x08;
	GICR = GICR | 0x80;
}

/*
* Function Name:left_position_encoder_interrupt_init
* Input: None
* Output: None
* Logic: This function sets up pins for enabling interrupt for left position encoder.
* Example Call: right_position_encoder_interrupt_init()
*/

void left_position_encoder_interrupt_init(void)
{
	MCUCR = MCUCR | 0x02;
	GICR = GICR | 0x40;
}

void lcd_port_config (void)
{
	DDRC = DDRC | 0xF7;    //all the LCD pin's direction set as output
	PORTC = PORTC & 0x80;  // all the LCD pins are set to logic 0 except PORTC 7
}

/*
  * Function Name:port_init
  * Input: None
  * Output: None
  * Logic: This function calls various pin configuring functions of different devices/functions on bot.
  * Example Call: port_init()
  */

void port_init(void)
{
	 motion_pin_config();
	 buzzer_pin_config();
	 uart0_init();
	 left_encoder_pin_config();
	 right_encoder_pin_config();
	 adc_pin_config();
	 color_sensor_pin_config();
	 rgb_led_pin_config();
}

/*
  * Function Name:init_devices
  * Input: None
  * Output: None
  * Logic: This function initializes almost all the ports/devices necessary for the bot to execute the program
  * Example Call: init_devices()
  */ 

void init_devices (void)
{
	cli(); //Clears the global interrupts
	port_init();
	timer1_init();
	adc_init();
	left_position_encoder_interrupt_init();
	right_position_encoder_interrupt_init();
	set_servo();
	sei(); //Enables the global interrupts
}

