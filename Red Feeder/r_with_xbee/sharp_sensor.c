#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <math.h> //included to support power function
#define F_CPU 7372800
unsigned char sharp=0;
unsigned int value;

unsigned int Sharp_GP2D12_estimation(unsigned char adc_reading)
{
	float distance;
	unsigned int distanceInt;
	distance = (int)(10.00*(2799.6*(1.00/(pow(adc_reading,1.1546)))));
	distanceInt = (int)distance;
	if(distanceInt>800)
	{
		distanceInt=800;
	}
	return distanceInt;
}

unsigned int sharp_check(void)
{
    sharp=ADC_Conversion(7);
    value= Sharp_GP2D12_estimation(sharp);
	_delay_ms(300);
	if(value<150)
	 {
	   return 1;
	 }
	 return 0;
}

