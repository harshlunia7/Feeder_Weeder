/*
* Team Id: eYRC-841
* Author List: Harsh Lunia and Chethan M
* Filename: xbee.c
* Theme: Feeder-Weeder
* Functions: unsigned int convert_rowcol_to_seq(unsigned char x1,unsigned char x2),
             ISR(USART_RXC_vect)        --      [Interrupt Service Routine for xbee],
			 void xbee_seq_node_transfer(unsigned int node_tx, unsigned int send_flag).
*/

/*

NOTE:
Xbee communication has been used either to notify a completion of a certain event/cycle by the host bot or
to send position data.

The Xbee uses following identity code for notifying its completion of an event/cycle :-
1. Firebird V (green weeder bot ) - 0x00 
2. Spark V (red feeder bot) - 0x40
3. Spark V (blue feeder bot) - 0x80

To send current position of the bot via xbee the following logic has been used :-

Xbee in AT mode transfers 8 bits in one go. We have broke 8 bits in 3 sets as follows:

example : UDR = 0b B7 B6 B5 B4 B3 B2 B1 B0

1. B7 and B6 are identity bits, i.e.,
        B7   B6
		 0    0  for firebird V Green weeder bot 
		 0    1  for spark V red  feeder bot 
		 1    0  for spark V blue feeder bot.
2. B5, B4 and B3 are for column, it will have following values :-
      col    B5  B4  B3
	   A      0   0   1
	   B      0   1   0
	   C      0   1   1
	   D      1   0   0
	   E      1   0   1
	   F      1   1   0
	   G      1   1   1
3. B2, B1 and B0 are for rows, it will have following values :- 
      row   B2  B1  B0
	   1     0   0   1
	   2     0   1   0
	   3     0   1   1
	   4     1   0   0
	   5     1   0   1
	   6     1   1   0
	   7     1   1   1
*/

#include<avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>

/*
  * Function Name: convert_rowcol_to_seq
  * Input: unsigned char x1 -- row value
           unsigned char x2 -- column value
  * Output: sequential address of the node passed as parameter
  * Logic: This function calculates the sequential address of a node whose col/row address hs been passed.
  * Example Call: convert_rowcol_to_seq('1','A')
  */
unsigned int checker;

unsigned int convert_rowcol_to_seq(unsigned char x1,unsigned char x2)
{
	unsigned int x;
	x=0;
	if((x1 == 0x02) || (x1 == '2'))
	{
		x+=7;
	}
	else if((x1 == 0x03) || (x1 == '3'))
	{
		x+=14;
	}
	else if((x1 == 0x04) || (x1 == '4'))
	{
		x+=21;
	}
	else if((x1 == 0x05) || (x1 == '5'))
	{
		x+=28;
	}
	else if((x1 == 0x06) || (x1 == '6'))
	{
		x+=35;
	}
	else if((x1 == 0x07) || (x1 == '7'))
	{
		x+=42;
	}
	if((x2 == 0x10) || (x2 == 'B'))
	{
		x+=1;
	}
	else if((x2 == 0x18) || (x2 == 'C'))
	{
		x+=2;
	}
	else if((x2 == 0x20) || (x2 == 'D'))
	{
		x+=3;
	}
	else if((x2 == 0x28) || (x2 == 'E'))
	{
		x+=4;
	}
	else if((x2 == 0x30) || (x2 == 'F'))
	{
		x+=5;
	}
	else if((x2 == 0x38) || (x2 == 'G'))
	{
		x+=6;
	}
	return x;
}

/*
  * Function Name:ISR
  * Input:USART_RXC_vect
  * Output: None
  * Logic: This interrupt service routine is called on receiving data over xbee.
           It makes use of xbee_com variable to differentiate between different after-communication process to follow.
  * Example Call: N.A
  */

/* 
NOTE:

1. xbee_com == 0
This process receives the Dijkstra calculated path from firebird V and stores the same path in array called initial_home_run.   

2. xbee_com == 1
This process is executed when set0 is in progress.
It receives red plant locations in other two unexplored sets and stores the same location in different arrays.
The above arrays are used by bot to traverse directly to the plant in the other two sets instead of scanning every plant location.

*/

ISR(USART_RXC_vect)
{
	data_rec = UDR;
	if((data_rec == 0x00) && (green_xbee_flag == 0))
	{
		home_bit[1]=1;
		green_xbee_flag = 1;
	}		
	if((data_rec == 0x40) && (red_xbee_flag == 0)) 
	{
		home_bit[2]=1;
		red_xbee_flag = 1;
	}
	if(xbee_com==0)
	{
		data_temp = data_rec & 0xC0;
		
		if(data_temp == 0x80)
		{  
			data_temp_s1 = data_rec & 0x07;
			data_temp_s2 = data_rec & 0x38;
			initial_home_run[node_count]=convert_rowcol_to_seq(data_temp_s1,data_temp_s2);
			++node_count;
		}
	}
	else if(xbee_com==1)
	{
		data_temp = data_rec & 0xC0;
		uint8_t temp;
		
		if(data_temp == 0x80)
		{
			data_temp_s1 = data_rec & 0x07;
			data_temp_s2 = data_rec & 0x38;
			temp=convert_rowcol_to_seq(data_temp_s1,data_temp_s2);
			if(set_cover==0)
			{
				if((temp>6) && (temp<14))
				{
					right_plant_set1[right_plant_set1_count]=temp;
					++right_plant_set1_count;
				}
				else
				{
					right_plant_set2[right_plant_set2_count]=temp;
					++right_plant_set2_count;
				}
			}
			else
			{
				if(set_cover==50)
				{
					if((temp>6) && (temp<14))
					{
						left_plant_set1[left_plant_set1_count]=temp;
						++left_plant_set1_count;
					}
					else
					{
						left_plant_set2[left_plant_set2_count]=temp;
						++left_plant_set2_count;
					}
				}
			}
		}
	}
}

/*
  * Function Name: xbee_seq_node_transfer
  * Input: unsigned int node_tx   -- sequential address to be transferred over xbee
           unsigned int send_flag -- A flag for selecting which bot to transfer to : 0 for red bot ; 1 for green bot; 2 for blue bot.
  * Output: None
  * Logic: This function sends the data containing the sequential node position to the respective bot over xbee.
  * Example Call: xbee_seq_node_transfer(22 , 0)
  */

void xbee_seq_node_transfer(unsigned int node_tx, unsigned int send_flag)
{
	data = 0x00;
	    if(node_tx < 7)
		{
			flag_switch = 0;
			switch(node_tx)
			{
				case 0: data = 0x09; flag_switch = 1;
				case 1: if(flag_switch==0)
				{
					data = 0x11;
					flag_switch=1;
				}
				case 2:if(flag_switch==0)
				{
					data = 0x19;
					flag_switch=1;
				}
				case 3:if(flag_switch==0)
				{
					data = 0x21;
					flag_switch=1;
				}
				case 4:if(flag_switch==0)
				{
					data = 0x29;
					flag_switch=1;
				}
				case 5:if(flag_switch==0)
				{
					data = 0x31;
					flag_switch=1;
				}
				case 6:if(flag_switch==0)
				{
					data = 0x39;
					flag_switch=1;
				}
			}
		}
		else
		{
				flag_switch=0;
				switch(node_tx%7)
				{
					case 0:data = data | 0x08; flag_switch=1;
					case 1:if(flag_switch==0)
					{
						data = data | 0x10;
						flag_switch=1;
					}
					case 2:if(flag_switch==0)
					{
						data = data | 0x18;
						flag_switch=1;
					}
					case 3:if(flag_switch==0)
					{
						data = data | 0x20;
						flag_switch=1;
					}
					case 4:if(flag_switch==0)
					{
						data = data | 0x28;
						flag_switch=1;
					}
					case 5:if(flag_switch==0)
					{
						data = data | 0x30;
						flag_switch=1;
					}
					case 6:if(flag_switch==0)
					{
						data = data | 0x38;
						flag_switch=1;
					}
				}
				flag_switch=0;
				switch((node_tx/7)+1)
				{
					case 1:data = data | 0x01; flag_switch=1;
					case 2:if(flag_switch==0)
					{
						data = data | 0x02;
						flag_switch=1;
					}
					case 3:if(flag_switch==0)
					{
						data = data | 0x03;
						flag_switch=1;
					}
					case 4:if(flag_switch==0)
					{
						data = data | 0x04;
						flag_switch=1;
					}
					case 5:if(flag_switch==0)
					{
						data = data | 0x05;
						flag_switch=1;
					}
					case 6:if(flag_switch==0)
					{
						data = data | 0x06;
						flag_switch=1;
					}
					case 7:if(flag_switch==0)
					{
						data = data | 0x07;
						flag_switch=1;
					}
				}
				flag_switch=0;
		}			
	    
	
        
		if(send_flag == 0)
		data = data | 0x80;
		else if(send_flag == 1)
		data = data | 0x00;
		else if(send_flag == 2)
		data = data | 0x40;

		UDR = data ;
}

