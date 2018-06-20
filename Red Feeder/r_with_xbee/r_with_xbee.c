/*
 * Team Id: eYRC-841
 * 
 * Author List: Harsh Lunia, Faiz Rahman and Chethan M
 * 
 * Filename: Red_bot.c
 * 
 * Theme: Feeder-Weeder
 * 
 * Functions:  void set_path_to_99(void),  
               void reach_destination(unsigned char s1, unsigned char s2 , unsigned int ini_orient),
			   void check_path_rec(void),
			   void all_bot_ready(void),
			   void all_bot_clear(void),
			   void shiftr(int b),
			   int main(void).
			   
 *  
 * Global Variables: volatile unsigned int home_bit[3],
                     volatile unsigned int curr_pos,
					 volatile unsigned int cycle,
					 volatile unsigned int orient,
					 volatile unsigned int set,
					 volatile unsigned int set_cover,
					 volatile unsigned int event,
					 volatile unsigned int temp_node,
					 volatile unsigned int flag_switch,
                     volatile uint8_t j,
                     volatile int flag,
					 volatile int color_result,
					 volatile unsigned char data,          
					 volatile unsigned char data_rec,             
					 volatile unsigned char data_temp,           
					 volatile unsigned char data_temp_s1,         
					 volatile unsigned char data_temp_s2,         
					 volatile int8_t xbee_com,                  
					 volatile unsigned int node_count,       
					 volatile int initial_home_run[20],
					 volatile int right_plant_set1[5],          
					 volatile int left_plant_set1[5],            
					 volatile int right_plant_set2[5],          
					 volatile int left_plant_set2[5],            
					 volatile int right_plant_set1_count,      
					 volatile int left_plant_set1_count,       
					 volatile int right_plant_set2_count,      
					 volatile int left_plant_set2_count,       
					 volatile int array_index.                                   
 * 
 */



#define F_CPU 7372800
#include<avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>
#include<math.h>               //included to support power function
#include<ctype.h>              //included for toupper function

#define		THRESHOLD		95       // THRESHOLD FOR WHITE LINE SENSORS 
#define		VELOCITY_MAX	60       // VELOCITY_MAX , VELOCITY_MIN , VELOCITY_LOW ARE MACROS FOR DIFFERENT DC-MOTOR SPEEDS
#define		VELOCITY_MIN	20
#define 	VELOCITY_LOW	0

//general variables

volatile unsigned int home_bit[3]={0,0,0};   // [0]-Self(spark 1) , [1]-firebird V, [2]- spark 2;  these are variable which will ensure whether all bots have completed the present cycle or whether they have completed their Dijkstra generated path traversal
volatile unsigned int curr_pos;              // the current node 
volatile unsigned int cycle=0;               // cycle:0-for initial home run ; cycle:1-for set0 and first navigation
                                             // cycle:2-going to specific nodes and doing the job for set 1 contains navigation to next cycle 
                                             // cycle:3-going to specific nodes and doing the job for set 2 
											 // cycle:4- final home run ; cycle:4-return from main
volatile unsigned int orient;                // orient variable will keep track of present orientation of the bot
                                             // orient:0- NORTH ; orient:1- EAST ; orient:2- SOUTH ; orient:3- WEST  
volatile unsigned int set=0;                 // keeps track of the current set 
volatile unsigned int set_cover=0;           // set_cover gives percentages of completion of set: 0%-0 ; 50%-50 ; 100%-100
volatile unsigned int event=0;               // this variable helps to distinguish between different task within a cycle 
volatile unsigned int temp_node;             // a general variable used for different and diverse task
volatile unsigned int flag_switch=0;         // used to control switch cases  
volatile uint8_t j;                          // loop variable 
volatile int flag;                           // a if-else control variable 
volatile int color_result;                   // to record the color of the plant
volatile int start_flag;
//----------------------------

//Xbee variables

volatile unsigned char data=0x00;            // stores the value to be transmitted over xbee eventually
volatile unsigned char data_rec;             // stores the received value; assigned to UDR resgister
volatile unsigned char data_temp;            // a general variable used for different and diverse task concerning xbee
volatile unsigned char data_temp_s1;         // stores row in which the bot is currently traversing 
volatile unsigned char data_temp_s2;         // stores column in which the bot is currently traversing 
volatile int8_t xbee_com=0;                  // variable to differentiate between different actions performed on receiving data over xbee

//-----------------------------

//Dijkstra path variables

volatile unsigned int node_count=0;         // will keep track of number of nodes to be traversed for reaching to their first set start position
volatile int initial_home_run[20];                   // will hold the Dijkstra calculated path sent to bot by firebird over the xbee

//-------------------------------

//Plant block location

volatile int right_plant_set1[5];           // stores the position of  red plants on the right in set1 
volatile int left_plant_set1[5];            // stores the position of  red plants on the left in set1  
volatile int right_plant_set2[5];           // stores the position of  red plants on the right in set2 
volatile int left_plant_set2[5];            // stores the position of  red plants on the left in set2
volatile int right_plant_set1_count=0;      // stores number of red plants on right in set1
volatile int left_plant_set1_count=0;       // stores number of red plants on left in set1
volatile int right_plant_set2_count=0;      // stores number of red plants on right in set2
volatile int left_plant_set2_count=0;       // stores number of red plants on left in set2
volatile int array_index=0;                 // a general variable for accessing values at different indices of any array
volatile int blue_xbee_flag = 0;
volatile int green_xbee_flag = 0;
//---------------------------------

#include "pin_config_func.c"               //Please check the file
#include "buzzer_adc_func.c"               //Please check the file                         
#include "motor_func.c"                    //Please check the file 
#include "ir_prox.c"                       //Please check the file
#include "color_sensor.c"                  //Please check the file 
#include "xbee.c"                          //Please check the file
#include "rgb_led.c"                       //Please check the file
#include "servo_motor.c"                   //Please check the file  
#include "sharp_sensor.c"
/*
  * Function Name:set_path_to_99
  * Input: None
  * Output: None
  * Logic: This function initializes initial_home_run[] array to 99.
  * Example Call: set_path_to_99()
  */

void set_path_to_99()
{   
	for(j=0;j<20;++j)
	initial_home_run[j]=99;
}

/*
  * Function Name: reach_destination
  * Input: None
  * Output: None
  * Logic: This function is used to convert the row-column address of the bot to sequential node-wise address and stores this and orientation.
           It also transfers the the starting position to firebird V via xbee module so that firebird can calculate its Dijkstra path right at start and send the same back to it. 
  * Example Call: reach_destination(4,'A',0)
  */

void reach_destination(unsigned char s1, unsigned char s2 , unsigned int ini_orient)
{  
    set_path_to_99();
	
	orient=ini_orient;
	curr_pos = convert_rowcol_to_seq(s1,s2);
	
	xbee_seq_node_transfer(curr_pos,0);
}

/*
  * Function Name:check_path_rec
  * Input: None
  * Output: None
  * Logic: This function checks whether the red bot has received whole of Dijkstra calculated path from firebird V or not.
  * Example Call: check_path_rec()
  */

void check_path_rec()
{
	if((initial_home_run[node_count-1]==27) && (home_bit[0]==0))
	{
		UDR = 0x40;
		home_bit[0]=1;
	}
}

/*
  * Function Name:all_bot_ready
  * Input: None
  * Output: None
  * Logic: This function  checks whether all the bots have finished their set/cycle/navigation and returns 1 or 0 accordingly
  * Example Call: all_bot_ready()
  */ 

int all_bot_ready()
{
 	if((home_bit[0]==1) && (home_bit[1]==1) && (home_bit[2]==1)) return 1; 
	return 0;
}

/*
  * Function Name:all_bot_clear
  * Input: None
  * Output: None
  * Logic: This function is used to reset home_bit[] array once all the bots have finished the set/cycle/navigation .
  * Example Call: all_bot_clear()
  */

void all_bot_clear()
{
   home_bit[0]=0;
   home_bit[1]=0;
   home_bit[2]=0;	
}
void all_xbee_clear()
{
	green_xbee_flag = 0;
	blue_xbee_flag = 0;
}

/*
  * Function Name:shiftr
  * Input: int b
  * Output: None
  * Logic: This function is used to shift the elements of initiial_home_run[] array to introduce the position behind the start position of the bot
  * Example Call: shiftr(26)
  */ 

void shiftr(int b)
{
    int i,n;
    i=node_count;
    n=node_count-1;
    while(i>0)
    {  
		initial_home_run[i] = initial_home_run[n] ;
		--n;
		--i;
	}		
    initial_home_run[0]=b ;
}

/*
 * Function Name:main
 * Input: None
 * Output: None
 * Logic: This function is called right at start of program execution.
          It contains events loop. 
 * Example Call: N.A
*/

/*
NOTE:
 
 cycle 0: This cycle deals with Dijkstra path receiving from firebird V and consequently traversal of the bot 
          based on the path received.
          
		  event 0:This event continually checks for full Dijkstra path. If found to be received it increments event variable. 
		  event 1:This event calls the run_home function which sets the bot on the Dijkstra path. 
		  event 2:This event waits for all bots to have reached the starting nodes, i.e. 13 or 27 or 41, and 
		          upon being confirmed increments cycle and sets event to 0. 
 
 cycle 1: This cycle deals with scanning the whole set blocks and finding the plant locations. 
          If the plant is of interest the bot will do the job, otherwise it will transmit the location to the concerned bot.
          
		  event 0:Detects the presence of a plant by repeatedly calling the ir_check() function, if found goes for event1 
		          otherwise calls lie follow function for this set.
		  event 1:Checks the color of the plant once the bot has turned 90 degrees.
		          This is followed by the task if the plant is of interest otherwise the location is communicated.
		  event 2:Checks if all the bots have completed set0. On confirmation starts navigation to next set.
		          After navigation checks if all have reached the next set starting position. On confirmation increments cycle.
 
 cycle 2: In this cycle since the bot knows the location of all the interested plants, it straight away goes to their location and does the job.
          If their are no plants of interest in the set the bot will simply go to other end of the set and come back to the start position of set, i.e. 35 and 41 respectively. 
 		  
		  event 0:In this event the bot travels to the interested plant location if their are so, 
		          otherwise it simply goes and comes back to start position. 
 		  event 1:Here the bot glows the RGB LED and dispenses the fertilizers.
 		  event 2:This event is executed only upon being confirmed of all bots arrival at the start position of their respective sets after completely traversing the same. 
                  On being so, navigation to last set is started. 
				  Once navigation is complete the bot again waits arrival of the other bots to start of their respective last sets.
 cycle 3: In this cycle since the bot knows the location of all the interested plants, it straight away goes to their location and does the job.
          If their are no plants of interest in the set the bot will simply go to other end of the set and come back to the start position of set, i.e. 7 and 13 respectively.
          
		  event 0:In this event the bot travels to the interested plant location if their are so,
                  otherwise it simply goes and comes back to start position.
          event 1:Here the bot glows the RGB LED and dispenses the fertilizers.
          event 2:This event is executed only upon being confirmed of all bots arrival at the start position of their respective sets after completely traversing the same.
                  On being so, final home run is executed.
                 
 cycle 4: Run for the home location is started by calling final_home_run() function.

 cycle 5: This cycle marks the end of task.		  		  

*/

void main()
{
   init_devices();
   _delay_ms(250);
  reach_destination('5','G',3);
  Servo_Value=Convert_Angle(0);
  _delay_ms(75);
  while(1)
  {
	  
	  if(cycle == 0)
	  {
		  if(event == 0)
		  {
			  check_path_rec();
			  
			  if(all_bot_ready() == 1)
			  {
				  all_bot_clear();
				  all_xbee_clear();
				  xbee_com = -1;
				  event++;
			  }
		  }
		  if(event == 1)
		  {
			  run_home();
			  event = 2;
			  cycle = 0;
		  }
		  if(event == 2)
		  {
			  if(all_bot_ready() == 1)
			  {
				  cycle = 1;
				  event = 0;
				  all_bot_clear();
				  xbee_com = 1;
			  }
		  }
		  
	  }
	  if(cycle == 1)
	  {
		  if(event == 0)
		  {
			  forward_mm(115);
			  stop();
			  
			  
			  if(sharp_check() == 1)
			  {
				  event = 1;
				  forward_mm(40);
				  stop();
				  _delay_ms(200);
				  
			  }
			  else
			  event = 2;
		  }
		  if(event == 1)
		  {
			  color_check();
			  if(color_result == 0)
			  {
				  glow_red();
				  feedOut();
				  _delay_ms(2000);
				  rgb_off();
			  }
			  else
			  {
				  if(color_result == 1)
				  {
					  glow_green();
					  xbee_seq_node_transfer(curr_pos, 1);
					  _delay_ms(2000);
					  rgb_off();
				  }
				  else if(color_result == 2)
				  {
					  glow_blue();
					  xbee_seq_node_transfer(curr_pos, 2);
					  _delay_ms(2000);
					  rgb_off();
				  }
			  }
			  event = 2;
		  }
		  if(event == 2)
		  {
			  line_follow_set0();
		  }
		  if(event == 3)
		  {
			  xbee_com = 1;
			  set_cover = 50;
			  if(all_bot_ready()==1)
			  {
				  all_bot_clear();
				  cycle = 2;
				  navigate_0();
				  stop();
				  UDR = 0x40;
				  _delay_ms(100);
				  UDR = 0x40;
				  _delay_ms(100);
				  UDR = 0x40;
				  home_bit[0] = 1;
				  while(all_bot_ready() == 0);
				  all_bot_clear();
				  start_flag = 1;
			  }
		  }
	  }
	  else if(cycle == 2)
	  {
		  if(event==0)
		  {
			  if(set_cover==0)
			  {
				  if(right_plant_set1_count==0)line_follow_till_1(35);
				  else
				  {
					  line_follow_till_1(right_plant_set1[array_index]);
				  }
				  
			  }
			  else if(set_cover==50)
			  {
				  if(left_plant_set1_count==0)line_follow_till_1(41);
				  else
				  {
					  line_follow_till_1(left_plant_set1[array_index]);
				  }
			  }
		  }
		  else if(event==1)
		  {
			  glow_red();
			  feedOut();
			  rgb_off();
			  event=0;
		      if(set_cover==0)                 {--right_plant_set1_count;  ++array_index; }
	          else if(set_cover==50)           {--left_plant_set1_count;   ++array_index; }
          }
  else if(event == 2)       // will wait for all bots to complete the whole set and hence start navigation to next step
  {
	  if(all_bot_ready()==1)
	  {
		  all_bot_clear();
		  navigate_1();
		  ++set;
		  set_cover=0;
		  array_index=0;
		  event=0;
		  ++cycle;
		  stop();
		  UDR = 0x40;
		  _delay_ms(100);
		  UDR = 0x40;
		  _delay_ms(100);
		  UDR = 0x40;
		  home_bit[0] = 1;
		  while(all_bot_ready() == 0);
		  all_bot_clear();
		  
	  }
  }
	   }
	  else if(cycle == 3)
	  {
		  
		  if(event==0)
		  {
			  if(set_cover==0)
			  {
				  if(right_plant_set2_count==0)line_follow_till_2(7);
				  else
				  {
					  line_follow_till_2(right_plant_set2[array_index]);
				  }
				  
			  }
			  else if(set_cover==50)
			  {
				  if(left_plant_set2_count==0)line_follow_till_2(13);
				  else
				  {
					  line_follow_till_2(left_plant_set2[array_index]);
				  }
			  }
		  }
		  else if(event==1)
		  {
			  glow_red();
			  feedOut();
			  rgb_off();
			  event=0;
		      if(set_cover==0)                 {--right_plant_set2_count;  ++array_index; }
	          else if(set_cover==50)           {--left_plant_set2_count;   ++array_index; }
          }
  else if(event==2)
  {
	  if(all_bot_ready()==1)
	  {
		  all_bot_clear();
		  ++set;
		  set_cover=0;
		  array_index=0;
		  event=0;
		  ++cycle;
	  }
  }
	   }
	   else if(cycle==4)   // final home run begins
	   {
		   final_home_run();
		   ++cycle;
	   }
	   else if(cycle==5)  // come out of event loop
	   {
		   buzzer_on();
		   _delay_ms(6000);
		    buzzer_off();
		   break;
	   }
  }	
}