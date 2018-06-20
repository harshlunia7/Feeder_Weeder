/*
* Team Id: eYRC-841
* Author List: Harsh Lunia, Chethan M and Mahesh Joseph Sadashiv
* Filename: motor_func.c
* Theme: Feeder-Weeder
* Functions: void run_home(void),
             void final_home_run(void),
			 void navigate_1(void),
			 void navigate_0(void),
			 void line_follow_till_2(unsigned int req_pos),
			 void line_follow_till_1(unsigned int req_pos),
			 void line_follow_set0(void),
			 void velocity (unsigned char left_motor, unsigned char right_motor),
			 void motion_set (unsigned char Direction),
			 ISR(INT1_vect),
			 ISR(INT0_vect),
			 void linear_distance_mm(unsigned int DistanceInMM),
			 void angle_rotate(unsigned int deg),
			 void forward (void),
			 void back (void),
			 void left (void),
			 void right (void),
			 void soft_left (void),
			 void soft_right (void),
			 void soft_left_2 (void),
			 void soft_right_2 (void),
			 void stop (void),
			 void soft_stop (void),
			 void forward_mm(unsigned int DistanceInMM),
			 void left_degrees(unsigned int Degrees),
			 void right_degrees(unsigned int Degrees),
			 void turn(unsigned int direction),
			 void ensure_orient(unsigned int req_orient),
			 void update_curr_pos().
			  
* Global Variables: float req_shaft_count,
                    unsigned int req_shaft_count_int,
                    volatile unsigned int shaft_count_right,
                    volatile unsigned long int shaft_count_left,
                    volatile float Kp,
                    volatile float Kd,
                    unsigned char rightBaseSpeed,
                    unsigned char leftBaseSpeed,
                    volatile unsigned int Left_white_line,
                    volatile unsigned int Center_white_line,
                    volatile unsigned int Right_white_line,
                    volatile int error,
                    unsigned char lastError.
*/

#include<avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>
#include<math.h> //included to support power function

#define		THRESHOLD		95       // THRESHOLD FOR WHITE LINE SENSORS
#define		VELOCITY_MAX	60       // VELOCITY_MAX , VELOCITY_MIN , VELOCITY_LOW ARE MACROS FOR DIFFERENT DC-MOTOR SPEEDS
#define		VELOCITY_MIN	20
#define 	VELOCITY_LOW	0
/*#define     rightMaxSpeed   40
#define     leftMaxSpeed    40
*/
float req_shaft_count=0;                        // this variable will store the calculated value of number of pulses required to move the bot by a particular distance or rotate it by a specific angle 
unsigned int req_shaft_count_int=0;             // this variable will store the integer part of calculated value of number of pulses required to move the bot by a particular distance or rotate it by a specific angle 
volatile unsigned int shaft_count_right=0;      // this variable will store the number of pulses already registered by right position encoder   
volatile unsigned long int shaft_count_left=0;  // this variable will store the number of pulses already registered by left position encoder  

volatile float Kp = 0.7;
volatile float Kd = 0.75;
#define rightMaxSpeed 45// max speed of the robot
#define leftMaxSpeed 45 // max speed of the robot
unsigned char rightBaseSpeed=45;// this is the speed at which the motors should spin when the robot is perfectly on the line
unsigned char leftBaseSpeed=45;  // this is the speed
volatile int lastError=0;
volatile int error;
unsigned char ADC_Conversion(unsigned char);
//unsigned char ADC_Value;
unsigned char Left_white_line = 0;
unsigned char Center_white_line = 0;
unsigned char Right_white_line = 0;
 /*
  * Function Name:run_home
  * Input: None
  * Output: None
  * Logic: This function is used for following the Dijkstra path sent over xbee by firebird V.
  * Example Call: run_home()
  */

void run_home()
{
	int temp;
	if(orient==0)
	temp=initial_home_run[0]+7;
	else if(orient==1)
	temp=initial_home_run[0]-1;
	else if(orient==2)
	temp=initial_home_run[0]-7;
	else if(orient==3)
	temp=initial_home_run[0]+1;
	
	shiftr(temp);
	curr_pos = temp;
	
	int prev,present,next,m;
	prev=initial_home_run[0];
	present=initial_home_run[1];
	next=initial_home_run[2];
	m=1;
	
	/*
	update_curr_pos();
	
	//velocity(40,40);
	//_delay_ms(300);
	//just_forward();
	stop();
	buzz();
	
	
	if(next-present==1)
	{
		if(present-prev==1)
		//forward();
		just_forward();
		if(present-prev==7)
		//turn(0);
		turn_left();
		if(present-prev==-7)
		//turn(1);
		turn_right();
		if(present-prev==-1)
		{
			//turn(1);
			//turn(1);
			turn_reverse();
		}
		
		m++;
		prev=initial_home_run[m-1];
		present=initial_home_run[m];
		next=initial_home_run[m+1];
	}
	
	else if(next-present==7)
	{
		if(present-prev==7)
		//forward();
		just_forward();
		if(present-prev==-1)
		//turn(0);
		turn_left();
		if(present-prev==1)
		//turn(1);
		turn_right();
		if(present-prev==-7)
		{
			//turn(1);
			//turn(1);
			turn_reverse();
		}
		
		m++;
		prev=initial_home_run[m-1];
		present=initial_home_run[m];
		next=initial_home_run[m+1];
	}
	
	else if(next-present==-7)
	{
		if(present-prev==-7)
		//forward();
		just_forward();
		if(present-prev==1)
		//turn(0);
		turn_left();
		if(present-prev==-1)
		//turn(1);
		turn_right();
		if(present-prev==7)
		{
			//turn(1);
			//turn(1);
			turn_reverse();
		}
		
		m=m+1;
		prev=initial_home_run[m-1];
		present=initial_home_run[m];
		next=initial_home_run[m+1];
	}
	
	else if(next-present==-1)
	{
		if(present-prev==-1)
		//forward();
		just_forward();
		if(present-prev==-7)
		//turn(0);
		turn_left();
		if(present-prev==7)
		//turn(1);
		turn_right();
		if(present-prev==1)
		{
			//turn(1);
			//turn(1);
			turn_reverse();
		}

		m=m+1;
		prev=initial_home_run[m-1];
		present=initial_home_run[m];
		next=initial_home_run[m+1];
	}*/
	
	
	while(initial_home_run[m]!=99)
	{
		Left_white_line = ADC_Conversion(3);
		Center_white_line = ADC_Conversion(4);
		Right_white_line = ADC_Conversion(5);
		error = Right_white_line - Left_white_line;
		int motorSpeed = Kp * error + Kd * (error - lastError);
		lastError = error;
		int rightMotorSpeed = rightBaseSpeed - motorSpeed;
		int leftMotorSpeed = leftBaseSpeed + motorSpeed;
		if (rightMotorSpeed > rightMaxSpeed ) rightMotorSpeed = rightMaxSpeed;
		if (leftMotorSpeed > leftMaxSpeed ) leftMotorSpeed = leftMaxSpeed;
		if (rightMotorSpeed < 0) rightMotorSpeed = 0;
		if (leftMotorSpeed < 0) leftMotorSpeed = 0;

		forward();
		velocity(leftMotorSpeed,rightMotorSpeed);

		if(Center_white_line>40 && Left_white_line>40 && Right_white_line>40)
		{
			update_curr_pos();
			
			//velocity(40,40);
			//_delay_ms(300);
			//just_forward();
			stop();
			buzz();
			
			if(next==99)
			{
				break;
			}
			
			if(next-present==1)
			{
				if(present-prev==1)
				//forward();
				just_forward();
				if(present-prev==7)
				//turn(0);
				turn_left();
				if(present-prev==-7)
				//turn(1);
				turn_right();
				if(present-prev==-1)
				{
					//turn(1);
					//turn(1);
					turn_reverse();
				}
				
				m++;
				prev=initial_home_run[m-1];
				present=initial_home_run[m];
				next=initial_home_run[m+1];
			}
			
			else if(next-present==7)
			{
				if(present-prev==7)
				//forward();
				just_forward();
				if(present-prev==-1)
				//turn(0);
				turn_left();
				if(present-prev==1)
				//turn(1);
				turn_right();
				if(present-prev==-7)
				{
					//turn(1);
					//turn(1);
					turn_reverse();
				}
				
				m++;
				prev=initial_home_run[m-1];
				present=initial_home_run[m];
				next=initial_home_run[m+1];
			}
			
			else if(next-present==-7)
			{
				if(present-prev==-7)
				//forward();
				just_forward();
				if(present-prev==1)
				//turn(0);
				turn_left();
				if(present-prev==-1)
				//turn(1);
				turn_right();
				if(present-prev==7)
				{
					//turn(1);
					//turn(1);
					turn_reverse();
				}
				
				m=m+1;
				prev=initial_home_run[m-1];
				present=initial_home_run[m];
				next=initial_home_run[m+1];
			}
			
			else if(next-present==-1)
			{
				if(present-prev==-1)
				//forward();
				just_forward();
				if(present-prev==-7)
				//turn(0);
				turn_left();
				if(present-prev==7)
				//turn(1);
				turn_right();
				if(present-prev==1)
				{
					//turn(1);
					//turn(1);
					turn_reverse();
				}

				m=m+1;
				prev=initial_home_run[m-1];
				present=initial_home_run[m];
				next=initial_home_run[m+1];
			}
			//forward();
			//velocity(20,20);
		//	_delay_ms(400);
		}
		//ir_check();
	}
	++event;
    ensure_orient(3);
	_delay_ms(500);
	UDR= 0x80;
	_delay_ms(100);
	UDR = 0x80;
	_delay_ms(100);
	UDR = 0x80;
	home_bit[0]=1;
}

void just_forward()
{
	//stop();
	//_delay_ms(500);
	forward();
	velocity(35,35);
	_delay_ms(800);
	
}
void turn_right()
{
	//stop();
	//_delay_ms(500);
	forward();
	velocity(35,35);
	_delay_ms(600);
	turn(1);
}
void turn_right_ensure()
{
	//stop();
	//_delay_ms(500);
	//forward();
	//velocity(35,35);
	//_delay_ms(600);
	turn(1);
}
void turn_left()
{
	//stop();
	//_delay_ms(500);
	forward();
	velocity(35,35);
	_delay_ms(600);
	//left();
	//_delay_ms(1700);
	turn(0);
}

void turn_reverse()
{
	//stop();
	//_delay_ms(500);
	forward();
	velocity(35,35);
	_delay_ms(600);
	//right();
	//_delay_ms(3450);
	turn(2);
}

 /*
  * Function Name:final_home_run
  * Input: None
  * Output: None
  * Logic: This function upon being called starts the robots final run for its parking at its respective home position, i.e., from node 13 to red home.
  * Example Call: final_home_run()
  */

void final_home_run(void)
{
	while(curr_pos!=13)
	{
		Left_white_line = ADC_Conversion(3);
		Center_white_line = ADC_Conversion(4);
		Right_white_line = ADC_Conversion(5);
		error = Right_white_line - Left_white_line;
		int motorSpeed = Kp * error + Kd * (error - lastError);
		lastError = error;
		int rightMotorSpeed = rightBaseSpeed - motorSpeed;
		int leftMotorSpeed = leftBaseSpeed + motorSpeed;
		if (rightMotorSpeed > rightMaxSpeed ) rightMotorSpeed = rightMaxSpeed;
		if (leftMotorSpeed > leftMaxSpeed ) leftMotorSpeed = leftMaxSpeed;
		if (rightMotorSpeed < 0) rightMotorSpeed = 0;
		if (leftMotorSpeed < 0) leftMotorSpeed = 0;

		forward();
		velocity(leftMotorSpeed,rightMotorSpeed);

		if(Center_white_line>40 && Left_white_line>40 && Right_white_line>40)
		{
			update_curr_pos();
			
		//	velocity(40,40);
	     //_delay_ms(300);
		 //just_forward();
			stop();
			buzz(); 
			 
			 if(curr_pos==13)
			 {
				// turn(1);
				turn_right();
				 forward_mm(130);
			 }
			 else
			 {
				 // forward();
				 // velocity(100,100);
				 just_forward();
				  //_delay_ms(300);
			 }
		}			
	}
}

/*
  * Function Name:navigate_1
  * Input: None
  * Output: None
  * Logic: This function upon being called makes the robot traverse path for changing set1 to set2, i.e, from node 41 to node 13.
  * Example Call: navigate_1()
  */

void navigate_0(void)
{
	while(curr_pos != 13)
	{
		
		
			Left_white_line = ADC_Conversion(3);
			Center_white_line = ADC_Conversion(4);
			Right_white_line = ADC_Conversion(5);
			error = Right_white_line - Left_white_line;
			int motorSpeed = Kp * error + Kd * (error - lastError);
			lastError = error;
			int rightMotorSpeed = rightBaseSpeed - motorSpeed;
			int leftMotorSpeed = leftBaseSpeed + motorSpeed;
			if (rightMotorSpeed > rightMaxSpeed ) rightMotorSpeed = rightMaxSpeed;
			if (leftMotorSpeed > leftMaxSpeed ) leftMotorSpeed = leftMaxSpeed;
			if (rightMotorSpeed < 0) rightMotorSpeed = 0;
			if (leftMotorSpeed < 0) leftMotorSpeed = 0;

			forward();
			velocity(leftMotorSpeed,rightMotorSpeed);

			if(Center_white_line>40 && Left_white_line>40 && Right_white_line>40)
			{
				update_curr_pos();
				
				//velocity(40,40);
				//_delay_ms(300);
				//just_forward();
				stop();
				buzz();
				
				  if(curr_pos == 40)
				  {
					  all_xbee_clear();
				  }
				 if((curr_pos== 40) | (curr_pos==12)) 
				 //turn(1);
				 turn_right();
				 else if(curr_pos==13)
				 {
					 ensure_orient(3);
					 event = 0;
					 array_index = 0;
					 set_cover=0;
					 ++set;
					 xbee_com=-1;
					 event=0;
				 }
				 else
				 {
					// forward();
					//velocity(100,100);
					// _delay_ms(300);
					just_forward();
				 }
									 					 					 
			 }				 
		
	}	       
}

/*
  * Function Name:navigate_0
  * Input: None
  * Output: None
  * Logic: This function upon being called makes the robot traverse path for changing set0 to set1, i.e, from node 41 to node 13.
  * Example Call: navigate_0()
  */

void navigate_1()
{
	while(curr_pos != 27)
	{ 
		 Left_white_line = ADC_Conversion(3);
		 Center_white_line = ADC_Conversion(4);
		 Right_white_line = ADC_Conversion(5);
		 error = Right_white_line - Left_white_line;
		 int motorSpeed = Kp * error + Kd * (error - lastError);
		 lastError = error;
		 int rightMotorSpeed = rightBaseSpeed - motorSpeed;
		 int leftMotorSpeed = leftBaseSpeed + motorSpeed;
		 if (rightMotorSpeed > rightMaxSpeed ) rightMotorSpeed = rightMaxSpeed;
		 if (leftMotorSpeed > leftMaxSpeed ) leftMotorSpeed = leftMaxSpeed;
		 if (rightMotorSpeed < 0) rightMotorSpeed = 0;
		 if (leftMotorSpeed < 0) leftMotorSpeed = 0;

		 forward();
		 velocity(leftMotorSpeed,rightMotorSpeed);

		 if(Center_white_line>40 && Left_white_line>40 && Right_white_line>40)
		 {
			  update_curr_pos();
			  
			 // velocity(40,40);
			  //_delay_ms(300);
			  //just_forward();
			  stop();
			  buzz();
			  if(curr_pos == 20)
			  {
				  all_xbee_clear();
			  }
			if(curr_pos!=27)
			{ 
				//forward();
				//velocity(100,100);
				//_delay_ms(300);
				just_forward();
			}
			else
			{
				ensure_orient(3);
				array_index = 0;
				set_cover=0;
				++set;
				xbee_com=-1;
				event=0;
		    }	
					
		  }
	    	 
     }		
}

/*
  * Function Name:line_follow_till_2()
  * Input: None
  * Output: None
  * Logic: This function is used for following the black line till the bot reaches node just before the plant of its interest.
           If their are no plants it will simply traverse the other end of the set and come back to the start of the start, i.e., node 13.
  * Example Call: line_follow_till_2()
  */ 

void line_follow_till_1(unsigned int req_pos)
{
	if(((req_pos == 13) && (set_cover == 0)) || ((req_pos == 7) && (set_cover == 50)))
	{
		forward_mm(150);
		event=1;
	}
	else
	{
		while(curr_pos!=req_pos)
	{
		Left_white_line = ADC_Conversion(3);
		Center_white_line = ADC_Conversion(4);
		Right_white_line = ADC_Conversion(5);
		error = Right_white_line - Left_white_line;
		int motorSpeed = Kp * error + Kd * (error - lastError);
		lastError = error;
		int rightMotorSpeed = rightBaseSpeed - motorSpeed;
		int leftMotorSpeed = leftBaseSpeed + motorSpeed;
		if (rightMotorSpeed > rightMaxSpeed ) rightMotorSpeed = rightMaxSpeed;
		if (leftMotorSpeed > leftMaxSpeed ) leftMotorSpeed = leftMaxSpeed;
		if (rightMotorSpeed < 0) rightMotorSpeed = 0;
		if (leftMotorSpeed < 0) leftMotorSpeed = 0;

		forward();
		velocity(leftMotorSpeed,rightMotorSpeed);

		if(Center_white_line>40 && Left_white_line>40 && Right_white_line>40)
		{
			update_curr_pos();
			//velocity(40,40);
			//_delay_ms(300);
			//just_forward();
			stop();
			buzz();
			
			
			if((curr_pos==req_pos) && (curr_pos!=13) && (curr_pos!=7))
			{
				forward_mm(210);
				event=1;
			}
			else if((curr_pos==req_pos) && (curr_pos==7) && (set_cover == 0))
			{
				ensure_orient(1);
				set_cover = 50;
				array_index = 0;
				event=0;
			}
			else if((curr_pos==req_pos) && (curr_pos==13) && (set_cover == 50))
			{
				ensure_orient(2);
				UDR = 0x80;
				_delay_ms(100);
				UDR = 0x80;
				_delay_ms(100);
				UDR = 0x80;
				home_bit[0]=1;
				event=2;
				start_flag = 1;
			}
			else
			{
				//forward();
				//velocity(100,100);
				//_delay_ms(300);
				just_forward();
			}
			if(curr_pos == 11)
			{
				all_xbee_clear();
			}
		}
	}
  }  	
}

/*
  * Function Name:line_follow_till_1()
  * Input: None
  * Output: None
  * Logic: This function is used for following the black line till the bot reaches node just before the plant of its interest.
           If their are no plants it will simply traverse the other end of the set and come back to the start of the start, i.e., node 41.
  * Example Call: line_follow_till_1()
  */ 

void line_follow_till_2(unsigned int req_pos)
{
	if(((req_pos == 27) && (set_cover == 0)) || ((req_pos == 21) && (set_cover == 50)))
	{
		forward_mm(150);
		event=1;
	}
	else
	{
		while(curr_pos!=req_pos)
		{
			Left_white_line = ADC_Conversion(3);
			Center_white_line = ADC_Conversion(4);
			Right_white_line = ADC_Conversion(5);
			error = Right_white_line - Left_white_line;
			int motorSpeed = Kp * error + Kd * (error - lastError);
			lastError = error;
			int rightMotorSpeed = rightBaseSpeed - motorSpeed;
			int leftMotorSpeed = leftBaseSpeed + motorSpeed;
			if (rightMotorSpeed > rightMaxSpeed ) rightMotorSpeed = rightMaxSpeed;
			if (leftMotorSpeed > leftMaxSpeed ) leftMotorSpeed = leftMaxSpeed;
			if (rightMotorSpeed < 0) rightMotorSpeed = 0;
			if (leftMotorSpeed < 0) leftMotorSpeed = 0;

			forward();
			velocity(leftMotorSpeed,rightMotorSpeed);

			if(Center_white_line>40 && Left_white_line>40 && Right_white_line>40)
			{
				update_curr_pos();
				
				//velocity(40,40);
				//_delay_ms(300);
				//just_forward();
				stop();
				buzz();
				
				
				if((curr_pos==req_pos) && (curr_pos!=27) && (curr_pos!=21))
				{
					forward_mm(210);
					event=1;
				}
				else if((curr_pos==req_pos) && (curr_pos==21) && (set_cover == 0))
				{
					ensure_orient(1);
					event = 0;
					set_cover = 50;
					array_index = 0;
				}
				else if((curr_pos==req_pos) && (curr_pos==27) && (set_cover == 50))
				{
					ensure_orient(0);
					UDR = 0x80;
					_delay_ms(100);
					UDR = 0x80;
					_delay_ms(100);
					UDR = 0x80;
					home_bit[0]=1;
					event=2;
					start_flag = 1;
				}
				else
				{
					//forward();
					//velocity(100,100);
					//_delay_ms(300);
					just_forward();
				}
				if(curr_pos == 25)
				{
					all_xbee_clear();
				}
			}
		}
	}
	
}

/*
  * Function Name:line_follow_set0
  * Input: None
  * Output: None
  * Logic: This function is used for following the black line and till a node is sensed by IR proximity sensors or 
           the bot has reached the other end of the set, i.e. node 21, where it has to make a 180 deg turn or
		   it has reached the end of set traversal, i.e. node 27, where the bot has to make a 90 deg right turn and wait for others to complete their set traversal. 
           
  * Example Call: line_follow_set0()
  */ 

void line_follow_set0(void)
{
	
	Left_white_line = ADC_Conversion(3);
	Center_white_line = ADC_Conversion(4);
	Right_white_line = ADC_Conversion(5);
	error = Right_white_line - Left_white_line;
	int motorSpeed = Kp * error + Kd * (error - lastError);
	lastError = error;
	int rightMotorSpeed = rightBaseSpeed - motorSpeed;
	int leftMotorSpeed = leftBaseSpeed + motorSpeed;
	if (rightMotorSpeed > rightMaxSpeed ) rightMotorSpeed = rightMaxSpeed;
	if (leftMotorSpeed > leftMaxSpeed ) leftMotorSpeed = leftMaxSpeed;
	if (rightMotorSpeed < 0) rightMotorSpeed = 0;
	if (leftMotorSpeed < 0) leftMotorSpeed = 0;

	forward();
	velocity(leftMotorSpeed,rightMotorSpeed);

	if(Center_white_line>40 && Left_white_line>40 && Right_white_line>40)
	{	
		update_curr_pos();
		
	//	velocity(40,40);
		//_delay_ms(300);
		//just_forward();
		stop();
		buzz();
		
       if(curr_pos == 35)
		{
			ensure_orient(1);
			UDR = 0x80;
			_delay_ms(100);
			UDR = 0x80;
			_delay_ms(100);
			UDR = 0x80;
			home_bit[0] = 1;
			while(all_bot_ready() == 0);
			all_bot_clear();
			event = 0;
			cycle = 1;
			set_cover=50;
		}
		else if(curr_pos==41)
		{
			event=3;
			cycle = 1;
			ensure_orient(3);
			home_bit[0]=1;
			UDR = 0x80;
			_delay_ms(100);
			UDR = 0x80;
			_delay_ms(100);
			UDR = 0x80;
		}
		else
		{
			event=0;
			//forward();
			//velocity(100,100);
			//_delay_ms(300);
			just_forward();
		}
		if((curr_pos == 39) && (set_cover == 0))
		{
			all_xbee_clear();
		}
		if((curr_pos == 37) && (set_cover == 50))
		{
			all_xbee_clear();
		}
	}	 
}

void just_line()
{
	Left_white_line = ADC_Conversion(3);
	Center_white_line = ADC_Conversion(4);
	Right_white_line = ADC_Conversion(5);
	error = Right_white_line - Left_white_line;
	int motorSpeed = Kp * error + Kd * (error - lastError);
	lastError = error;
	int rightMotorSpeed = rightBaseSpeed - motorSpeed;
	int leftMotorSpeed = leftBaseSpeed + motorSpeed;
	if (rightMotorSpeed > rightMaxSpeed ) rightMotorSpeed = rightMaxSpeed;
	if (leftMotorSpeed > leftMaxSpeed ) leftMotorSpeed = leftMaxSpeed;
	if (rightMotorSpeed < 0) rightMotorSpeed = 0;
	if (leftMotorSpeed < 0) leftMotorSpeed = 0;

	forward();
	velocity(leftMotorSpeed,rightMotorSpeed);
}

 /*
  * Function Name:velocity
  * Input: unsigned char left_motor,unsigned char right_motor
  * Output: None
  * Logic: This function sets the desired values of left and right 
           motor speeds in output compare register.Using PWM the left 
           and right motor speeds are controlled.
  * Example Call: velocity(70,70)
  */ 

void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR1AH = 0x00;
	OCR1AL = left_motor;
	OCR1BH = 0x00;
	OCR1BL = right_motor;
}

/*
  * Function Name:motion_set
  * Input: None
  * Output: None
  * Logic: This function sets the motion pins of the bot to move in a particular direction.
  * Example Call: motion_set(0x06)
  */ 

void motion_set (unsigned char Direction)
{
	unsigned char PortBRestore = 0;

	Direction &= 0x0F; 			// removing upper nibbel as it is not needed
	PortBRestore = PORTB; 			// reading the PORTB's original status
	PortBRestore &= 0xF0; 			// setting lower direction nibbel to 0
	PortBRestore |= Direction; 	// adding lower nibbel for direction command and restoring the PORTB status
	PORTB = PortBRestore; 			// setting the command to the port
}

/*
  * Function Name:ISR
  * Input:INT1_vect
  * Output: None
  * Logic: This interrupt service routine is called on receiving pulses from right position encoder.
  * Example Call: N.A
  */

ISR(INT1_vect)
{
	++shaft_count_right;
}

/*
  * Function Name:ISR
  * Input:INT0_vect
  * Output: None
  * Logic: This interrupt service routine is called on receiving pulses from left position encoder.
  * Example Call: N.A
  */

ISR(INT0_vect)
{
	++shaft_count_left;
}


/*
  * Function Name:linear_distance_mm
  * Input: unsigned int DistanceInMM -- distance in mm
  * Output: None
  * Logic: This function is used for following the black line till the bot covers a distance in mm passed to it as a parameter.
  * Example Call: linear_distance_mm(150)
  */ 

void linear_distance_mm(unsigned int DistanceInMM)
{
 	req_shaft_count=0;
	req_shaft_count_int=0;

 	req_shaft_count = (float)(DistanceInMM / 12.92);
 	req_shaft_count_int = (unsigned long int)req_shaft_count;

 	shaft_count_right=0;
 	while(1)
 	{
		 just_line();
  		if(shaft_count_right > req_shaft_count_int)
  		{
  			break;
  		}
 	}
 	stop(); //Stop robot
}

/*
  * Function Name:angle_rotate
  * Input: unsigned int deg -- degrees by which the bot has to turn
  * Output: None
  * Logic: This function is used for rotating the bot by a specific angle.
  * Example Call: angle_rotate(100)
  */

void angle_rotate(unsigned int deg)
{
	req_shaft_count=0;
	req_shaft_count_int=0;

	req_shaft_count=(float)(deg/12.85);
	req_shaft_count_int=(unsigned long int)req_shaft_count;
	shaft_count_left=0;
	shaft_count_right=0;

	while(1)
	{
		if((shaft_count_right >= req_shaft_count_int)|(shaft_count_left >= req_shaft_count_int))
		break;
 	}
	stop();
}

 /*
  * Function Name:forward
  * Input: None
  * Output: None
  * Logic: This function sets up the motion pins of the bot to set the bot in the forward direction
  * Example Call: forward()
  */ 

void forward (void)         //both wheels forward
{
	motion_set(0x06);
}

/*
  * Function Name:back
  * Input: None
  * Output: None
  * Logic: This function sets up the motion pins of the bot so that it moves in the backward direction
  * Example Call: back()
  */ 

void back (void)
{
	motion_set(0x09);
}

/*
  * Function Name:left
  * Input: None
  * Output: None
  * Logic: This function sets up the motion pins of the bot to set the bot so that it turns left
  * Example Call: left()
  */ 
void left (void) 
{
  motion_set(0x05);
}
   /*
  * Function Name:right
  * Input: None
  * Output: None
  * Logic: This function sets up the motion pins of the bot so that the bot turns right
  * Example Call: right()
  */ 
void right (void) 
{
  motion_set(0x0A);
}
 /*
  * Function Name:soft_left
  * Input: None
  * Output: None
  * Logic: This function sets up the motion pins of the bot so that the bot takes a soft left turn
  * Example Call: soft_left()
  */ 
void soft_left (void) //Left wheel stationary, Right wheel forward
{
 motion_set(0x04);
}
  /*
  * Function Name:soft_right
  * Input: None
  * Output: None
  * Logic: This function sets up the motion pins of the bot so that the bot takes a soft right turn
  * Example Call: soft_right()
  */ 
void soft_right (void) //Left wheel forward, Right wheel is stationary
{
 motion_set(0x02);
}
 /*
  * Function Name:soft_left_2
  * Input: None
  * Output: None
  * Logic: This function sets up the motion pins of the bot so that the bot takes a soft left turn in the reverse direction
  * Example Call: soft_left_2()
  */ 
void soft_left_2 (void) //Left wheel backward, right wheel stationary
{
 motion_set(0x01);
}
  /*
  * Function Name:soft_right_2
  * Input: None
  * Output: None
  * Logic: This function sets up the motion pins of the bot so that the bot takes a soft right turn in the reverse direction
  * Example Call: soft_right_2()
  */ 
void soft_right_2 (void) //Left wheel stationary, Right wheel backward
{
 motion_set(0x08);
}
 /*
  * Function Name:stop
  * Input: None
  * Output: None
  * Logic: This function sets up the motion pins of the bot so that the bot stops
  * Example Call: stop()
  */ 
void stop (void)
{
  motion_set(0x00);
}

/*
  * Function Name:forward_mm
  * Input: unsigned int DistanceInMM -- distance in mm
  * Output: None
  * Logic: This function sets up the motion pins of the bot for forward movement and calls linear_distance_mm("distance in mm") function for the bot to move by a specific distance.
  * Example Call: forward_mm(100)
  */ 

void forward_mm(unsigned int DistanceInMM)
{
 	forward();
 	linear_distance_mm(DistanceInMM);
}

/*
  * Function Name:left_degrees
  * Input: unsigned int Degrees -- degrees by which the bot has to turn left
  * Output: None
  * Logic: This function is used for setting pins for left turn and rotating the bot by a specific angle.
  * Example Call: left_degrees(100)
  */

void left_degrees(unsigned int Degrees)
{
 	left();
 	angle_rotate(Degrees);
}

/*
  * Function Name:right_degrees
  * Input: unsigned int Degrees -- degrees by which the bot has to turn right
  * Output: None
  * Logic: This function is used for setting pins for right turn and rotating the bot by a specific angle.
  * Example Call: right_degrees(100)
  */

void right_degrees(unsigned int Degrees)
{

 	right();
 	angle_rotate(Degrees);
}

/*
  * Function Name:turn
  * Input: unsigned int direction -- the direction of 90 degree turn : 0 for left and 1 for right
  * Output: None
  * Logic: This function is used for rotating the bot by 60 degrees (either left or right) first with the help of position encoders 
           and then to just keep rotating till the center white line senses black line just below it.
  * Example Call: turn(0)
  */

void turn(unsigned int direction)
{
    //0 for left
    //1 for right

    if(direction==0)
    {
        left_degrees(80);
        Center_white_line = ADC_Conversion(4);	//Getting data of Center WL Sensor
        while(Center_white_line < 60)
        {
            left();
            Center_white_line = ADC_Conversion(4);
        }
        stop();
		if(orient==0)orient=3;
		else
		orient=orient-1;
    }
    else if(direction == 1)
    {
        right_degrees(80);
        Center_white_line = ADC_Conversion(4);	//Getting data of Center WL Sensor
        while(Center_white_line < 60)
        {
            right();
            Center_white_line = ADC_Conversion(4);
        }
        stop();
		orient=(orient+1)%4;
    }
	else if(direction == 2)
	{
		right_degrees(150);
		Center_white_line = ADC_Conversion(4);	//Getting data of Center WL Sensor
		while(Center_white_line < 60)
		{
			right();
			Center_white_line = ADC_Conversion(4);
		}
		stop();
		orient=(orient+2)%4;
	}
}

/*
  * Function Name:ensure_orient
  * Input: unsigned int req_orient -- the required orientation of the bot
  * Output: None
  * Logic: This function is used for rotating the bot by 90 degrees right till the required orientation is achieved. 
  * Example Call: ensure_orient(0)
  */

void ensure_orient(unsigned int req_orient)
{
	stop();
	int chethan=0;
	while(req_orient!=orient)
	{   
		if(chethan==0)
		{
			//turn(1);
			turn_right();
			chethan=1;
		}		
		else
		{
			
			turn_right_ensure();
		}
	}
}

/*
  * Function Name:update_curr_pos
  * Input: None
  * Output: None
  * Logic: This function updates the current position of the bot depending upon its present orientation.
           Its called upon node detection by the bot.
  * Example Call: update_curr_pos()
  */

void update_curr_pos()
{
	if(orient==0)        curr_pos-=7;
	else if(orient==1)   curr_pos+=1;
	else if(orient==2)   curr_pos+=7;
	else if(orient==3)   curr_pos-=1;
}