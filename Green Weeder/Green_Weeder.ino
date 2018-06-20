/*
 * Team Id: eYRC-841orig
 * 
 * Author List: Mahesh Joseph Sadashiv, Harsh Lunia and Faiz Rahman
 * 
 * Filename: testbotOG.c
 * 
 * Theme: Feeder-Weeder
 * 
 * Functions:  ISR(INT0_vect);
               ISR(INT5_vect);
 ISR(INT4_vect)
 ISR(USART0_RX_vect)
 void fire_for_red();
 void fire_for_blue();
 void dijkstra(int n,int startnode,int obst,int obst2,int obst3,int obst4);
 void lineFollow(void);
 void servoPick(void); 
 void colorSense(void);
 void color_sensor_pin_config(void);
 void printSensor(void); 
 boolean irSense(void);
 void buzz(void);
 LiquidCrystal lcd(RS, EN, DS4, DS5, DS6, DS7); 
 void color_sensor_pin_interrupt_init(void);
 void filter_red(void);
 void filter_green(void);
 void filter_blue(void);
 void color_sensor_scaling(void);
 void red_read(void);
 void blue_read(void);
 void green_read(void);   
 void color_check(void);
 void setup();
 void buzz(void);
 void printSensor(void);
 void lineFollow(void);
 void timer5_init();
 void velocity (unsigned char left_motor, unsigned char right_motor);
 void motion_pin_config (void);
 void init_ports();
 void motion_set (unsigned char Direction);
 void forward (void);
 void back (void);  
 void left (void);  
 void right (void);
 void soft_left (void);
 void soft_right (void);
 void soft_left_2 (void);
 void soft_right_2 (void);
 void stop (void);
 void color_init(void);
 void uart0_init(void);
 void init_devices (void);
 boolean irSense(void);
 void loop();
 void set_zero_path();
 int all_bot_ready();
 void all_bot_clear();
 unsigned int convert(unsigned char x1,unsigned char x2);
 void shiftr(int b);
 void shiftg(int b);
 void reach_destination(unsigned char s1, unsigned char s2 , unsigned int ini_orient);
 void navigate_0();
 void left_encoder_pin_config (void);
 void right_encoder_pin_config (void);
void left_position_encoder_interrupt_init(void);
void right_position_encoder_interrupt_init(void);
void angle_rotate(unsigned int Degrees);
void linear_distance_mm(unsigned int DistanceInMM);
void forward_mm(unsigned int DistanceInMM);
void back_mm(unsigned int DistanceInMM);
void left_degrees(unsigned int Degrees);
void right_degrees(unsigned int Degrees);
void turn(unsigned int direction);
void ensure_orient(unsigned int req_orient);
void update_curr_pos(void);
void Main(void);
void set_zero();
void wall_type(int cval,int *d);
void insert_row(int i,int j,int *d);
void buildGraph(void);
void main_function( int start, int finish,int obst,int obst2,int obst3,int obst4);
void ReachDestinationAvoidingNode(volatile int *path2,volatile int c,volatile int a,volatile int b,volatile int d,volatile int e,volatile int f);
void finish(void);
void line_follow_till_2();
void line_follow_till_1();
void servoDrop();
void line_follow_set0();

void run_home(void); 
          
 * Global Variables:  int src;
   struct Graph adjacency_matrix1;
   struct Path_Array path1;
   int left_white_line_sensor;
   int center_white_line_sensor;
   int right_white_line_sensor;
   int RS,RW,EN,DS4,DS5,DS6,DS7;
   volatile  int Left_white_line;
   volatile  int Center_white_line ;
   volatile  int Right_white_line ;
   volatile  int flag;
   volatile  int frequency;
   volatile int i;
   volatile unsigned long int ShaftCountLeft = 0; 
   volatile unsigned long int ShaftCountRight = 0; 
   volatile unsigned int Degrees; 
   volatile int temp_tx;
   volatile int arrpath[60];
   volatile int gpath[60];
   volatile int rpath[60];
   volatile int bpath[60];
   volatile int temp,i2,i3;
   volatile int g;
   volatile int r;
   volatile int b;
   volatile int tempmaze1[7][7];
   volatile uint8_t tempGraph[X][X];
   volatile uint8_t cost[MAX][MAX];
   volatile unsigned int home_bit[3]; 
volatile unsigned int curr_pos;      
volatile unsigned int cycle;              
volatile unsigned int orient;               
volatile unsigned int set;                 
volatile unsigned int set_cover;           
volatile unsigned int event;                
volatile unsigned int temp_node;
volatile unsigned int flag_switch;
volatile uint8_t j;                          
volatile unsigned char data;
volatile unsigned char data_rec;
volatile unsigned char data_temp;
volatile unsigned char data_temp_s1;
volatile unsigned char data_temp_s2;
volatile int8_t xbee_com;
volatile unsigned int node_count;
int initial_home_run[10];
volatile int right_plant_set1[5];
volatile int left_plant_set1[5];
volatile int right_plant_set2[5];
volatile int left_plant_set2[5];
volatile int right_plant_set1_count;
volatile int left_plant_set1_count;
volatile int right_plant_set2_count;
volatile int left_plant_set2_count;
volatile int array_index=0;

volatile unsigned  int color;
volatile unsigned long int pulse; 
volatile unsigned long int red;      
volatile unsigned long int blue;  
volatile unsigned long int green;                                 
 * 
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


#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978


#include <Servo.h>
#include <LiquidCrystal.h>
#include<math.h> //included to support power function
#include<ctype.h>
#include<avr/io.h>
#include<avr/interrupt.h>
void fire_for_red();
void fire_for_blue();
void run_home();
void ir_check();
int speakerPin=34;
void GameOfThrones();
/*******************************************************PIN DECLARATIONS******************************************************************/
//servoMid:Servo object used to access elbow servo motor of weeder arm
Servo servoMid;
//servoBase: Servo object used to access shoulder servo motor of weeder arm
Servo servoBase;
//servoTop: Servo object used to access wrist motor of weeder arm
Servo servoTop;
//Buzzer: pin no. used as output for buzzer
#define Buzzer 34
//left_white_line_sensor: used to access the analog left white line sensor values through adc channel 3
int left_white_line_sensor = A3;
//center_white_line_sensor: used to access the analog center white line sensor values through adc channel 2
int center_white_line_sensor = A2;
//right_white_line_sensor: used to access the analog right white line sensor values through adc channel 1
int right_white_line_sensor = A1;
//enabe_lineSensor: pin number used for enabling line sensors
#define enabe_lineSensor 39
//enable_irSensor: pin number used for enabling irsensor
#define enable_irSensor 6
//ir4: pin no. used to get reading of ir sensor5
#define ir4 A8
//RS,RW,EN,DS4,DS5,DS6,DS7: used for initialising lcd
int RS = 37,RW = 36 ,EN = 35,DS4 = 33,DS5 =32  ,DS6 = 31 ,DS7 = 30;
//red: output pin for red color in rgb led
#define redLED 49//extension board pin=28
//green: output pin for green color in rgb led
#define greenLED 48 //extension board pin=27 interchange green and blue in the module
//blue:output pin for blue color in rgb led
#define blueLED 47 //extension board pin=30
/********************************************************GLOBAL_VARIABLES**************************************************************************/
//Left_white_line:used to store left white line sensor value and according to value ,line following is executed
volatile  int Left_white_line;
//Center_white_line:used to store center white line sensor value and according to value ,line following is executed
volatile  int Center_white_line ;
//Right_white_line:used to store right white line sensor value and according to value ,line following is executed
volatile  int Right_white_line ;
//flag: used to implement line following and correct path of robot when it strays from line 
volatile  int flag;
//i:iteration variable used to execute loops
volatile int i;
//ShaftCountLeft: used to keep track of left position encoder
volatile unsigned long int ShaftCountLeft = 0;
//ShaftCountRight: used to keep track of right position encoder 
volatile unsigned long int ShaftCountRight = 0;
//Degrees: used to accept angle in degrees for turning 
volatile unsigned int Degrees; 
#define INFI 9999
#define MAX 49
#define X 49
#define Y 7
volatile unsigned int temp_tx;
volatile unsigned int arrpath[60];
volatile unsigned int gpath[60];
volatile unsigned int rpath[60];
volatile unsigned int bpath[60];
volatile unsigned int temp,i2,i3; // CHECK WITH CHETHAN
volatile unsigned int g;
volatile unsigned int r;
volatile unsigned int b;
volatile unsigned int blue_ini;
volatile unsigned int red_ini;
volatile int tempmaze1[7][7]={{9,1,1,1,1,1,3},{8,0,0,0,0,0,2},{8,0,0,0,0,0,2},{8,0,0,0,0,0,2},{8,0,0,0,0,0,2},{8,0,0,0,0,0,2},{12,4,4,4,4,4,6}};
volatile uint8_t tempGraph[X][X];
volatile uint8_t cost[MAX][MAX];
volatile int red_xbee_flag = 0;
volatile int blue_xbee_flag = 0;

/********************************************************INTIGRATION_VARIABLES****************************************************/
//general variables
/*home_bit[3]:used to verify if all bots have finished their set and are ready to execute next set depending on whether value is 0(bot not ready) or 1(bot ready).
  homebit[0]-indicates bot itself
  homebit[1]-indicates red bot
  homebit[2]-indicates blue bot
 */   
volatile unsigned int home_bit[3]={0,0,0};

//curr_pos: used to keep track of the current node position of bot
volatile unsigned int curr_pos; 
/*cycle: used to control the flow of program.Values of cycle indicate folowing:-
  0-initial home run to take place
  1-used for set0 and navigation to take place
  2-used for going to sepecified nodes and doing job for sets 1 and 2 and also for navigation
  3-used to indicate final home run is occurring
  4-return from main
 */ 
volatile unsigned int cycle=0;
/*orient:keep track of present orientation of the bot.Its values indicates:-
   0-North
   1-East
   2-South
   3-West 
 */
volatile unsigned int orient;
//set:indicates which portion of the arena bot is in    
volatile unsigned int set=0;
//set_cover:indicates percentage of completion of set: 0-0%,50-50%,100-100%                 
volatile unsigned int set_cover=0;
//event:helps to distinguish between tasks being performed in a cycle and  controls the flow of program          
volatile unsigned int event=0;
//temp_node: stores location of node temporarily                
volatile unsigned int temp_node;
//flag_switch:Set at 0 initially,then used to break once switch case evaluates by getting set to 1
volatile unsigned int flag_switch=0;
//j:iteration variable used for loop
volatile uint8_t j;
volatile int irVal  = 0;
//----------------------------

//Xbee variables
//data:used to store value for transmission before it is assigned to UDR
volatile unsigned char data=0x00;
//data_rec:used to store data stored in UDR after it is  received through XBee 
volatile unsigned char data_rec;
//data_temp:used to store the first two bits of data_rec from which ID of sender can be fund out
volatile unsigned char data_temp;
//data_temp_s1:Used to store bits 3-5 of data_rec from which row of green plant can be obtained
volatile unsigned char data_temp_s1;
//data_temp_s2:Used to store bits 6-8 of data_rec from which column of green plant can be obtained
volatile unsigned char data_temp_s2;
//xbee_com:Used to determine course of action in the event of receival of data through XBee(0:receive path of other codes,1:receive location of green plants,-1:should not do anything)
volatile int8_t xbee_com=0;          

//-----------------------------

//Dijkstra path variables
volatile unsigned int node_count=0;
int initial_home_run[10];

//-------------------------------
//Plant block location


//right_plant_set1[]:array to store location of green plants to be weeded on the right side of set1
volatile int right_plant_set1[5];
//left_plant_set1[]:array to store location of green plants to be weeded on the left side of set1
volatile int left_plant_set1[5];
//right_plant_set2[]:array to store location of green plants to be weeded on the right side of set2
volatile int right_plant_set2[5];
//left_plant_set2[]:array to store location of green plants to be weeded on the left side of set2
volatile int left_plant_set2[5];
//right_plant_set1_count:counter to keep track of the no. of plants on right side of set1
volatile int right_plant_set1_count=0;
//left_plant_set1_count:counter to keep track of the no. of plants on left side of set1
volatile int left_plant_set1_count=0;
//right_plant_set2_count:counter to keep track of the no. of plants on right side of set2
volatile int right_plant_set2_count=0;
//left_plant_set2_count:counter to keep track of the no. of plants on left side of set2
volatile int left_plant_set2_count=0;
//array_index: pointer for accessing the arrays:right_plant_set1[],right_plant_set2[],left_plant_set1[],left_plant_set2[]
volatile int array_index=0;

//---------------------------------

//COLOR SENSOR 
//color:based on what the color of scanned object is,it will be 0(red),1(blue) or 2(green)
volatile unsigned  int color;
//pulse:used to keep count of the no. of pulses                     
volatile unsigned long int pulse = 0; //to keep the track of the number of pulses generated by the color sensor
//red:stores pulse count when read_red() function is called
volatile unsigned long int red;
//blue:stores pulse count when read_blue() function is called
volatile unsigned long int blue;
//green:stores pulse count when read_green() function is called
volatile unsigned long int green;     // variable to store the pulse count when read_green function is called

//----------------------------------------
/*********************************************************PROTOTYPING****************************************************************************/
 void lineFollow(void);
 void servoPick(void); 
 void colorSense(void);
 void printSensor(void); 
 boolean irSense(void);
 void buzz(void);
 LiquidCrystal lcd(RS, EN, DS4, DS5, DS6, DS7); 
/*************************************************************************************************************************************/
 void ir_check(void)
 {
   if(analogRead(A6) < 500)
   {
     stop();
     delay(3000);
     forward();
     velocity(150,150);  
   }
 }
 /*
  * Function Name:color_sensor_pin_config
  * Input: None
  * Output: None
  * Logic: Initialize the pins for color sensor
  * Example Call: color_sensor_pin_config()
  */ 
 void color_sensor_pin_config(void)
   {
     DDRD  = DDRD | 0xFE; //set PD0 as input for color sensor output
     PORTD = PORTD | 0x01;//Enable internal pull-up for PORTD 0 pin
   }
 /*
  * Function Name:color_sensor_port_init
  * Input: None
  * Output: None
  * Logic: Initialize the necessary ports for color sensor
  * Example Call: color_sensor_port_init()
  */ 
  void color_port_init(void)
 { 
  color_sensor_pin_config();//color sensor pin configuration
 }
 /*
  * Function Name:color_sensor_pin_interrupt_init
  * Input: None
  * Output: None
  * Logic: Initialize the interrupt pins for color sensor
  * Example Call: color_sensor_pin_interrupt_init()
  */ 
  
void color_sensor_pin_interrupt_init(void) //Interrupt 0 enable
{
  cli(); //Clears the global interrupt
  EICRA = EICRA | 0x02; // INT0 is set to trigger with falling edge
  EIMSK = EIMSK | 0x01; // Enable Interrupt INT0 for color sensor
  sei(); // Enables the global interrupt
}
 /*
  * Function Name:ISR
  * Input: INT0_vect
  * Output: None
  * Logic: Increment the pulse count  on receiving pulse from the color sensor
  */ 
  ISR(INT0_vect)
    {
      pulse++; 
    } 
//Filter Selection
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
      PORTD = PORTD & 0xBF; //set S2 low
      PORTD = PORTD & 0x7F; //set S3 low
    }
 /*
  * Function Name:filter_green()
  * Input: None
  * Output: None
  * Logic: Initialize the pins for reading pulses corresponding to green color
  * Example Call: filter_green()
  */ 
  void filter_green(void) //Used to select green filter
    {
     //Filter Select - green filter
      PORTD = PORTD | 0x40; //set S2 High
      PORTD = PORTD | 0x80; //set S3 High
    }
/*
  * Function Name:filter_blue()
  * Input: None
  * Output: None
  * Logic: Initialize the pins for reading pulses corresponding to blue color
  * Example Call: filter_blue()
  */ 
  void filter_blue(void)  //Used to select blue filter
  {
    //Filter Select - blue filter
    PORTD = PORTD & 0xBF; //set S2 low
    PORTD = PORTD | 0x80; //set S3 High
  }



//Color Sensing Scaling
 /*
  * Function Name:color_sensor_scaling
  * Input: None
  * Output: None
  * Logic: This function is used to select the scaled down version of the original frequency of the output generated by the color sensor(20%)
  * Example Call: color_sensor_scaling()
  */ 
  void color_sensor_scaling()   
  {
    PORTD = PORTD | 0x10; //set S0 high
    PORTD = PORTD | 0x20; //set S1 high
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
  void red_read(void) 
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
  void green_read(void) 
    {
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
  * Example Call: red_read()
  */ 
  void blue_read(void) 
  {
     filter_blue(); //select blue filter
     pulse=0; //reset the count to 0
    _delay_ms(100); //capture the pulses for 100 ms or 0.1 second
    blue = pulse;  //store the count in variable called blue  
  }
 /*
  * Function Name:color_check
  * Input: None
  * Output: None
  * Logic: This function is used to find color of object
           based on which color pulse count is the largest
  * Example Call: color_check()
  */ 
  void color_check()
    {
        
        color_sensor_scaling();
        color = 99;
        red_read(); //display the pulse count when red filter is selected
        delay(500);
        green_read(); //display the pulse count when green filter is selected
        delay(500);
        blue_read(); //display the pulse count when blue filter is selected
        delay(500);
        if(red>blue&&red>green)
          color = 0; 
        if(green>blue&&green>red)
          color = 1;
        if(blue>red&&blue>green)
          color = 2;
 
    
    }
 /*
  * Function Name:setup
  * Input: None
  * Output: None
  * Logic: This function sets up the IRsensors,LCD,Servo Motors,UART port and the rest of the devices and readies the bot for running the program
  * Example Call: setup()
  */ 
void setup()
  {   
    
    init_devices();
    pinMode(speakerPin,OUTPUT);
    pinMode(A6,INPUT);
    pinMode(ir4,INPUT);
    pinMode(enable_irSensor,OUTPUT);
    digitalWrite(enable_irSensor,LOW); 
    pinMode(RW,OUTPUT);//for lcd
    digitalWrite(RW,LOW);//for lcd
    lcd.begin(16, 2);
    pinMode(enabe_lineSensor,OUTPUT);
    digitalWrite(enabe_lineSensor,LOW);
    pinMode(Buzzer,OUTPUT);
    pinMode(right_white_line_sensor,INPUT);
    pinMode(center_white_line_sensor,INPUT);
    pinMode(left_white_line_sensor,INPUT);
    pinMode(redLED,OUTPUT);
    pinMode(blueLED,OUTPUT);
    pinMode(greenLED,OUTPUT); 
    servoTop.attach(11);
    servoMid.attach(12);
    servoBase.attach(13);
  }
/*************************************************************************************************************************************/
  /*
  * Function Name:buzz
  * Input: None
  * Output: None
  * Logic: This function switches ON the buzzer for 1 second and offs it
  * Example Call: buzz()
  */ 
    void buzz(void)
    {
        digitalWrite(Buzzer,HIGH);
        delay(500);
        digitalWrite(Buzzer,LOW);
        delay(50);
    }
    void servoDrop(void)
{
   servoBase.write(160);
   delay(500);
   servoTop.write(60);
   delay(500);
   servoMid.write(140);
   delay(500);
   for(i=160 ; i>=20 ; i= i-10)
   {
   servoBase.write(i);
   delay(100);  
   } 
   servoMid.write(70);
   delay(500);
}

/**************************************************************************************************************************************/
 /*
  * Function Name:printSensor
  * Input: None
  * Output: None
  * Logic: This function acquires the readings of white line sensors and prints their values on the lcd screen if any calibration is needed
  * Example Call: printSensor();
  */ 
  void printSensor(void)
    {
      
      Left_white_line = map(analogRead(left_white_line_sensor),30,450,0,150);
      Center_white_line =map(analogRead(center_white_line_sensor),30,450,0,150);
      Right_white_line = map(analogRead(right_white_line_sensor),30,450,0,150);
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("left   center   right");
      lcd.setCursor(0, 1);
      lcd.print(Left_white_line);
      lcd.print(" ");
      lcd.print(Center_white_line);
      lcd.print(" ");
      lcd.print(Right_white_line);
          
    }
/**************************************************************************************************************************************/
  /*
  * Function Name:line_follow_set0()
  * Input: None
  * Output: None
  * Logic: This function is used for following the black line and change sets after the bot has finished traversing the set.
  * Example Call: line_follow_set0()
  */ 
  void line_follow_set0(void)
    {
    
      Left_white_line = map(analogRead(left_white_line_sensor),30,450,0,150);
      Center_white_line =map(analogRead(center_white_line_sensor),30,450,0,150);
      Right_white_line = map(analogRead(right_white_line_sensor),30,450,0,150);
      flag=0;

      if((Left_white_line<5)&&(Right_white_line<5))
        {
          flag=1;
          forward();
          digitalWrite(46,HIGH);
          digitalWrite(45,HIGH);
        }

      if((Left_white_line>10)&&(Left_white_line<30)&&(flag==0))
        {
          flag=1;
          forward();
          velocity(150,150);  
        }
      if((Left_white_line>30)&&(Left_white_line<50)&&(flag==0))
        {
          flag=1;
          soft_left();    
        }
      if((Left_white_line>50)&&(Left_white_line<70)&&(flag==0))
        {
          flag=1;
          soft_left();
        }
      if((Right_white_line>10)&&(Right_white_line<30)&&(flag==0))
        {
          flag=1;
          forward();
          velocity(150,150); 
        }
      if((Right_white_line>30)&&(Right_white_line<50)&&(flag==0))
        {
          flag=1;
          soft_right();
        }
      if((Right_white_line>50)&&(Right_white_line<70)&&(flag==0))
       {
          flag=1;
          soft_right();
       }
      if(Center_white_line>70 && Left_white_line>70 && Right_white_line>70)
        { 
         
          stop();
          buzz();

          update_curr_pos();
          lcd.setCursor(0,0);
          lcd.print(curr_pos);
          irVal = analogRead(ir4);
        
          if(curr_pos == 7)
            {
              forward_mm(70);
              ensure_orient(1);
              event = 0; 
              UDR0 = 0x00;
              delay(100);
              UDR0 = 0x00;
              delay(100);
              UDR0 = 0x00;
              home_bit[0] = 1;
              while(all_bot_ready() == 0);
              all_bot_clear();
              set_cover= 50;
            }
          else if(curr_pos == 13)
            {
              set_cover = 50;
              forward_mm(70);
              event = 3;
              ensure_orient(2);
              home_bit[0] = 1;
              UDR0 = 0x00;
              delay(100);
              UDR0 = 0x00;
              delay(100);
              UDR0 = 0x00;
            }
         else
            {
              event = 0;
              forward();
              velocity(150,150);
              delay(400);
            }
        if((curr_pos == 11) && (set_cover == 0))
        {
           all_xbee_clear();
        }
        if((curr_pos == 9) && (set_cover == 50))
        {
          all_xbee_clear();
        }
       }
    } 
/*
  * Function Name:line_follow_till_1()
  * Input: unsigned int req_pos
  * Output: None
  * Logic: This function is used for line following of set1 for the bot
  * Example Call: line_follow_till_1(21)
  */ 
  
void line_follow_till_1(unsigned int req_pos)
{
  if(((req_pos == 27) && (set_cover == 0)) || ((req_pos == 21) && (set_cover == 50))) 
  {
    forward_mm(80);
    event=1;
  }
  else 
  {
   while(curr_pos != req_pos)
    {
      Left_white_line = map(analogRead(left_white_line_sensor),30,450,0,150);
      Center_white_line =map(analogRead(center_white_line_sensor),30,450,0,150);
      Right_white_line = map(analogRead(right_white_line_sensor),30,450,0,150);
      flag=0;

      if((Left_white_line<5)&&(Right_white_line<5))
        {
          flag=1;
          forward();
          digitalWrite(46,HIGH);
          digitalWrite(45,HIGH);
        }

      if((Left_white_line>10)&&(Left_white_line<30)&&(flag==0))
        {
          flag=1;
          forward();
          velocity(150,150);  
        }
      if((Left_white_line>30)&&(Left_white_line<50)&&(flag==0))
        {
          flag=1;
          soft_left();    
        }
      if((Left_white_line>50)&&(Left_white_line<70)&&(flag==0))
        {
          flag=1;
          soft_left();
        }
      if((Right_white_line>10)&&(Right_white_line<30)&&(flag==0))
        {
          flag=1;
          forward();
          velocity(150,150); 
        }
      if((Right_white_line>30)&&(Right_white_line<50)&&(flag==0))
        {
          flag=1;
          soft_right();
        }
      if((Right_white_line>50)&&(Right_white_line<70)&&(flag==0))
       {
          flag=1;
          soft_right();
       }
      if(Center_white_line>70 && Left_white_line>70 && Right_white_line>70)
        { 
          stop();
          buzz();
          update_curr_pos();
      
         if((curr_pos==req_pos) && (curr_pos!= 21) && (curr_pos!= 27))
         {
              forward_mm(210);
              event=1;
         }
         else if((curr_pos==req_pos) && (curr_pos==21))
         {
            forward_mm(70);
            ensure_orient(1);
            event = 0;
            set_cover = 50;
            array_index = 0;
         }
          else if((curr_pos==req_pos) && (curr_pos==27))
          {
             forward_mm(70);
             ensure_orient(2);
             UDR0 = 0x00;
             delay(100);
             UDR0 = 0x00;
             delay(100);
             UDR0 = 0x00;
             home_bit[0]=1;
             event=2;
         }
         else 
         {
            forward();
            velocity(150,150);
            _delay_ms(400);
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
  * Function Name:line_follow_till_2()
  * Input: None
  * Output: None
  * Logic: This function is used for line following of set2 for the bot
  * Example Call: line_follow_till_2(34))
  */ 
void line_follow_till_2(unsigned int req_pos)
{
  if(((req_pos == 41) && (set_cover == 0)) || ((req_pos == 35) && (set_cover == 50))) 
  {
    forward_mm(80);
    event=1;
  }
  else
  { 
    while(curr_pos != req_pos)
    {
      Left_white_line = map(analogRead(left_white_line_sensor),30,450,0,150);
      Center_white_line =map(analogRead(center_white_line_sensor),30,450,0,150);
      Right_white_line = map(analogRead(right_white_line_sensor),30,450,0,150);
      flag=0;

      if((Left_white_line<5)&&(Right_white_line<5))
        {
          flag=1;
          forward();
          digitalWrite(46,HIGH);
          digitalWrite(45,HIGH);
        }

      if((Left_white_line>10)&&(Left_white_line<30)&&(flag==0))
        {
          flag=1;
          forward();
          velocity(150,150);  
        }
      if((Left_white_line>30)&&(Left_white_line<50)&&(flag==0))
        {
          flag=1;
          soft_left();    
        }
      if((Left_white_line>50)&&(Left_white_line<70)&&(flag==0))
        {
          flag=1;
          soft_left();
        }
      if((Right_white_line>10)&&(Right_white_line<30)&&(flag==0))
        {
          flag=1;
          forward();
          velocity(150,150); 
        }
      if((Right_white_line>30)&&(Right_white_line<50)&&(flag==0))
        {
          flag=1;
          soft_right();
        }
      if((Right_white_line>50)&&(Right_white_line<70)&&(flag==0))
       {
          flag=1;
          soft_right();
       }
      if(Center_white_line>70 && Left_white_line>70 && Right_white_line>70)
        { 
          
          stop();
          buzz();
          update_curr_pos();

          if(curr_pos == 39)
         {
          all_xbee_clear();
         }
      
         if((curr_pos==req_pos) && (curr_pos!=41) && (curr_pos!=35))
         {
              forward_mm(210);
              event=1;
         }
         else if((curr_pos==req_pos) && (curr_pos==35) && (set_cover == 0))
         {
            forward_mm(70);
            ensure_orient(1);
            event = 0;
            set_cover = 50;
            array_index = 0;
         }
          else if((curr_pos==req_pos) && (curr_pos==41) && (set_cover == 50))
          {
             forward_mm(70);
             turn(0);
             forward_mm(100);
             servoDrop();
             UDR0 = 0x00;
             delay(100);
             UDR0 = 0x00;
             delay(100);
             UDR0 = 0x00;
             home_bit[0]=1;
             event=2;
         }
         else 
         {
           forward();
            velocity(150,150);
            _delay_ms(400);
         }
     }
  }
 }
}
/*
  * Function Name: navigate_0
  * Input: None
  * Output: None
  * Logic: This function is called for navigation of the bot from set0 to set1.  
  * Example Call: navigate_0()
  */
void navigate_0()
{
  while(curr_pos != 27)
  {
      Left_white_line = map(analogRead(left_white_line_sensor),30,450,0,150);
      Center_white_line =map(analogRead(center_white_line_sensor),30,450,0,150);
      Right_white_line = map(analogRead(right_white_line_sensor),30,450,0,150);
      flag=0;

      if((Left_white_line<5)&&(Right_white_line<5))
        {
          flag=1;
          forward();
          digitalWrite(46,HIGH);
          digitalWrite(45,HIGH);
        }

      if((Left_white_line>10)&&(Left_white_line<30)&&(flag==0))
        {
          flag=1;
          forward();
          velocity(150,150);  
        }
      if((Left_white_line>30)&&(Left_white_line<50)&&(flag==0))
        {
          flag=1;
          soft_left();    
        }
      if((Left_white_line>50)&&(Left_white_line<70)&&(flag==0))
        {
          flag=1;
          soft_left();
        }
      if((Right_white_line>10)&&(Right_white_line<30)&&(flag==0))
        {
          flag=1;
          forward();
          velocity(150,150); 
        }
      if((Right_white_line>30)&&(Right_white_line<50)&&(flag==0))
        {
          flag=1;
          soft_right();
        }
      if((Right_white_line>50)&&(Right_white_line<70)&&(flag==0))
       {
          flag=1;
          soft_right();
       }
      if(Center_white_line>70 && Left_white_line>70 && Right_white_line>70)
        { 
            
            stop();           
            buzz();
            update_curr_pos();
            lcd.setCursor(0,5);
            lcd.print(curr_pos);

            if(curr_pos!=27)
            { 
              forward();
             velocity(150,150);
             _delay_ms(400);
            }
            else
            {
             forward_mm(70);
             ensure_orient(3);
             array_index = 0;
             set_cover=0;
             ++set;
             event=0;
           } 
           if(curr_pos == 20)
           {
              all_xbee_clear();
           }
          }
        }
    }
   /* forward_mm(630);
    forward_mm(70);
    ensure_orient(3);
    array_index = 0;
    set_cover=0;
    ++set;
    curr_pos = 27;
    event=0;*/

/*
  * Function Name: navigate_1
  * Input: None
  * Output: None
  * Logic: This function is called for changing the bot's set from set1 to set2
  * Example Call: navigate_1()
  */
void navigate_1()
{
  while(curr_pos != 41)
  {
      Left_white_line = map(analogRead(left_white_line_sensor),30,450,0,150);
      Center_white_line =map(analogRead(center_white_line_sensor),30,450,0,150);
      Right_white_line = map(analogRead(right_white_line_sensor),30,450,0,150);
      flag=0;

      if((Left_white_line<5)&&(Right_white_line<5))
        {
          flag=1;
          forward();
          digitalWrite(46,HIGH);
          digitalWrite(45,HIGH);
        }

      if((Left_white_line>10)&&(Left_white_line<30)&&(flag==0))
        {
          flag=1;
          forward();
          velocity(150,150);  
        }
      if((Left_white_line>30)&&(Left_white_line<50)&&(flag==0))
        {
          flag=1;
          soft_left();    
        }
      if((Left_white_line>50)&&(Left_white_line<70)&&(flag==0))
        {
          flag=1;
          soft_left();
        }
      if((Right_white_line>10)&&(Right_white_line<30)&&(flag==0))
        {
          flag=1;
          forward();
          velocity(150,150); 
        }
      if((Right_white_line>30)&&(Right_white_line<50)&&(flag==0))
        {
          flag=1;
          soft_right();
        }
      if((Right_white_line>50)&&(Right_white_line<70)&&(flag==0))
       {
          flag=1;
          soft_right();
       }
      if(Center_white_line>70 && Left_white_line>70 && Right_white_line>70)
        { 
           
            stop();
            buzz();
            update_curr_pos();
            lcd.setCursor(0,5);
            lcd.print(curr_pos);

            if(curr_pos!=41)
            { 
             forward();
             velocity(150,150);
             delay(300);
            }
            else
            { 
             forward_mm(70); 
             ensure_orient(3);
             array_index = 0;
             set_cover=0;
             ++set;
             xbee_com=-1;
             event = 2;
           }
           if(curr_pos == 34)
           {
            all_xbee_clear();
           } 
        }
    }
    /*
    forward_mm(630);
    forward_mm(70); 
    ensure_orient(3);
    array_index = 0;
    set_cover=0;
    ++set;
    xbee_com=-1;
    event = 2;
    curr_pos = 41;*/
}
 /*
  * Function Name: final_home_run()
  * Input: None
  * Output: None
  * Logic: This function is called after the bots have finished traversing all their sets and takes the bot to its home position.
  * Example Call: final_home_run()
  */
void final_home_run()
{
  while(curr_pos != 52)
  {
      Left_white_line = map(analogRead(left_white_line_sensor),30,450,0,150);
      Center_white_line =map(analogRead(center_white_line_sensor),30,450,0,150);
      Right_white_line = map(analogRead(right_white_line_sensor),30,450,0,150);
      flag=0;

      if((Left_white_line<5)&&(Right_white_line<5))
        {
          flag=1;
          forward();
          digitalWrite(46,HIGH);
          digitalWrite(45,HIGH);
        }

      if((Left_white_line>10)&&(Left_white_line<30)&&(flag==0))
        {
          flag=1;
          forward();
          velocity(150,150);  
        }
      if((Left_white_line>30)&&(Left_white_line<50)&&(flag==0))
        {
          flag=1;
          soft_left();    
        }
      if((Left_white_line>50)&&(Left_white_line<70)&&(flag==0))
        {
          flag=1;
          soft_left();
        }
      if((Right_white_line>10)&&(Right_white_line<30)&&(flag==0))
        {
          flag=1;
          forward();
          velocity(150,150); 
        }
      if((Right_white_line>30)&&(Right_white_line<50)&&(flag==0))
        {
          flag=1;
          soft_right();
        }
      if((Right_white_line>50)&&(Right_white_line<70)&&(flag==0))
       {
          flag=1;
          soft_right();
       }
      if(Center_white_line>70 && Left_white_line>70 && Right_white_line>70)
        { 
            stop();  
            buzz();
            update_curr_pos();
            lcd.setCursor(0,5);
            lcd.print(curr_pos);

            if((curr_pos == 34) || (curr_pos == 31))
            { 
              forward_mm(70);
              turn(0);
            }
            else
            {
                forward();
                velocity(150,150);
                _delay_ms(400);
            } 
        }
    }
            
            
    
}
/*********************************************************************************************************************************************/
 /*
  * Function Name:servoPick
  * Input: None
  * Output: None
  * Logic: This function co-ordinates the movements of the top,mid and bottom servo motors 
           to pickup the stalk and deposit it in the collector box at the back of the bot.
  * Example Call: servoPick()
  */ 
  void servoPick(void)
    {
      forward_mm(20);
      servoBase.write(10);
      delay(500);
      servoMid.write(70);
      delay(500);
      servoTop.write(160);
      delay(500);
      for(i=70;i<=140;i=i+5)
       {
        servoMid.write(i);
        delay(50);
       }
      for(i=160;i>=60;i=i-3)
       {
        servoTop.write(i);
        delay(50);
       }
      for(i=140;i>=70;i=i-5)
       {
        servoMid.write(i);
        delay(50);
       }
      /*for(i=10;i<=95;i=i+5)
       {
        servoBase.write(i);
        delay(50);
       }*/
       servoBase.write(80); 
      delay(1000);
      servoTop.write(160);
      delay(400);
      servoBase.write(10);
      delay(500);
      servoMid.write(70);
      delay(500);
  }
/**********************************************************************************************************************************/
/*
  * Function Name:timer5_init
  * Input: None
  * Output: None
  * Logic: This function initializes the timer5 for pwm
  * Example Call: timer5_init()
  */ 
  void timer5_init()
    {
      TCCR5B = 0x00;  //Stop
      TCNT5H = 0xFF;  //Counter higher 8-bit value to which OCR5xH value is compared with
      TCNT5L = 0x01;  //Counter lower 8-bit value to which OCR5xH value is compared with
      OCR5AH = 0x00;  //Output compare register high value for Left Motor
      OCR5AL = 0xFF;  //Output compare register low value for Left Motor
      OCR5BH = 0x00;  //Output compare register high value for Right Motor
      OCR5BL = 0xFF;  //Output compare register low value for Right Motor
      OCR5CH = 0x00;  //Output compare register high value for Motor C1
      OCR5CL = 0xFF;  //Output compare register low value for Motor C1
      TCCR5A = 0xA9;  //{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
      TCCR5B = 0x0B;  //WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
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
    OCR5AL = (unsigned char)left_motor;
    OCR5BL = (unsigned char)right_motor;  
  }
  /*
  * Function Name:motion_pin_config
  * Input: None
  * Output: None
  * Logic: This function sets up motion pins and enables velocity control using pwm  
  * Example Call: motion_pin_config()
  */ 
  void motion_pin_config (void)
  {
    DDRA = DDRA | 0x0F;
    PORTA = PORTA & 0xF0;
    DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
    PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
  }
/*
  * Function Name:init_ports
  * Input: None
  * Output: None
  * Logic: This function calls motion_pin_config() and sets up the pins for motion of bot
  * Example Call: init_ports()
  */ 
  void init_ports()
    {
      motion_pin_config();
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
      unsigned char PortARestore = 0;
      Direction &= 0x0F;       // removing upper nibbel as it is not needed
      PortARestore = PORTA;      // reading the PORTA's original status
      PortARestore &= 0xF0;      // setting lower direction nibbel to 0
      PortARestore |= Direction;   // adding lower nibbel for direction command and restoring the PORTA status
      PORTA = PortARestore;      // setting the command to the port
    }
 /*
  * Function Name:forward
  * Input: None
  * Output: None
  * Logic: This function sets up the motion pins of the bot to set the bot in the forward direction
  * Example Call: forward()
  */ 
  void forward (void) 
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
  * Logic: This function sets up the motion pins of the bot so that  the bot takes a soft left turn
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
  * Function Name:color_init
  * Input: None
  * Output: None
  * Logic: This function sets up the  pins of the bot for color sensor
  * Example Call: color_init()
  */ 
  void color_init(void)
    {
    DDRD  = DDRD | 0xFE; //set PD0 as input for color sensor output
    PORTD = PORTD | 0x01;//Enable internal pull-up for PORTD 0 pin
    color_sensor_pin_interrupt_init();
    }
   
 /*
  * Function Name:uart0_init
  * Input: None
  * Output: None
  * Logic: This function initializes  the uart  register of the bot  for xbee comunication
  * Example Call: uart0_init()
  */ 
  void uart0_init(void)
  {
    UCSR0B = 0x00; //disable while setting baud rate
    UCSR0A = 0x00;
    UCSR0C = 0x06;
    UBRR0L = 0x5F; //set baud rate lo
    UBRR0H = 0x00; //set baud rate hi
    UCSR0B = 0x98;
   }
 /*
  * Function Name:init_devices
  * Input: None
  * Output: None
  * Logic: This function initializes almost all the ports necessary for the bot to execute the program
  * Example Call: init_devices()
  */ 
  void init_devices (void) //use this function to initialize all devices
    {
      init_ports();
      timer5_init();
      uart0_init();
      color_init();
      left_encoder_pin_config(); //left encoder pin config
      right_encoder_pin_config(); //right encoder pin config
      left_position_encoder_interrupt_init();
      right_position_encoder_interrupt_init();
      color_port_init(); 
      color_sensor_pin_interrupt_init();
    }
/***********************************************************************************************************************************************/
 /*
  * Function Name:irSense
  * Input: None
  * Output: None
  * Logic: This function is used to check for the proximity of plants to the bot.It returns 1 if the plant is nearby
  * Example Call: irSense()
  */ 
  boolean irSense(void)
    {
      digitalWrite(enable_irSensor,LOW);//low for enable 
      if(analogRead(ir4)<470)
        {
          digitalWrite(enable_irSensor,HIGH);//high for disable
          return true;
        }
      else
        {
          digitalWrite(enable_irSensor,HIGH);//high for disable
          return false;
        }  
     }
  
/*************************************************************************************************************************************/
 /*
  * Function Name:loop
  * Input: None
  * Output: None
  * Logic:It is equivalent to main(it's the first function ). This function repeatedly executes the main function so there is no need to encapsulate the code in main function in a loop. 
  * Example Call: loop()
  */ 
  void loop()
  { 
     Main();
  } 

/****************************************************************************************************************/
 /*
  * Function Name:ISR
  * Input: USART0_RX_vect
  * Output: None
  * Logic: This function defines the interrupt service routine to be executed when the bot receives data in the UDR.
           Based on the data received and xbee_com value, either homebit[] array is updated to keep the bot up to date on whether
           the other bots have finished their sets and update the arrays for keeping track of position of plants to be weeded.
  */ 
  
  ISR(USART0_RX_vect)     
    {
      data_rec = UDR0;
      if((data_rec == 0x40) && (red_xbee_flag ==0))
      {
        home_bit[1]=1;
        red_xbee_flag = 1;
      }
      if((data_rec == 0x80) && (blue_xbee_flag == 0))
      {
        home_bit[2]=1;
        blue_xbee_flag = 1;
      }
   
      if(xbee_com == 0)
        {
          data_temp = data_rec & 0xC0;
          if(data_temp == 0x40)
            {  
              buzz();
              data_temp_s1 = data_rec & 0x07;
              data_temp_s2 = data_rec & 0x38;
              r = convert_rowcol_to_seq(data_temp_s1,data_temp_s2);
              home_bit[1]=1;
              lcd.clear();
              lcd.setCursor(0,0);
              lcd.print(r);
             }
          if(data_temp == 0x80)
            {
              buzz();
              data_temp_s1 = data_rec & 0x07;
              data_temp_s2 = data_rec & 0x38;
              b = convert_rowcol_to_seq(data_temp_s1,data_temp_s2);
              home_bit[2] = 1;
              lcd.clear();
              lcd.setCursor(0,0);
              lcd.print(b);
            }       
         }
        if(xbee_com == 1)
          {
            data_temp = data_rec & 0xC0;
            uint8_t temp;
            if(data_temp == 0x00)
              {
                UDR0 = data_rec;
                data_temp_s1 = data_rec & 0x07;
                data_temp_s2 = data_rec & 0x38;
                temp=convert_rowcol_to_seq(data_temp_s1,data_temp_s2);
                if(set_cover==0)
                  {
                    if((temp>20) && (temp<28))
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
                         if((temp>20) && (temp<28))
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
 /***********************************************************************************************************************************************************************/ 
 /*
  * Function Name:set_zero_path
  * Input: None
  * Output: None
  * Logic: This function initializes initial_home_run[] array
  * Example Call: set_zero_path()
  */ 
  void set_zero_path()
    {
      for(j=0;j<10;++j)
      initial_home_run[j]=99;
    }
 /*
  * Function Name:all_bot_ready
  * Input: None
  * Output: None
  * Logic: This function  checks whether all the bots have finished their sets and returns 1 or 0; 
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
  * Logic: This function is used to reset home_bit[] array once all the bots have finished the set.
  * Example Call: all_bot_clear()
  */ 
   void all_bot_clear()
    {
      home_bit[0]=0;
      home_bit[1]=0;
      home_bit[2]=0;
    }
 /*
  * Function Name: convert_rowcol_to_seq
  * Input: None
  * Output: None
  * Logic: This function is used to convert the co-ordinates of the node to node number
  * Example Call:  convert_rowcol_to_seq(0x01,0x38)
  */ 
  unsigned int  convert_rowcol_to_seq(unsigned char x1,unsigned char x2)
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
  * Function Name:shiftr
  * Input: int b
  * Output: None
  * Logic: This function is used to shift the elements of initial_home_run[] array to introduce the position behind the current position of the bot
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
  * Function Name:reach_destination
  * Input: None
  * Output: None
  * Logic: This function is used to convert the row-column address of the bot to sequential node-wise address and stores this and orientation
  * Example Call: reach_destination(4,'A',0)
  */  
  void reach_destination(unsigned char s1, unsigned char s2 , unsigned int ini_orient)
    {   
      data = data & 0x00;
      set_zero_path();
      orient=ini_orient;
      curr_pos=convert_rowcol_to_seq(s1,s2);
      g = curr_pos;
      home_bit[0]=1;
    }
/*********************************************************************************************************************************************/
 
/********************************************************************************************************************************************/



//Function to configure INT4 (PORTE 4) pin as input for the left position encoder
 /*
  * Function Name:left_encoder_pin_config
  * Input: None
  * Output: None
  * Logic: This function is used to initialize and enable the necessary pins for left encoder to work
  * Example Call: lift_encoder_pin_config()
  */  
  void left_encoder_pin_config (void)
    {
      DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
      PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin
    }

//Function to configure INT5 (PORTE 5) pin as input for the right position encoder
  /*
  * Function Name:right_encoder_pin_config
  * Input: None
  * Output: None
  * Logic: This function is used to initialize and enable the necessary pins for right encoder to work
  * Example Call: right_encoder_pin_config()
  */ 
  void right_encoder_pin_config (void)
    {
      DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 4 pin as input
      PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 4 pin
    }
 /*
  * Function Name:left_position_encoder_interrupt_init
  * Input: None
  * Output: None
  * Logic: This function is used to initialize and enable the necessary pins for left encoder interrupt to work
  * Example Call: left_position_encoder_interrupt_init()
  */ 

  void left_position_encoder_interrupt_init (void) //Interrupt 4 enable
    {
      cli(); //Clears the global interrupt
      EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
      EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
      sei();   // Enables the global interrupt
    }
 /*
  * Function Name:right_position_encoder_interrupt_init
  * Input: None
  * Output: None
  * Logic: This function is used to initialize and enable the necessary pins for right encoder interrupt to work
  * Example Call: right_position_encoder_interrupt_init()
  */ 

  void right_position_encoder_interrupt_init (void) //Interrupt 5 enable
    {
      cli(); //Clears the global interrupt
      EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
      EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
      sei();   // Enables the global interrupt
    }
 //ISR for right position encoder
 /*
  * Function Name:ISR
  * Input: INT5_vect
  * Output: None
  * Logic: This function is used to increment the ShaftCount for right encoder to use 
  *        for rotating and traversal of the bot through the usage of encoder
  */ 
  
   ISR(INT5_vect)
     {
      ShaftCountRight++;  //increment right shaft position count
     }
 
//ISR for left position encoder
 /*
  * Function Name:ISR
  * Input: INT4_vect
  * Output: None
  * Logic: This function is used to increment the ShaftCount for left encoder to use 
  *        for rotating and traversal of the bot through the usage of encoder
  */ 

  ISR(INT4_vect)
    {
      ShaftCountLeft++;  
    }
 /*
  * Function Name:angle_rotate
  * Input: INT5_vect
  * Output: None
  * Logic: This function is used to rotate the bot according to specified degrees 
  *        for rotating  of the bot through the usage of encoder.
  * Example Call: angle_rotate(90)
  */ 

  void angle_rotate(unsigned int Degrees)
    {
      float ReqdShaftCount = 0;
      unsigned long int ReqdShaftCountInt = 0;
      ReqdShaftCount = (float) Degrees/ 4.090; // division by resolution to get shaft count
      ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
      ShaftCountRight = 0;
      ShaftCountLeft = 0;
      while (1)
       {
        if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
        break;
       }
      stop(); //Stop robot
    }

//Function used for moving robot forward by specified distance
 /*
  * Function Name:linear_distance_mm
  * Input: INT5_vect
  * Output: None
  * Logic: This function is used to make the bot traverse a certain distance according to specified distance 
  *        through the usage of encoder.
  * Example Call: linear_distance_mm(100)
  */ 

void linear_distance_mm(unsigned int DistanceInMM)
{
  float ReqdShaftCount = 0;
  unsigned long int ReqdShaftCountInt = 0;

  ReqdShaftCount = DistanceInMM / 5.338; // division by resolution to get shaft count
  ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
  
  ShaftCountRight = 0;
  while(1)
  { 
    Left_white_line = map(analogRead(left_white_line_sensor),30,450,0,150);
    Center_white_line =map(analogRead(center_white_line_sensor),30,450,0,150);
    Right_white_line = map(analogRead(right_white_line_sensor),30,450,0,150);
    printSensor();
      flag=0;

      if((Left_white_line<5)&&(Right_white_line<5))
      {
        flag=1;
        forward();
        digitalWrite(46,HIGH);
        digitalWrite(45,HIGH);
      }

      if((Left_white_line>10)&&(Left_white_line<30)&&(flag==0))
      {
        flag=1;
        forward();
        velocity(150,150);  
      }
      if((Left_white_line>30)&&(Left_white_line<50)&&(flag==0))
      {
        flag=1;
        soft_left();    
      }
      if((Left_white_line>50)&&(Left_white_line<70)&&(flag==0))
      {
        flag=1;
         soft_left();
      }
      if((Right_white_line>10)&&(Right_white_line<30)&&(flag==0))
      {
        flag=1;
        forward();
        velocity(150,150); 
      }
      if((Right_white_line>30)&&(Right_white_line<50)&&(flag==0))
      {
        flag=1;
       soft_right();
        
      }
      if((Right_white_line>50)&&(Right_white_line<70)&&(flag==0))
      {
        flag=1;
        soft_right();
      }
    if(ShaftCountRight > ReqdShaftCountInt)
    {
      break;
    }
  }
  stop(); //Stop robot
}
 /*
  * Function Name:forward_mm
  * Input: INT5_vect
  * Output: None
  * Logic: This function is used to make the bot traverse a certain distance in the FORWARD direction according to specified distance 
  *        through the usage of encoder.
  * Example Call: forward_mm(100)
  */ 
  void forward_mm(unsigned int DistanceInMM)
    {
      forward();
      linear_distance_mm(DistanceInMM);
    }
/*
  * Function Name:back_mm
  * Input: INT5_vect
  * Output: None
  * Logic: This function is used to make the bot traverse a certain distance in the BACKWARD direction according to specified distance 
  *        through the usage of encoder.
  * Example Call: forward_mm(100)
  */ 
  void back_mm(unsigned int DistanceInMM)
    {
      back();
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
     // 88 pulses for 360 degrees rotation 4.090 degrees per count
      left(); //Turn left
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
      // 88 pulses for 360 degrees rotation 4.090 degrees per count
      right(); //Turn right
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
        left_degrees(60);
        Center_white_line =map(analogRead(center_white_line_sensor),30,450,0,150);
        while(map(analogRead(center_white_line_sensor),30,450,0,150) < 100)
        {
            left();
            
        }
        stop();
    if(orient==0)orient=3;
    else
     {
      orient=orient-1;
     }
    lcd.setCursor(0,6);
    lcd.print(orient);

    }
      else if(direction==1)
      {
        right_degrees(60);
        Center_white_line =map(analogRead(center_white_line_sensor),30,450,0,150);
        while(map(analogRead(center_white_line_sensor),30,450,0,150) < 100)
        {
            right();
        }
        stop();
        orient=(orient+1)%4;
        lcd.setCursor(0,6);
        lcd.print(orient);  
       }
       else if(direction==2)
       {
          right_degrees(170);
          Center_white_line =map(analogRead(center_white_line_sensor),30,450,0,150);
          while(map(analogRead(center_white_line_sensor),30,450,0,150) < 100)
          {
              right();
          }
          stop();
          orient = (orient + 2)%4;
    
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
      while(req_orient!=orient)
      {
         turn(1);
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
           case 0:data = 0x09;flag_switch = 1; 
           case 1:if(flag_switch == 0)
                  {
                     data = 0x11;
                     flag_switch = 1;
                  }
           case 2:if(flag_switch == 0)
                  {
                     data = 0x19;
                     flag_switch = 1;
                  }
           case 3:if(flag_switch == 0)
                  {
                     data = 0x21;
                     flag_switch = 1;
                  }
          case 4:if(flag_switch == 0)
                  {
                     data = 0x29;
                     flag_switch = 1;
                  }
          case 5:if(flag_switch == 0)
                  {
                     data = 0x31;
                     flag_switch = 1;
                  }
          case 6:if(flag_switch == 0)
                  {
                     data = 0x39;
                     flag_switch = 1;
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
    data = data | 0x00;
    else if(send_flag == 1)
    data = data | 0x40;
    else if(send_flag == 2)
    data = data | 0x80;

    UDR0 = data ;
  
}

void GameOfThrones()
  {
    for(int i=0; i<4; i++)
    {
    tone(speakerPin, NOTE_G4);
    delay(500);
    noTone(speakerPin);
    tone(speakerPin, NOTE_C4);
    delay(500);
    noTone(speakerPin);
    tone(speakerPin, NOTE_DS4);
    delay(250);
    noTone(speakerPin);
    tone(speakerPin, NOTE_F4);
    delay(250);
    noTone(speakerPin);
    }
    for(int i=0; i<4; i++)
    {
    tone(speakerPin, NOTE_G4);
    delay(500);
    noTone(speakerPin);
    tone(speakerPin, NOTE_C4);
    delay(500);
    noTone(speakerPin);
    tone(speakerPin, NOTE_E4);
    delay(250);
    noTone(speakerPin);
    tone(speakerPin, NOTE_F4);
    delay(250);
    noTone(speakerPin);
    }
        tone(speakerPin, NOTE_G4);
        delay(500);
        noTone(speakerPin);
        tone(speakerPin, NOTE_C4);
        delay(500);
        tone(speakerPin, NOTE_DS4);
        delay(250);
        noTone(speakerPin);
        tone(speakerPin, NOTE_F4);
        delay(250);
        noTone(speakerPin);
        tone(speakerPin, NOTE_D4);
        delay(500);
        noTone(speakerPin);
    for(int i=0; i<3; i++)
    {
    tone(speakerPin, NOTE_G3);
    delay(500);
    noTone(speakerPin);
    tone(speakerPin, NOTE_AS3);
    delay(250);
    noTone(speakerPin);
    tone(speakerPin, NOTE_C4);
    delay(250);
    noTone(speakerPin);
    tone(speakerPin, NOTE_D4);
    delay(500);
    noTone(speakerPin);
    }//
        tone(speakerPin, NOTE_G3);
        delay(500);
        noTone(speakerPin);
        tone(speakerPin, NOTE_AS3);
        delay(250);
        noTone(speakerPin);
        tone(speakerPin, NOTE_C4);
        delay(250);
        noTone(speakerPin);
        tone(speakerPin, NOTE_D4);
        delay(1000);
        noTone(speakerPin);
        
        tone(speakerPin, NOTE_F4);
        delay(1000);
        noTone(speakerPin);
        tone(speakerPin, NOTE_AS3);
        delay(1000);
        noTone(speakerPin);
        tone(speakerPin, NOTE_DS4);
        delay(250);
        noTone(speakerPin);
        tone(speakerPin, NOTE_D4);
        delay(250);
        noTone(speakerPin);
        tone(speakerPin, NOTE_F4);
        delay(1000);
        noTone(speakerPin);
        tone(speakerPin, NOTE_AS3);
        delay(1000);
        noTone(speakerPin);
        tone(speakerPin, NOTE_DS4);
        delay(250);
        noTone(speakerPin);
        tone(speakerPin, NOTE_D4);
        delay(250);
        noTone(speakerPin);
        tone(speakerPin, NOTE_C4);
        delay(500);
        noTone(speakerPin);
    for(int i=0; i<3; i++)
    {
    tone(speakerPin, NOTE_GS3);
    delay(250);
    noTone(speakerPin);
    tone(speakerPin, NOTE_AS3);
    delay(250);
    noTone(speakerPin);
    tone(speakerPin, NOTE_C4);
    delay(500);
    noTone(speakerPin);
    tone(speakerPin, NOTE_F3);
    delay(500);
    noTone(speakerPin);
    }
          tone(speakerPin, NOTE_G4);
          delay(1000);
          noTone(speakerPin);
          tone(speakerPin, NOTE_C4);
          delay(1000);
          noTone(speakerPin);
          tone(speakerPin, NOTE_DS4);
          delay(250);
          noTone(speakerPin);
          tone(speakerPin, NOTE_F4);
          delay(250);
          noTone(speakerPin);
          tone(speakerPin, NOTE_G4);
          delay(1000);
          noTone(speakerPin);
          tone(speakerPin, NOTE_C4);
          delay(1000);
          noTone(speakerPin);
          tone(speakerPin, NOTE_DS4);
          delay(250);
          noTone(speakerPin);
          tone(speakerPin, NOTE_F4);
          delay(250);
          noTone(speakerPin);
          tone(speakerPin, NOTE_D4);
          delay(500);
          noTone(speakerPin);
    for(int i=0; i<4; i++)
    {
    tone(speakerPin, NOTE_G3);
    delay(500);
    noTone(speakerPin);
    tone(speakerPin, NOTE_AS3);
    delay(250);
    noTone(speakerPin);
    tone(speakerPin, NOTE_C4);
    delay(250);
    noTone(speakerPin);
    tone(speakerPin, NOTE_D4);
    delay(500);
    noTone(speakerPin);
    }
}

void all_xbee_clear()
{
  red_xbee_flag = 0;
  blue_xbee_flag = 0;
}
/********************************************************************************************************************************************/

/*
NOTE:
 
 cycle 0: This cycle deals with Dijkstra path calculation for all the three bots and sending the Dijkstra path of red 
          and blue bots respectively over xbee.
          
      event 0:This event continually checks for arrival of starting positions of the other two bots.
              Upon receiving the same it calls function finish() which calculates a collision free path for all 
              the three bots.  
      event 1:This transmits the red bot Dijkstra path to the same over xbee. 
      event 2:This transmits the blue bot Dijkstra path to the same over xbee.
      event 3:This event sends a confirmation of green Dijkstra path being ready to be followed by the bot over xbee.  
      event 4:This event waits for all the bots to confirm receiving of their respective Dijkstra path.
      event 5:This event executes the run_home() function which makes the bot traverse the Dijkstra path.
              On completing the Dijkstra path it sends the same info over xbee to other bots.  
      event 6:This event waits for all bots to have reached the starting nodes, i.e. 13 or 27 or 41, and 
              upon being confirmed increments cycle and sets event to 0. 
 
 cycle 1: This cycle deals with scanning the whole set blocks and finding the plant locations. 
          If the plant is of interest the bot will do the job, otherwise it will transmit the location to the concerned bot.
          
      event 0:Detects the presence of a plant by repeatedly calling the irSense() function, if found goes for event1 
              otherwise calls lie follow function for this set.
      event 1:Checks the color of the plant.
              This is followed by the task if the plant is of interest otherwise the location is communicated.
      event 2:Checks if all the bots have completed set0. On confirmation starts navigation to next set.
              After navigation checks if all have reached the next set starting position. On confirmation increments cycle.
 
 cycle 2: In this cycle since the bot knows the location of all the interested plants, it straight away goes to their location and does the job.
          If their are no plants of interest in the set the bot will simply go to other end of the set and come back to the start position of set, i.e. 21 and 27 respectively. 
      
      event 0:In this event the bot travels to the interested plant location if their are so, 
              otherwise it simply goes and comes back to start position. 
      event 1:Here the bot glows the RGB LED and picks up the weed plant stalk.
      event 2:This event is executed only upon being confirmed of all bots arrival at the start position of their respective sets after completely traversing the same. 
                  On being so, navigation to last set is started. 
          Once navigation is complete the bot again waits arrival of the other bots to start of their respective last sets.
 cycle 3: In this cycle since the bot knows the location of all the interested plants, it straight away goes to their location and does the job.
          If their are no plants of interest in the set the bot will simply go to other end of the set and come back to the start position of set, i.e. 35 and 41 respectively.
          
      event 0:In this event the bot travels to the interested plant location if their are so,
                  otherwise it simply goes and comes back to start position.
          event 1:Here the bot glows the RGB LED and picks up the weed plant stalk.
          event 2:This event is executed only upon being confirmed of all bots arrival at the start position of their respective sets after completely traversing the same.
                  On being so, final home run is executed.
                 
 cycle 4: Run for the home location is started by calling final_home_run() function.

 cycle 5: This cycle marks the end of task.           

*/

void Main(void)
{
  servoBase.write(10);
  servoMid.write(70);
  servoTop.write(160);
  delay(500); 
  reach_destination('4','D',0);
  
   while(1)
   {
     if(cycle==0)
     {
       if(event == 0)
       {
         if(all_bot_ready() == 1)
         {
           finish();
           all_bot_clear();
           all_xbee_clear();
           ++event;
           xbee_com = -1;
         }           
       }
       if(event == 1)
       {
          fire_for_red();
          ++event;  
       }
       if(event == 2)
       {
          fire_for_blue();
          ++event;
       }  
       if(event == 3)
       {
         ++event;
         UDR0 = 0x00;
         home_bit[0]=1;
       }
       if(event == 4)
       {
         if(all_bot_ready() == 1)
         {
           all_bot_clear();
           all_xbee_clear();
           ++event;
         }           
       }
       if(event == 5)
       {
         run_home();
         ++event;
         UDR0 = 0x00;
         delay(100);
         UDR0 = 0x00;
         delay(100);
         UDR0 = 0x00;
         home_bit[0]=1;
         forward_mm(70);
         ensure_orient(3);
       }
       if(event == 6)
       {
         if(all_bot_ready() == 1)
         {
           all_bot_clear();
           event = 0;
           ++cycle;
           xbee_com = 1;
         }           
       }                                               
     }
     else if(cycle == 1)
     {
         if(event == 0)
          { 
            if(curr_pos == 13 || curr_pos == 7)
           {
              irVal = analogRead(ir4); 
              forward_mm(80);
           }
            else
               forward_mm(150);
               
               stop(); 
               delay(100); 
               if(irVal-20 > analogRead(ir4))
               {
                   color_check();
                   event = 1; 
               }
               else
               {
                 event = 2;
                 forward();
                 
               }      
          }
         if(event == 1)
          {
              if(color == 1)  
              { 
                 digitalWrite(greenLED,HIGH);
                 delay(500);
                 servoPick();
                 digitalWrite(greenLED,LOW);
                 forward();
                 velocity(150,150);
                 delay(100);
              }
              else
              {
                 if(color == 0)      
                 {
                     digitalWrite(redLED,HIGH);
                     delay(500);
                     xbee_seq_node_transfer(curr_pos, 1);
                     digitalWrite(redLED,LOW);
                     forward();
                     velocity(150,150);
                     delay(100);
                 }             
                 else if(color == 2) 
                 {
                     digitalWrite(blueLED,HIGH);
                     delay(500);
                     xbee_seq_node_transfer(curr_pos, 2);
                     digitalWrite(blueLED,LOW);
                     forward();
                     velocity(150,150);
                     delay(100);
                 }             
              }  
              event = 2;        
           } 
           else if(event == 2)
           {
              line_follow_set0();
           }
           else if(event==3)
           {
            xbee_com = 1;
            set_cover = 50;
             if(all_bot_ready()==1)
              { 
                xbee_com = -1;
                cycle = 2;
                event = 0;
                all_bot_clear();
                navigate_0();
                stop();
                delay(2000);
                UDR0 = 0x00;
                delay(100);
                UDR0 = 0x00;
                delay(100);
                UDR0 = 0x00;
                home_bit[0] = 1;
                array_index = 0;
                while(all_bot_ready() == 0);
                all_bot_clear();
              }
          }                     
       }
       else if(cycle == 2)
       {
          if(event==0) 
          {
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print(curr_pos);
          lcd.print("  ");
          lcd.print(orient);
          lcd.print("  ");
          lcd.print(right_plant_set1[array_index]);
         if(set_cover==0)
         {
           if(right_plant_set1_count==0)line_follow_till_1(21);
           else
           {
             line_follow_till_1(right_plant_set1[array_index]);
           }
           
         }
         else if(set_cover==50)
         {
           if(left_plant_set1_count==0)line_follow_till_1(27);
           else
           {
             line_follow_till_1(left_plant_set1[array_index]);
           }
         }
       }
       else if(event==1)       
       {
         digitalWrite(greenLED,HIGH);
         servoPick();
         digitalWrite(greenLED,LOW);
         forward();
         velocity(150,150);
         delay(100);
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
           delay(2000);
           UDR0 = 0x00;
           delay(100);
           UDR0 = 0x00;
           delay(100);
           UDR0 = 0x00;
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
           if(right_plant_set2_count==0)line_follow_till_2(35);
           else
           {
             line_follow_till_2(right_plant_set2[array_index]);
           }
           
         }
         else if(set_cover==50)
         {
           if(left_plant_set2_count==0)line_follow_till_2(41);
           else
           {
             line_follow_till_2(left_plant_set2[array_index]);
           }
         }
       }
       else if(event==1)
       {
         digitalWrite(greenLED,HIGH);
         servoPick();
         digitalWrite(greenLED,LOW);
         forward();
         velocity(150,150);
         delay(100);
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
           forward();
           velocity(150,150);
           cycle = 4;
         }
       }         
       }
       else if(cycle == 4)
       {
         final_home_run();
         ++cycle;
       }
       else if(cycle == 5)
       {
          stop();
          buzz();
          buzz();
          delay(6000);
          ensure_orient(0);
          GameOfThrones();
          ++cycle;
          break;
       }
     }         
 }
/***********************************************************************************************************************/




struct Maze
{
    volatile int maze_array[Y][Y];
};

struct Graph
{
    volatile int graph_array[X][X];
};

struct Path_Array
{
  volatile int path[X];
};

int src;
struct Graph adjacency_matrix1;
struct Path_Array path1;
void dijkstra(int n,int startnode,int obst,int obst2,int obst3,int obst4);

/*
  * Function Name:set_zero
  * Input: None
  * Output: None
  * Logic: This function is used to to initialize all the members of tempGraph[][] array to 0.
  * Example Call: set_zero()
  */
void set_zero()
{  
  
    int i,j;
     for(i=0;i<49;++i)
    {
        for(j=0;j<49;++j)
        {
             tempGraph[i][j]=0;
        }
        
         
    }
}
/*
  * Function Name:wall_type
  * Input: int cval,int *d
  * Output: None
  * Logic: This function is used to find the walls of the cell 
  *        block according to the cell number.
  * Example Call: update_curr_pos()
  */

void wall_type(int cval,int *d)
  {
   switch(cval)
   {
   case 0:d[0]=0;d[1]=0;d[2]=0;d[3]=0;break;
   case 1:d[0]=1;d[1]=0;d[2]=0;d[3]=0;break;
   case 2:d[0]=0;d[1]=1;d[2]=0;d[3]=0;break;
   case 3:d[0]=1;d[1]=1;d[2]=0;d[3]=0;break;
   case 4:d[0]=0;d[1]=0;d[2]=1;d[3]=0;break;
   case 5:d[0]=1;d[1]=0;d[2]=1;d[3]=0;break;
   case 6:d[0]=0;d[1]=1;d[2]=1;d[3]=0;break;
   case 7:d[0]=1;d[1]=1;d[2]=1;d[3]=0;break;
   case 8:d[0]=0;d[1]=0;d[2]=0;d[3]=1;break;
   case 9:d[0]=1;d[1]=0;d[2]=0;d[3]=1;break;
   case 10:d[0]=0;d[1]=1;d[2]=0;d[3]=1;break;
   case 11:d[0]=1;d[1]=1;d[2]=0;d[3]=1;break;
   case 12:d[0]=0;d[1]=0;d[2]=1;d[3]=1;break;
   case 13:d[0]=1;d[1]=0;d[2]=1;d[3]=1;break;
   case 14:d[0]=0;d[1]=1;d[2]=1;d[3]=1;break;
   case 15:d[0]=1;d[1]=1;d[2]=1;d[3]=1;break;
   }
  }
 /*
  * Function Name:insert_row
  * Input: int i,int j,int *d
  * Output: None
  * Logic: This function is used to insert row into adjacency matrix.
  * Example Call: insert_row(4,3,dir)
  */  
void insert_row(int i,int j,int *d)
{
    int k=(i*7)+j;
    if(d[0]==0)
      tempGraph[k][k-7]=1;
    if(d[1]==0)
      tempGraph[k][k+1]=1;
    if(d[2]==0)
       tempGraph[k][k+7]=1;
    if(d[3]==0)
       tempGraph[k][k-1]=1;
}
/*
  * Function Name:buildGraph
  * Input: None
  * Output: None
  * Logic: It prepares an adjacency matrix graph using wall_type() and insert_row() functions 
  * Example Call: buildGraph();
  */ 
void buildGraph()
{
      
   //struct Graph adj_matrix;

    int i,j;
    int dir[4];
    set_zero();
   
    for(i=0;i<7;++i)
    {
        for(j=0;j<7;++j)
        {
            wall_type(tempmaze1[i][j],dir);
            insert_row(i,j,dir);
        }
    }

  //  return adj_matrix;
}
/*
  * Function Name:main_function
  * Input: int start,int finish,int obst,int obst2,int obst3,int obst4
  * Output: None
  * Logic: It accepts starting,final and all obstacle positions and calculates shortest path avoiding the obstacles using dijkstra() function 
  * Example Call: main_function(23,13,12,11,24,25);
  */ 
void main_function( int start, int finish,int obst,int obst2,int obst3,int obst4)//MAINFUNTION
{


    buildGraph();

    dijkstra(finish+1,start,obst,obst2,obst3,obst4);

}
/*
  * Function Name:dijkstra
  * Input: int n,int startnode,int obst,int obst2,int obst3,int obst4
  * Output: None
  * Logic: It calculates the shortest path from starting to final position avoiding obstacles 
  * Example Call: dijkstra(14,23,12,11,24,25);
  */ 
void dijkstra(int n,int startnode,int obst,int obst2,int obst3,int obst4)
{


    int distance[MAX],pred[MAX];
    int visited[MAX],counth,mindistance,nextnode,i,j;
    //pred[] stores the predecessor of each node
    //count gives the number of nodes seen so far
    //create the cost matrix
   int obstInd=0;
   int FinalObst[4];
   if(obst!=99)
   {
    FinalObst[obstInd]=obst;
    obstInd=obstInd+1;
    }
    if(obst2!=99)
   {
    FinalObst[obstInd]=obst2;
    obstInd=obstInd+1;
    }
    if(obst3!=99)
   {
    FinalObst[obstInd]=obst3;
    obstInd=obstInd+1;
    }
    if(obst4!=99)
   {
    FinalObst[obstInd]=obst4;
    obstInd=obstInd+1;
    }
     i=0;
   j=0; 
   int india;
     india=1;
    while(i<49)
    {
        while(j<49)
         
            {

              if(tempGraph[i][j]==0)
                cost[i][j]=INFI;
            else
                cost[i][j]=tempGraph[i][j];
                ++j;
            }
   //         lcd.clear();

     //     lcd.setCursor(0,0);
  //lcd.print(tempGraph[i][j]);
  //delay(10);
  //lcd.print(india);
  //delay(1000);
 //india++;
          ++i;
         j=0;   
    }
    //initialize pred[],distance[] and visited[]
       
   
    for(i=0;i<49;i++)
    {
        distance[i]=cost[startnode][i];
        pred[i]=startnode;
        visited[i]=0;
    }

    distance[startnode]=0;
    visited[startnode]=1;
    counth=1;

    while(counth<49-1)
    {
        mindistance=INFI;
  if (obstInd==0)
  {
    for(i=0;i<49;i++)
    if((distance[i]<mindistance)&&(!visited[i]))
    {
      mindistance=distance[i];
      nextnode=i;
    }

    //check if a better path exists through nextnode
    visited[nextnode]=1;
    for(i=0;i<49;i++)
    if((!visited[i]))
    if(((mindistance+cost[nextnode][i])<distance[i]))
    {
      distance[i]=mindistance+cost[nextnode][i];
      pred[i]=nextnode;
    }
  }
    
else if(obstInd==1)
{
        //nextnode gives the node at minimum distance
        for(i=0;i<49;i++)
            if((distance[i]<mindistance)&&(!visited[i])&&(i!=FinalObst[0]))
            {
                mindistance=distance[i];
                nextnode=i;
            }

            //check if a better path exists through nextnode
            visited[nextnode]=1;
            for(i=0;i<49;i++)
                if((!visited[i]))
                    if(((mindistance+cost[nextnode][i])<distance[i])&&(i!=FinalObst[0]))
                    {
                        distance[i]=mindistance+cost[nextnode][i];
                        pred[i]=nextnode;
                    }
}

else if(obstInd==2)
{
        //nextnode gives the node at minimum distance
        for(i=0;i<49;i++)
            if((distance[i]<mindistance)&&(!visited[i])&&(i!=FinalObst[0])&&(i!=FinalObst[1]))
            {
                mindistance=distance[i];
                nextnode=i;
            }

            //check if a better path exists through nextnode
            visited[nextnode]=1;
            for(i=0;i<49;i++)
                if((!visited[i]))
                    if(((mindistance+cost[nextnode][i])<distance[i])&&(i!=FinalObst[0])&&(i!=FinalObst[1]))
                    {
                        distance[i]=mindistance+cost[nextnode][i];
                        pred[i]=nextnode;
                    }
}

else if(obstInd==3)
{
        //nextnode gives the node at minimum distance
        for(i=0;i<49;i++)
            if((distance[i]<mindistance)&&(!visited[i])&&(i!=FinalObst[0])&&(i!=FinalObst[1])&&(i!=FinalObst[2]))
            {
                mindistance=distance[i];
                nextnode=i;
            }

            //check if a better path exists through nextnode
            visited[nextnode]=1;
            for(i=0;i<49;i++)
                if((!visited[i]))
                    if(((mindistance+cost[nextnode][i])<distance[i])&&(i!=FinalObst[0])&&(i!=FinalObst[1])&&(i!=FinalObst[2]))
                    {
                        distance[i]=mindistance+cost[nextnode][i];
                        pred[i]=nextnode;
                    }
}

else
{
        //nextnode gives the node at minimum distance
        for(i=0;i<49;i++)
            if((distance[i]<mindistance)&&(!visited[i])&&(i!=FinalObst[0])&&(i!=FinalObst[1])&&(i!=FinalObst[2])&&(i!=FinalObst[3]))
            {
                mindistance=distance[i];
                nextnode=i;
            }

            //check if a better path exists through nextnode
            visited[nextnode]=1;
            for(i=0;i<49;i++)
                if((!visited[i]))
                    if(((mindistance+cost[nextnode][i])<distance[i])&&(i!=FinalObst[0])&&(i!=FinalObst[1])&&(i!=FinalObst[2])&&(i!=FinalObst[3]))
                    {
                        distance[i]=mindistance+cost[nextnode][i];
                        pred[i]=nextnode;
                    }
}
        counth++;
    }

    //print the path and distance of each node  
  /*int gun=distance[n-1];
    gun=gun%10;*/
  
     int shortt[distance[n-1]];
  
   int k,start,endh,temp;
   
   k=0;
   
    i=n-1;
        if(i!=startnode)
        {
            j=i;
            do
            {
                j=pred[j];
                shortt[k]=j;
                k++;
            }while(j!=startnode);
    }
  
for( i = 0; i < X; i++)
    path1.path[i] = -1;
     start=0;
     endh=k-1;
      while (start < endh)
    {
        temp = shortt[start];
        shortt[start] = shortt[endh];
        shortt[endh] = temp;
        start++;
        endh--;
    }
  
            shortt[k]=n-1;
    for(i=0;i<=k;i++)
        path1.path[i]=shortt[i];
    

}
/*
  * Function Name:ReachDestinationAvoidingNode
  * Input: volatile int *pathg2,volatile int c,volatile int a,volatile int b,volatile int d,volatile int e,volatile int f
  * Output: None
  * Logic: It accepts the starting,final and obstacle positions,passes them to Dijkstra() function 
           using main_function() and stores the obtained path in *path2 array.
  * Example Call: ReachDestinationAvoidingNode(gpath,g,13,99,99,27,41);
  */ 
void ReachDestinationAvoidingNode(volatile int *path2,volatile int c,volatile int a,volatile int b,volatile int d,volatile int e,volatile int f)
{
  //TODO:: Please write your application code. You can make your own functions & header files

int start,endh,temp1;
int i=0,dest,obst,obst2,obst3,india,obst4,flag1=0,p,r;
  
//struct Maze maze;
/*dest=conversion(s1,s2);
obst=conversion(ob1,ob2);*/
dest=a;
obst=b;
obst2=d;
src=c;
obst3=e;
obst4=f;

if(src==obst)
obst=99;
if(src==obst2)
obst2=99;
if(src==obst3)
obst3=99;
if(src==obst4)
obst4=99;


if(src>dest)
{
  i=dest;
  dest=src;
  src=i;
  flag1=1;
}

 main_function(src,dest,obst,obst2,obst3,obst4);


if(flag1)
{
  for(i=0;path1.path[i]!=-1;++i);
  --i;


   start=0;
   endh=i;

   while (start < endh)
   {
     temp1 = path1.path[start];
     path1.path[start] = path1.path[endh];
     path1.path[endh] = temp1;
     start++;
     endh--;
     }
}
 int next=0;
    i=0;
while(next!=-1)
  {
    //printf("%d\n",path1.path[i]);
    *path2=path1.path[i];
    path2++;
    i=i+1;
    next=path1.path[i];
  }
  *path2=-1;
  
}

/*
  * Function Name:finish
  * Input: None
  * Output: None
  * Logic: It calculates the paths for red,blue and green bots using ReachDestinationAvoidingNode() and ensures these paths don't collide with each other.
  * Example Call: finish();
  */ 
void finish()
{
    
    
  //99 if ony one obstacle
  //Finding green path
  int ind=0;
    ind=10;
int obgreen;
obgreen=99;
if(r==48)
obgreen=47;
if(r==6&&g==5)
obgreen=6;
if(b==6&&g==5)
obgreen=6;
  ReachDestinationAvoidingNode(gpath,g,13,99,obgreen,27,41);


    

 

  //Finding red path
  temp=gpath[1];
  int obred1,obred2,obred3;
  obred1=13;
  obred2=41;
  obred3=99;
  if((r==6)&&(temp==5))
  obred1=99;
  if(b==6)
  obred3=5;
  ReachDestinationAvoidingNode(rpath,r,27,temp,obred3,obred1,obred2);

  
  ind=0;
  while((gpath[ind+1]!=13)&&(rpath[ind+1]!=27))
  {
  
    if((gpath[ind]==rpath[ind+1])&&(gpath[ind+1]==rpath[ind]))
    {
      temp=rpath[ind+1];
      ReachDestinationAvoidingNode(arrpath,rpath[ind],27,temp,obred3,obred1,obred2);
      i2=ind;
      i3=0;
      do
      {
        rpath[i2]=arrpath[i3];
        i2=i2+1;
        i3=i3+1;
      }while(arrpath[i3-1]!=-1);
      ind=ind+1;
      continue;
    }
    if(gpath[ind+1]==rpath[ind+1])
    {
      temp=rpath[ind+1];
      ReachDestinationAvoidingNode(arrpath,rpath[ind],27,temp,obred3,obred1,obred2);
      i2=ind;
      i3=0;
      do
      {
        rpath[i2]=arrpath[i3];
        i2=i2+1;
        i3=i3+1;
      }while(arrpath[i3-1]!=-1);
      ind=ind+1;
      continue;
    }
    ind=ind+1;
    
  }

//Finding Blue path
int obblue,obblue2,obblue3;
obblue2=27;
obblue3=13;
if((b==20)&&(gpath[1]==19))
obblue3=99;
if((b==20)&&(rpath[1]==19))
obblue2=99;
if((b==6)&&(gpath[1]==5))
obblue3=99;

//+++++++++++++++++++++++++++++++++++++++++++
if((b==20)&&(r==13)&&(g==26))
{
  obblue2=99;
  obblue3=99;
}
if((b==20)&&(r==12)&&(g==13))
obblue2=99;
if((b==13)&&(r==5))
obblue2=99;
//+++++++++++++++++++++++++++++++++++++++++++
ReachDestinationAvoidingNode(bpath,b,41,gpath[1],rpath[1],obblue2,obblue3);

ind =0;
while(((gpath[ind+1]!=13)&&(bpath[ind+1]!=41))||((rpath[ind+1]!=27)&&(bpath[ind+1]!=41)))
{
  if((gpath[ind]==bpath[ind+1])&&(gpath[ind+1]==bpath[ind]))
    {
      temp=bpath[ind+1];
      if(rpath[ind+1]!=-1)
      ReachDestinationAvoidingNode(arrpath,bpath[ind],41,temp,rpath[ind+1],obblue2,obblue3);
      else
      ReachDestinationAvoidingNode(arrpath,bpath[ind],41,temp,99,obblue2,obblue3);
      i2=ind;
      i3=0;
      do
      {
        bpath[i2]=arrpath[i3];
        i2=i2+1;
        i3=i3+1;
      }while(arrpath[i3-1]!=-1);
      ind=ind+1;
      continue;
    }
    if((rpath[ind]==bpath[ind+1])&&(rpath[ind+1]==bpath[ind]))
    {
      temp=bpath[ind+1];
      if(gpath[ind+1]!=-1)
      ReachDestinationAvoidingNode(arrpath,bpath[ind],41,temp,gpath[ind+1],obblue2,obblue3);
      else
      ReachDestinationAvoidingNode(arrpath,bpath[ind],41,temp,99,obblue2,obblue3);
      i2=ind;
      i3=0;
      do
      {
        bpath[i2]=arrpath[i3];
        i2=i2+1;
        i3=i3+1;
      }while(arrpath[i3-1]!=-1);
      ind=ind+1;
      continue;
    }
    if(gpath[ind+1]==bpath[ind+1])
    {
      temp=bpath[ind+1];
      if(rpath[ind+1]!=-1)
      ReachDestinationAvoidingNode(arrpath,bpath[ind],41,temp,rpath[ind+1],obblue2,obblue3);
      else
      ReachDestinationAvoidingNode(arrpath,bpath[ind],41,temp,99,obblue2,obblue3);
      i2=ind;
      i3=0;
      do
      {
      bpath[i2]=arrpath[i3];
        i2=i2+1;
        i3=i3+1;
      }while(arrpath[i3-1]!=-1);
      ind=ind+1;
      continue;
    }
    if(rpath[ind+1]==bpath[ind+1])
    {
      temp=bpath[ind+1];
      if(gpath[ind+1]!=-1)
      ReachDestinationAvoidingNode(arrpath,bpath[ind],41,temp,gpath[ind+1],obblue2,obblue3);
      else
      ReachDestinationAvoidingNode(arrpath,bpath[ind],41,temp,99,obblue2,obblue3);
      i2=ind;
      i3=0;
      do
      {
      bpath[i2]=arrpath[i3];
        i2=i2+1;
        i3=i3+1;
      }while(arrpath[i3-1]!=-1);
      ind=ind+1;
      continue;
    }
      ind=ind+1;
     
}

}


/*
  * Function Name:shiftg
  * Input: int b
  * Output: None
  * Logic: This function is used to shift the elements of initiial_home_run[] array  of 
           green  bot to introduce the position behind the current position of the bot
  * Example Call: shiftg(26)
  */

void shiftg(int img_prev)
{
int i,n;
i=0;
n=0;
do{
  n=n+1;
  i=i+1;
}while(gpath[i]!=-1);
for(i=n+1 ; i>=1 ; i--)
    gpath[i]=gpath[i-1] ;
  gpath[0]=img_prev ;

}


/***********************************************************************************************************************/
/*
  * Function Name:run_home
  * Input: None
  * Output: None
  * Logic: This function gets the required path from its initial position to the 
           beginning position of set0 and implements the required movements of the bot
           as per the path and orientation of the  bot.
  * Example Call: run_home()
  */
void run_home()
{
  int temp;
  if(orient==0)
  temp=gpath[0]+7;
  else if(orient==1)
  temp=gpath[0]-1;
  else if(orient==2)
  temp=gpath[0]-7;
  else if(orient==3)
  temp=gpath[0]+1;
  
  shiftg(temp);
  curr_pos = temp;
  
  int prev,present,next,m;
  prev=gpath[0];
  present=gpath[1];
  next=gpath[2];
  m=1;

  
      update_curr_pos();
      
       if(next-present==1)
       {
         if(present-prev==1)
         {forward();velocity(150,150);delay(400);}
         if(present-prev==7)
         {
          forward();
          delay(300);
          turn(0);
         }
         if(present-prev==-7)
         {forward();
          delay(300);
          turn(1);
         }
           if(present-prev==-1)
         {forward();
          delay(500);//<--------here
           turn(2); 
           }
         
         m++;
         prev=gpath[m-1];
         present=gpath[m];
         next=gpath[m+1];
       }
       
       else if(next-present==7)
       {
         if(present-prev==7)
            {forward();velocity(150,150);delay(400);}
         if(present-prev==-1)
         {forward();
          delay(300);
          turn(0);
         }
         if(present-prev==1)
         {forward();
          delay(300);
          turn(1);
         }
         if(present-prev==-7)
         {forward();
          delay(300);
          turn(2);
         }
         
         m++;
         prev=gpath[m-1];
         present=gpath[m];
         next=gpath[m+1];
       }         
       
       else if(next-present==-7)
       {
         if(present-prev==-7)
         {forward();velocity(150,150);delay(400);}
         if(present-prev==1)
         {forward();
          delay(300);
          turn(0);
         }
         if(present-prev==-1)
         {forward();
          delay(300);
          turn(1);
         }
         if(present-prev==7)
         {forward();
          delay(300);
          turn(2);
         }
         
         m=m+1;
         prev=gpath[m-1];
         present=gpath[m];
         next=gpath[m+1];
       }
       
       else if(next-present==-1)
       {
         if(present-prev==-1)
           {forward();velocity(150,150);delay(400);}
         if(present-prev==-7)
         {forward();
          delay(300);
          turn(0);
         }
         if(present-prev==7)
         {forward();
          delay(300);
          turn(1);
         }
         if(present-prev==1)
         {forward();
          delay(300);
           turn(2);
         }

         m=m+1;
         prev=gpath[m-1];
         present=gpath[m];
         next=gpath[m+1];
       }                 
      forward();
      velocity(150,150);
      _delay_ms(100);  

      stop();
      delay(2000);
      forward();
      velocity(150,150);
      _delay_ms(100);
   
    while(gpath[m]!=-1)
    {
     Left_white_line = map(analogRead(left_white_line_sensor),30,450,0,150);
    Center_white_line =map(analogRead(center_white_line_sensor),30,450,0,150);
    Right_white_line = map(analogRead(right_white_line_sensor),30,450,0,150);
     flag=0;
     lcd.setCursor(1,1);
     
     lcd.print(prev);
     lcd.print(" ");
     lcd.print(present);
     lcd.print(" ");
     lcd.print(next);
         
     if((Left_white_line<5)&&(Right_white_line<5))
      {
        flag=1;
        forward();
        digitalWrite(46,HIGH);
        digitalWrite(45,HIGH);
      }

      if((Left_white_line>10)&&(Left_white_line<30)&&(flag==0))
      {
        flag=1;
        forward();
        velocity(150,150);  
      }
      if((Left_white_line>30)&&(Left_white_line<50)&&(flag==0))
      {
        flag=1;
        soft_left();    
      }
      if((Left_white_line>50)&&(Left_white_line<70)&&(flag==0))
      {
        flag=1;
         soft_left();
      }
      if((Right_white_line>10)&&(Right_white_line<30)&&(flag==0))
      {
        flag=1;
        forward();
        velocity(150,150); 
      }
      if((Right_white_line>30)&&(Right_white_line<50)&&(flag==0))
      {
        flag=1;
       soft_right();
        
      }
      if((Right_white_line>50)&&(Right_white_line<70)&&(flag==0))
      {
        flag=1;
        soft_right();
      }
      if(Center_white_line>70 && Left_white_line>70 && Right_white_line>70)
       { 
        stop(); 
        buzz();
        delay(2000);
       update_curr_pos();
       lcd.setCursor(0,0);
       lcd.print(curr_pos);
       lcd.print("  ");
       lcd.print(orient);
        _delay_ms(200);
       //------>stop();
      
      
       if(next==-1)
       {
         break;
         }
       
       if(next-present==1)
       {
         if(present-prev==1)
            {forward();velocity(150,150);delay(400);}
         if(present-prev==7)
         {
          forward();
          delay(300);
         turn(0);
         }
         if(present-prev==-7)
         {forward();
          delay(300);
          turn(1);
         }
           if(present-prev==-1)
         {forward();
          delay(500);//<--------here
           turn(2); 
           }
         
         m++;
         prev=gpath[m-1];
         present=gpath[m];
         next=gpath[m+1];
       }
       
       else if(next-present==7)
       {
         if(present-prev==7)
            {forward();velocity(150,150);delay(400);}
         if(present-prev==-1)
         {forward();
          delay(300);
          turn(0);
         }
         if(present-prev==1)
         {forward();
          delay(300);
          turn(1);
         }
         if(present-prev==-7)
         {forward();
          delay(300);
           turn(2);
         }
         
         m++;
         prev=gpath[m-1];
         present=gpath[m];
         next=gpath[m+1];
       }         
       
       else if(next-present==-7)
       {
         if(present-prev==-7)
         {forward();velocity(150,150);delay(400);}
         if(present-prev==1)
         {forward();
          delay(300);
          turn(0);
         }
         if(present-prev==-1)
         {forward();
          delay(300);
          turn(1);
         }
         if(present-prev==7)
         {forward();
          delay(300);
          turn(2);
         }
         
         m=m+1;
         prev=gpath[m-1];
         present=gpath[m];
         next=gpath[m+1];
       }
       
       else if(next-present==-1)
       {
         if(present-prev==-1)
           {forward();velocity(150,150);delay(400);}
         if(present-prev==-7)
         {forward();
          delay(300);
          turn(0);
         }
         if(present-prev==7)
         {forward();
          delay(300);
          turn(1);
         }
         if(present-prev==1)
         {forward();
          delay(300);
          turn(2);
         }

         m=m+1;
         prev=gpath[m-1];
         present=gpath[m];
         next=gpath[m+1];
       }                 
      forward();
      velocity(150,150);
      _delay_ms(100);  
      }
      //ir_check();
     } 
  }      
/***********************************************************************************************************************/
/*
  * Function Name:fire_for_red
  * Input: None
  * Output: None
  * Logic: This function is used to transmit to the red bot its path 
            from its initial position to 27  through Xbee
  * Example Call: fire_for_red()
  */ 
void fire_for_red()
{
    array_index=0;
    do
    {   
      
        delay(100);
        data = 0x00;
        temp_tx = rpath[array_index];
          flag_switch=0;
            switch(temp_tx%7)
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
          switch((temp_tx/7)+1)
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
          ++array_index;
          data = data | 0x40;

          UDR0 = data ;
    }while(rpath[array_index-1]!=27);
    array_index = 0;
}

 /*
  * Function Name:fire_for_blue 
  * Input: None
  * Output: None
  * Logic: This function is used to transmit to the red bot its path 
           from its initial position to 41  through Xbee
  * Example Call: fire_for_blue()
  */ 
void fire_for_blue()
{
  data = 0x00;
    array_index=0;
    do
    {  
        delay(100);
        data = 0x00;
        temp_tx = bpath[array_index];
          flag_switch=0;
            switch(temp_tx%7)
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
          switch((temp_tx/7)+1)
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
           ++array_index;
          data = data | 0x80;

          UDR0 = data ;
    }while(bpath[array_index-1]!=41);
    array_index = 0;
}

/*******************************************************************************************************************************************************************/
