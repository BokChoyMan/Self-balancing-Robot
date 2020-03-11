#include <Wire.h>
#include "mcu6050.h"
#include <FlexiTimer2.h>

//===============INSTANCE_VARIABLES==============//

//Motor pin declarations//
int rDir = 3;    //Right direction.
int lDir = 6;    //Left direction.

int rStep = 2;   //Right step.
int lStep = 5;   //Left step.

const int DRIVER_MODE = 1;


//Data collection variables//
float rad_to_deg = 180/M_PI;
float angle_deg, dot_angle_deg;
const int MICROSECONDS = 1*10^(-6);
int loopNumber = 1;

//=================PID_VARIABLES==================//

float pid_p, pid_i, pid_d, PID, error;

float kp=25;   //proportion**
float ki=0;    //integral
float kd=2;    //derivative**

float desired_angle = 0; //Ideal/Target Angle

//=============TIME-BASED_VARIABLES==============//

//Timer variables for runtime from t0 => t1//
long t=0;       //The change in time within the loop.
long n=0;       //Sample size.

//Period microstepping of the motor//
boolean state = LOW;   //manipulates the HIGH and LOW pulse of PWM signal.
float set_delay_us = 3000;   //arbitrary default, beginning delay of 3000us. 
                                   
//=====================SETUP=====================//


void setup() {
  //Declare pins as output//
  Serial.begin(115200);
  pinMode(rStep, OUTPUT);
  pinMode(rDir, OUTPUT);
  pinMode(lStep, OUTPUT);
  pinMode(lDir, OUTPUT);

  //Initialize MPU6050//
  startMCU6050(); 

  FlexiTimer2::set(set_delay_us,MICROSECONDS, moveMotor);
  FlexiTimer2::start();
}

//======================LOOP======================//
                                                     
void loop() {
  long t0=micros();  //begin timer.

  ///Reads ACC and GYRO values from MPU6050//
  readMCU6050data(); 
  
  //Calculates angles used for roll//
  angle_deg = getMCU6050_fused_angleY()*rad_to_deg;       //roll angle.
  dot_angle_deg = getMCU6050_gyro_rateY()*rad_to_deg;     //the derivative of roll angle.
  
  //Difference from desired angle//
  error = angle_deg - desired_angle;
  
  //Proportion, Integral, Derivative Control loop//
  pid_p = kp*error;
  pid_i += ki*error;
  pid_d = kd*dot_angle_deg;
  PID = pid_p + pid_i + pid_d;
  
  set_delay_us = 4000/abs(PID);

  if(loopNumber == 1000)      //every 1000 loops, reset delay time for motors. Also allows motor to make 500 steps.
  {
    FlexiTimer2::stop();      //stop current ISR.
    loopNumber = 1;           //reset loop iteration.
    FlexiTimer2::set(set_delay_us, MICROSECONDS, moveMotor);    //reset delay time of ISR.
    FlexiTimer2::start();                                       //Begin ISR again with new delay time.
  }
  
  long t1=micros();  //end timer.
  //runtime(t0,t1);
  
  loopNumber++;
}

//=====================METHODS======================//
void moveMotor()
{
   if(angle_deg > 45 || angle_deg < -45)
  {
    halt();
  }
  
  else if(angle_deg < 0)
  {
    clockw();   
  }
  
  else
  {
    counter();
  }
}

//Clockwise movement//
void clockw()
{
  // Set the spinning direction clockwise:
  digitalWrite(rDir, HIGH);
  digitalWrite(lDir, HIGH);

  digitalWrite(rStep, state);
  digitalWrite(lStep, state);
  state = !state;
}

//Counter-clockwise movement//
void counter()
{
  // Set the spinning direction counterwise:
  digitalWrite(rDir, LOW);
  digitalWrite(lDir, LOW);

  digitalWrite(rStep, state);
  digitalWrite(lStep, state);
  state = !state;
 
}

//Halt movement//
void halt()
{
  digitalWrite(rStep, LOW);
  digitalWrite(lStep, LOW);
}

//Run-time Timer//
void runtime(long t0, long t1)
{
  t+=(t1-t0); 
  n ++;
  if (n==1000)
  {
    Serial.println(t/n); //prints the run time average from (t1 => t0) every n loops.
    n=0;   //reset n;
    t=0;   //reset timer;
  }
}
