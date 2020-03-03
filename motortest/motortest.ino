#include <Wire.h>
#include "mcu6050.h"

//===============INSTANCE_VARIABLES==============//

//Motor pin declarations//
int rDir = 3;    //Right direction.
int lDir = 6;    //Left direction.

int rStep = 2;   //Right step.
int lStep = 5;   //Left step.

//Data collection variables//
int16_t Acc_rawX, Acc_rawY, Acc_rawZ;
float rad_to_deg = 180/M_PI;

//=================PID_VARIABLES==================//

float pid_p, pid_i, pid_d, PID, error;

float kp=25;   //proportion
float ki=0;    //integral
float kd=2;    //derivative

float desired_angle = 0; //Ideal/Target Angle

//=============TIME-BASED_VARIABLES==============//

//Timer variables for runtime from t0 => t1//
long t=0;       //The change in time within the loop.
long n=0;       //Sample size.
long j=0;       //some time variable.

//Period microstepping of the motor//
boolean state = LOW;   //manipulates the HIGH and LOW pulse of PWM signal.
long nextStep_us;      //Next step of motor in microseconds

//=====================SETUP=====================//

void setup() {
  //Declare pins as output//
  Serial.begin(115200);
  pinMode(rStep, OUTPUT);
  pinMode(rDir, OUTPUT);
  pinMode(lStep, OUTPUT);
  pinMode(lDir, OUTPUT);

  //Initialize MPU6050//
  startMCU6050;

  //Begin timer for motor//
  nextStep_us=micros();  
}

//======================LOOP======================//
                                                     
void loop() {
  //long t0=micros();  //begin timer.

  //Reads ACC and GYRO values from MPU6050//
  readMCU6050data(); 

  //Calculates angles used for roll//
  float angle_deg = getMCU6050_fused_angleY()*rad_to_deg;       //roll angle.
  float dot_angle_deg = getMCU6050_gyro_rateY()*rad_to_deg;     //the derivative of roll angle.

  //Difference from desired angle//
  error = angle_deg - desired_angle;

  //Proportion, Integral, Derivative Control loop//
  pid_p = kp*error;
  pid_i += ki*error;
  pid_d = kd*dot_angle_deg;
  PID = pid_p + pid_i + pid_d;
  
  float set_delay_us = 3200/abs(PID);
  
  //Sending PWM signal to motors//
   if(angle_deg > 45 || angle_deg < -45)
  {
    halt();
  }
  
  else if(angle_deg < 0)
  {
    clockw(set_delay_us);   
  }
  
  else
  {
    counter(set_delay_us);
  }
  /*
  long t1=micros();  //end timer.
  runtime(t0,t1);
  */
}

//=====================METHODS======================//

//Clockwise movement//
void clockw(long delay_us)
{
  if (micros()>=nextStep_us) 
  {
    // Set the spinning direction clockwise:
    digitalWrite(rDir, HIGH);
    digitalWrite(lDir, HIGH);

    digitalWrite(rStep, state);
    digitalWrite(lStep, state);
    state = ! state;
    nextStep_us += delay_us;

  }
}

//Counter-clockwise movement//
void counter(long delay_us)
{
  if (micros()>=nextStep_us) 
  {
    // Set the spinning direction counterwise:
    digitalWrite(rDir, LOW);
    digitalWrite(lDir, LOW);

    digitalWrite(rStep, state);
    digitalWrite(lStep, state);
    state = ! state;
    nextStep_us += delay_us;
      
  }
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
