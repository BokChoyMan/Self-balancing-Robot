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
float error;

//1.8 degrees per step, 200 steps 1 revolution//
int stepsPerRevolution = 1675; 

//Next step of motor in microseconds//
long nextStep_us; 

//PID variables//
float pid_p, pid_i, pid_d, PID;

//PID Constants//
float kp=0;   //proportion
float ki=0;   //integral
float kd=0;   //derivative

//Target Roll Angle//
float desired_angle = 0;

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

  //Begin timer for motor//
  nextStep_us=micros();  
}

//=================TIMER_VARIABLES==================//

//Declare timer variables for runtime from t0 => t1//
long t=0;       //The change in time within the loop.
long n=0;       //Sample size.

//Period microstepping of the motor//
boolean state = LOW;     //manipulates the HIGH and LOW pulse of PWM signal.
long steps = 1675;       //Steps in each period.
long delay_t =  2000;   //delay cannot be less than 900 us because of overlap.

long period_us = delay_t*2;   /* 
                                  2 time periods for HIGH/LOW pulse,
                                  LOWER Period the faster, HIGHER Period the slower
                               */ 
int j=0;
//=======================LOOP=======================//
                                                     
void loop() {
  
  long t0=micros();  //begin timer.

  //Reads ACC and GYRO values from MPU6050//
  readMCU6050data(); 

  //Calculates angles used for roll//
  float angle_deg = getMCU6050_fused_angleY()*rad_to_deg;       //roll angle.
  float dot_angle_deg = getMCU6050_gyro_rateY()*rad_to_deg;     //the derivative of roll angle.
  
  //Difference from desired angle
  error = desired_angle - angle_deg;

  //Proportion, Integral, Derivative Control loop//
  pid_p = kp*error;
  pid_i += ki*error;
  pid_d = kd*dot_angle_deg;
  PID = pid_p + pid_i + pid_d;

  Serial.print("p: ");
  Serial.println(pid_p);
  Serial.print("d: ");
  Serial.println(pid_d);
  Serial.print("PID: ");
  Serial.println(PID);
  Serial.print("Angle: ");
  Serial.println(angle_deg);
  Serial.print("Error: ");
  Serial.println(error);
  delay(500);
  //Sending PWM signal to motors//
  if(angle_deg < 0)
  {
    clockw(5000/abs(PID));   
  }
  
  if(angle_deg > 0)
  {
    counter(5000/abs(PID));
  }
  
  if(angle_deg > 45 || angle_deg < -45)
  {
    halt();
  }
  
  long t1=micros();  //end timer.
 //==Run-timer==//
  t+=(t1-t0); 
  n ++;
  if (n==1000)
  {
    //Serial.println(t/n); //prints the run time average from (t1 => t0) every n loops.
    n=0;   //reset n;
    t=0;   //reset timer;
  }
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
