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
float rad_to_deg = 180/3.141592654;

//1.8 degrees per step, 200 steps 1 revolution//
int stepsPerRevolution = 1675; 

//Next step of motor in microseconds//
long nextStep_us; 

//PID variables//
float pid_p=0;
float pid_i=0;
float pid_d=0;

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
  
  //Begin I2C Communications//
  Wire.begin(); 
  
  //maximum frequency for arduino is 400k Hz, fastest processing clock.
  Wire.setClock(400000);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

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
long delay_us =  2000;   //delay cannot be less than 900 us because of overlap.

long period_us = delay_us*2;   /* 
                                  2 time periods for HIGH/LOW pulse,
                                  LOWER Period the faster, HIGHER Period the slower
                               */ 
//=======================LOOP=======================//
                                                     
void loop() {
  
  long t0=micros();  //begin timer.

  //Reads ACC and GYRO values from MPU6050//
  readMCU6050data(); 

  //Calculates angles used for roll//
  float angle_rad = getMCU6050_fused_angleY();       //roll angle.
  float dot_angle_rad = getMCU6050_gyro_rateY();     //the derivative of roll angle.
  
  long t1=micros();  //end timer.
  
  //Sending PWM signal to motors//
  clockw();
  
 //_Run-timer_//
  t+=(t1-t0); 
  n ++;
  if (n==1000)
  {
    Serial.println(t/n); //prints the run time average from (t1 => t0) every n loops.
    n=0;   //reset n;
    t=0;   //reset timer;
  }
}

//=====================METHODS======================//

//Clockwise movement//
void clockw()
{
  if (micros()>=nextStep_us) 
  {
    // Set the spinning direction clockwise:
    digitalWrite(rDir, HIGH);
    digitalWrite(lDir, HIGH);

    digitalWrite(rStep, state);
    digitalWrite(lStep, state);
    state = ! state;
    nextStep_us += period_us;
      
  }
}

//Counter-clockwise movement//
void counter()
{
  if (micros()>=nextStep_us) 
  {
    // Set the spinning direction counterwise:
    digitalWrite(rDir, LOW);
    digitalWrite(lDir, LOW);

    digitalWrite(rStep, state);
    digitalWrite(lStep, state);
    state = ! state;
    nextStep_us += period_us;
      
  }
}

//Halt movement//
void halt()
{
  if (micros()>=nextStep_us) 
  {
    digitalWrite(rStep, LOW);
    digitalWrite(lStep, LOW);
    nextStep_us += period_us;
  }
}
