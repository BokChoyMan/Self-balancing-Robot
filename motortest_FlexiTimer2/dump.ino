//Reads values using I2C
/* 
  Wire.beginTransmission(0x68);
  
  Wire.write(0x3B); 
  Wire.endTransmission(false);
  
  Wire.requestFrom(0x68,6,true);
  
////////////////////PULLING RAW ACCELEROMETER DATA FROM IMU///////////////// 
  Acc_rawX=Wire.read()<<8|Wire.read(); 
  Acc_rawY=Wire.read()<<8|Wire.read();
  Acc_rawZ=Wire.read()<<8|Wire.read(); 

  /////////////////////CONVERTING RAW DATA TO ANGLES/////////////////////
  roll = atan((Acc_rawX/16384.0)/(Acc_rawZ/16384.0))*rad_to_deg;
*/

//Begin I2C comm
/*
  Wire.begin(); 
  Wire.setClock(400000);       //maximum frequency for arduino is 400k Hz, fastest processing clock.
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
*/

//Testing time between each microstep//
/*
     if(j%2 == 0){
   
        Serial.println("step end ");
        Serial.println(micros());
        Serial.println(state);
       
        j=0;
      }
      else{
        Serial.println("step begin ");
        Serial.println(micros());
        Serial.println(state);
      }
    //////////////////////////////////////*/

   /*
    * #include <Wire.h>
#include "mcu6050.h"

//===============INSTANCE_VARIABLES==============//

//Motor pin declarations//
int rDir = 3;    //Right direction.
int lDir = 6;    //Left direction.

int rStep = 2;   //Right step.
int lStep = 5;   //Left step.

//Data collection variables//
float rad_to_deg = 180/M_PI;
float angle_deg;
float dot_angle_deg;

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
boolean state = HIGH;   //manipulates the HIGH and LOW pulse of PWM signal.
long nextStep_us;      //Next step of motor in microseconds.
float set_delay_us;    

//variables for interrupt//
const int max_16bit = 65535;        //Max integer 16bit can store.
const int f_clock = 16*10^6;        //Arduino UNO clock speed is 16 MHz.

int t_period_s = 800*10^(-6);       //target period to interrupt**.
int prescale = 8;                   //prescale factor to scale number of interrupts stored**. 

const uint16_t t1_load = 0;         //resets timer back to 0.
uint16_t t1_comp = max_16bit - f_clock*t_period_s/prescale; 
         /* 
          Sets the compare interupt time.
          t1_comp = 16bits - target_period(s)*16Mhz/prescale-factor.  
         */
  /*                                                
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

  //interruption set up//
  cli();          //disable global interrupts
  TCCR1A = 0;     //reset timer1 control reg A

  //set prescaling value to 8
  TCCR1B &= ~(1 << CS12);
  TCCR1B |= (1 << CS11);
  TCCR1B &= ~(1 << CS10);

  //reset timer and set compare value
  TCNT1 = t1_load;
  OCR1A = t1_comp;

  //Enable Timer1 compare interrupt
  TIMSK1 = (1 << OCIE1A);

  //enable global interrupts
  sei();
}

//======================LOOP======================//
                                                     
void loop() {
  //long t0=micros();  //begin timer.

  ///Reads ACC and GYRO values from MPU6050//
  readMCU6050data(); 
  
  //Calculates angles used for roll//
  angle_deg = getMCU6050_fused_angleY()*rad_to_deg;       //roll angle.
  dot_angle_deg = getMCU6050_gyro_rateY()*rad_to_deg;     //the derivative of roll angle.

  //Difference from desired angle//
  error = angle_deg - desired_angle;
  //Serial.println(angle_deg);
  
  //Proportion, Integral, Derivative Control loop//
  pid_p = kp*error;
  pid_i += ki*error;
  pid_d = kd*dot_angle_deg;
  PID = pid_p + pid_i + pid_d;
  
  set_delay_us = 3200/abs(PID);
  //Serial.println(set_delay_us);
  
  
  /*
  long t1=micros();  //end timer.
  runtime(t0,t1);
  */
