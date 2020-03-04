#include <Wire.h>
#include<MPU6050.h>
MPU6050 mpu;
//owo uwu
////////////////VARIABLE DEFINATION///////////////
int rDir = 3;
int rStep = 2;
int lDir = 6;
int lStep = 5;
int mspeed = 500;

int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;
float Acceleration_angle = 0;
float Gyro_angle[2];
float Current_Angle[2];
float elapsedTime, time, timePrev;
float rad_to_deg = 180/3.141592654;
float PID, pwmLeft, pwmRight, error, previous_error;
float pid_p=0;
float pid_i=0;
float pid_d=0;

////////////////////////PID CONSTANST/////////////////////
//float kp=1;
//float ki=0;
//float kd=0;
float desired_angle = 0;//////////////TARGET ANGLE/////////////

void setup() 
{
  Wire.begin(); /////////////TO BEGIN I2C COMMUNICATIONS///////////////
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  mpu.setXAccelOffset(-2650);
  mpu.setYAccelOffset(-1290);
  mpu.setZAccelOffset(1783);
  
  mpu.setXGyroOffset(-75);
  mpu.setYGyroOffset(-327);
  mpu.setZGyroOffset(-3);
  
  ////////////////PIN MODE DEFINITIONS//////////////////////
  pinMode(rDir,OUTPUT);
  pinMode(lDir,OUTPUT);
  pinMode(rStep,OUTPUT);
  pinMode(lStep,OUTPUT);
  Serial.begin(9600);
  time = millis(); ///////////////STARTS COUNTING TIME IN MILLISECONDS/////////////
}

void loop() 
{
  /*////////////////////////WARNING//////////////////////
   * DO NOT USE ANY DELAYS INSIDE THE LOOP OTHERWISE THE BOT WON'T BE 
   * ABLE TO CORRECT THE BALANCE FAST ENOUGH
   * ALSO, DONT USE ANY SERIAL PRINTS. BASICALLY DONT SLOW DOWN THE LOOP SPEED.
  */
  
    timePrev = time;  
    time = millis();  
    elapsedTime = (time - timePrev) / 1000; 
    Wire.beginTransmission(0x68);
    Wire.write(0x3B); 
    Wire.endTransmission(false);
    Wire.requestFrom(0x68,6,true);
    
    ////////////////////PULLING RAW ACCELEROMETER DATA FROM IMU///////////////// 
    Acc_rawX=Wire.read()<<8|Wire.read(); 
    Acc_rawY=Wire.read()<<8|Wire.read();
    Acc_rawZ=Wire.read()<<8|Wire.read(); 
    
    /////////////////////CONVERTING RAW DATA TO ANGLES/////////////////////
    Acceleration_angle = atan((Acc_rawX/16384.0)/(Acc_rawZ/16384.0))*rad_to_deg;
    
   /* 
    Wire.beginTransmission(0x68);
    Wire.write(0x43); 
    Wire.endTransmission(false);
    Wire.requestFrom(0x68,6,true); 
    
    //////////////////PULLING RAW GYRO DATA FROM IMU/////////////////////////
    Gyr_rawX=Wire.read()<<8|Wire.read(); 
    Gyr_rawY=Wire.read()<<8|Wire.read(); 
    Gyr_rawZ=Wire.read()<<8|Wire.read(); 
    
    ////////////////////CONVERTING RAW DATA TO ANGLES///////////////////////
    Gyro_angle[0] = Gyr_rawY/131.0; 
    //Gyro_angle[1] = Gyr_rawZ/131.0;
    
    //////////////////////////////COMBINING BOTH ANGLES USING COMPLIMENTARY FILTER////////////////////////
    Current_Angle[0] = 0.98 *(Current_Angle[0] + Gyro_angle[0]*elapsedTime) + 0.02*Acceleration_angle[0];
    //Current_Angle[1] = 0.98 *(Current_Angle[1] + Gyro_angle[1]*elapsedTime) + 0.02*Acceleration_angle[1];
    
    ////Current_Angle[0] IS THE PITCH ANGLE WHICH WE NEED////////////
    error = Acceleration_angle[0] - desired_angle; /////////////////ERROR CALCULATION////////////////////

    
    ///////////////////////PROPORTIONAL ERROR//////////////
    pid_p = kp*error;
    ///////////////////////INTERGRAL ERROR/////////////////
    pid_i = pid_i+(ki*error);  
    ///////////////////////DIFFERENTIAL ERROR//////////////
    pid_d = kd*((error - previous_error)/elapsedTime);

    
    ///////////////////////TOTAL PID VALUE/////////////////
    PID = pid_p + pid_i + pid_d;

    
    ///////////////////////UPDATING THE ERROR VALUE////////
    previous_error = error;
    //Serial.println(PID);                     //////////UNCOMMENT FOR DDEBUGGING//////////////
    //delay(60);                               //////////UNCOMMENT FOR DDEBUGGING//////////////
    //Serial.println(Current_Angle[0]);          //////////UNCOMMENT FOR DDEBUGGING//////////////
    
    /////////////////CONVERTING PID VALUES TO ABSOLUTE VALUES//////////////////////////////////
    mspeed = abs(PID);
    //Serial.println(mspeed);                  //////////UNCOMMENT FOR DDEBUGGING//////////////
    */
    ///////////////SELF EXPLANATORY///////////////
    
    if(Acceleration_angle < 1)
      {
       counter();
      }
    if(Acceleration_angle > 1)
      {
       clockw();
      }
    if(Acceleration_angle >45 || Acceleration_angle <-45 )
      {
        halt();
      }
     else{
      halt();
     }
  
}

//////////////MOVEMENT FUNCTION///////////////////
void move()
{
  digitalWrite(rStep, HIGH);
  digitalWrite(lStep, HIGH);
  delayMicroseconds(1/abs(Acceleration_angle));
  digitalWrite(rStep, LOW);
  digitalWrite(lStep, LOW);
  delayMicroseconds(1/abs(Acceleration_angle));
  
}

void clockw()
{
  digitalWrite(rDir, HIGH);
  digitalWrite(lDir, HIGH);
  move();
}

void counter()
{
  digitalWrite(rDir, LOW);
  digitalWrite(lDir, LOW);
  move();
}

void halt()
{
  digitalWrite(rStep,LOW);
  digitalWrite(lStep,LOW);
}

       
