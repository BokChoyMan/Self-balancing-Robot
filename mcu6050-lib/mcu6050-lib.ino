#include "mcu6050.h"

void setup() 
{
  Serial.begin(115200);        // start serial communication 

  // initialize IMU device
  startMCU6050();
  
  Serial.print("Time = ");
  Serial.print((float)data.local_time_us/1e6);
  Serial.print(", bias in angle from acc = ");
  Serial.print(data.acc_angleY_bias/M_PI*180);
  Serial.print(", bias in angle from gyro = ");
  Serial.println(data.gyroY_bias/M_PI*180);
}

int prt=0;

void loop() {
  long t=micros();
  
  readMCU6050data();
  
  float angle_rad=getMCU6050_fused_angleY();
  float dot_angle_rad=getMCU6050_gyro_rateY();
  
  if (prt++>1000/SAMPLING_TIME) {
    prt=0;
    Serial.print("t=");
    Serial.print((float)data.local_time_us/1e6);
    Serial.print(", Loc dt=");
    Serial.print(data.local_deltatime_us);
    Serial.print(", Read dt=");
    Serial.print(data.local_readtime_us);
    Serial.print(", dot ang=");
    Serial.print(dot_angle_rad/M_PI*180);
    Serial.print(", ang fused=");
    Serial.print(angle_rad/M_PI*180);
    Serial.print(", ang acc=");
    Serial.print(getMCU6050_acc_angleY() /M_PI*180);
    Serial.print(", ang gyro=");
    Serial.print(getMCU6050_gyro_angleY()/M_PI*180);
    Serial.print(", err=");
    Serial.println((getMCU6050_gyro_angleY()-getMCU6050_acc_angleY())/M_PI*180);

    //printMCU6050data(&data);
  }

  delayMicroseconds(1000*SAMPLING_TIME-(micros()-t));
}
