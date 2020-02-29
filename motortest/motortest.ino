// Define stepper motor connections and steps per revolution:
#include <Wire.h>
#define rDir 3
#define rStep 2
#define lDir 6
#define lStep 5
int16_t Acc_rawX, Acc_rawY, Acc_rawZ;
float roll = 0;
float rad_to_deg = 180/3.141592654;
#define stepsPerRevolution 1675 //1.8 degrees per step, 200 steps 1 revolution

long nextStep_us;

void setup() {
  // Declare pins as output:
  Serial.begin(115200);
  pinMode(rStep, OUTPUT);
  pinMode(rDir, OUTPUT);
  pinMode(lStep, OUTPUT);
  pinMode(lDir, OUTPUT);
  Wire.begin(); /////////////TO BEGIN I2C COMMUNICATIONS///////////////
  Wire.setClock(400000);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  

  nextStep_us=micros();  
}

long t=0;
long n=0;

long period_us=1675*2;
boolean state=LOW;

void loop() {
  long t0=micros();
  
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

  long t1=micros();

  t += (t1-t0);
  n ++;
  if (n==1000) {
    Serial.println(t/n);
    n=0;
    t=0;
  }


  if (micros()>=nextStep_us) {
    // Set the spinning direction clockwise:
    digitalWrite(rDir, HIGH);
    digitalWrite(lDir, HIGH);

    digitalWrite(rStep, state);
    digitalWrite(lStep, state);
    state = ! state;
    nextStep_us += period_us;
      
  }
  /*
  // Set the spinning direction clockwise:
  digitalWrite(rDir, HIGH);
  digitalWrite(lDir, HIGH);

  // These four lines result in 1 step:
  digitalWrite(rStep, HIGH);
  digitalWrite(lStep, HIGH);
  delayMicroseconds(600);
  digitalWrite(rStep, LOW);
  digitalWrite(lStep, LOW);
  delayMicroseconds(600);
*/

 /*    Serial.print("roll(Y): ");
    Serial.print(Acceleration_angle[0]);
    Serial.print("    ");
    Serial.print("AccelX: ");
    Serial.print(Acc_rawX/16384.0);
    Serial.print("    ");
    Serial.print("AccelY: ");
    Serial.print(Acc_rawY/16384.0);
    Serial.print("    ");
    Serial.print("AccelZ: ");
    Serial.print(Acc_rawZ/16384.0);
    Serial.println("");*/

    
   /*// Set the spinning direction counterclockwise:
  digitalWrite(rDir, LOW);
  digitalWrite(lDir, LOW);
  // Spin the stepper motor 1 revolution quickly:
  for (int i = 0; i < stepsPerRevolution; i++) {
    // These four lines result in 1 step:
    digitalWrite(rStep, HIGH);
    digitalWrite(lStep, HIGH);
    delayMicroseconds(150);
    digitalWrite(rStep, LOW);
    digitalWrite(lStep, LOW);
    delayMicroseconds(150);
    
  }
  */

  
  /*
  // Set the spinning direction clockwise:
  digitalWrite(rDir, HIGH);
  digitalWrite(lDir, HIGH);
  // Spin the stepper motor 5 revolutions fast:
  for (int i = 0; i < 5 * stepsPerRevolution; i++) {
    // These four lines result in 1 step:
    digitalWrite(rStep, HIGH);
    digitalWrite(lStep, HIGH);
    delayMicroseconds(500);
    digitalWrite(rStep, LOW);
    digitalWrite(lStep, LOW);
    delayMicroseconds(500);
  }
  delay(1000);
  // Set the spinning direction counterclockwise:
  digitalWrite(rDir, LOW);
  digitalWrite(lDir, LOW);
  //Spin the stepper motor 5 revolutions fast:
  for (int i = 0; i < 5 * stepsPerRevolution; i++) {
    // These four lines result in 1 step:
    digitalWrite(rStep, HIGH);
    digitalWrite(lStep, HIGH);
    delayMicroseconds(500);
    digitalWrite(rStep, LOW);
    digitalWrite(lStep, LOW);
    delayMicroseconds(500);
  }
  delay(1000);
  */
}
