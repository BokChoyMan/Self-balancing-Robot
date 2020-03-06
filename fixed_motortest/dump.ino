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
