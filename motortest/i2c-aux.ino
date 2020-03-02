#include "i2c-aux.h"

// reads a single 8bit register
byte readRegister(int device,int reg)
{
  Wire.beginTransmission(device); 
  Wire.write(reg);
  Wire.endTransmission();
  
  Wire.requestFrom(device, 1);
  return Wire.read();
}

// reads a 16bit register
int read2Registers(int device,int reg)
{
  Wire.beginTransmission(device); 
  Wire.write(reg);
  Wire.endTransmission();
  
  Wire.requestFrom(device, 2);
  return Wire.read()+(Wire.read()<<8);
}

// reads N 8bit registers
void readNregisters(int device,int reg,int n,byte *buffer)
{
  Wire.beginTransmission(device); 
  Wire.write(reg);
  Wire.endTransmission();

  Wire.requestFrom(device, n);
  for (int i=0;i<n;i++)
    *(buffer++) = Wire.read();
}

// reads N 16bit registers
void read2Nregisters(int device,int reg,int n,int *buffer)
{
  Wire.beginTransmission(device); 
  Wire.write(reg);
  Wire.endTransmission();

  Wire.requestFrom(device, 2*n);
  for (int i=0;i<n;i++)
    *(buffer++) = ((long)Wire.read()) + (((long)Wire.read())<<8);
}

void writeRegister(int device,int reg,int value)
{
  Wire.beginTransmission(device); 
  Wire.write(reg);  
  Wire.write(value); 
  Wire.endTransmission(); 
}

