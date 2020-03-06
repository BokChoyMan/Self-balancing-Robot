#include <Wire.h>

// reads a single 8bit register
extern byte readRegister(int device,int reg);

// reads a 16bit register
extern int read2Registers(int device,int reg);

// reads N 8bit registers
extern void readNregisters(int device,int reg,int n,byte *buffer);

// reads N 16bit registers
extern void read2Nregisters(int device,int reg,int n,int *buffer);

extern void writeRegister(int device,int reg,int value);
