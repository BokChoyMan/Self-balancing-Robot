#include "i2c-aux.h"

// Desired I2C freq in Hz. MCU6050 can handle up to 400MHz
// Arduino does not seem to handle past 800KHz.
#define TWI_FREQ_MCU6050 400000L 

// not really using interrupt since I2C cannot be called from inside interrupt handler
#define DATAREADY_INT_PIN 4

#define SAMPLING_TIME 3

// time constant of the complementary filter [sec]
#define TAU 1  

struct MCU6050data_S {
  // MCU6050 data
  int accX;
  int accY;
  int accZ;
  int temp;
  int gyroX;
  int gyroY;
  int gyroZ;

  // local data
  long local_time_us; // time stamp from interrupt

  // computed data
  long local_deltatime_us;
  long local_readtime_us;

  // bias
  float acc_angleY_bias;
  float gyroY_bias;

  // angle derivative from gyro
  float gyro_rateY;
  
  // integrated gyro angle
  float gyro_angleY;
  
  // accel angle
  float acc_angleY;

  // fused angle
  float fused_angleY;
  
  // max values
  int accX_max;
  int accY_max;
  int accZ_max;
  int temp_max;
  int gyroX_max;
  int gyroY_max;
  int gyroZ_max;
};

#define getMCU6050_gyro_rateY() (data.gyro_rateY)
#define getMCU6050_gyro_angleY() (data.gyro_angleY)
#define getMCU6050_acc_angleY() (data.acc_angleY)
#define getMCU6050_fused_angleY() (data.fused_angleY)

extern MCU6050data_S data;

extern void resetMCU6050data(struct MCU6050data_S *data);
extern void readMCU6050data();//struct MCU6050data_S *data);
extern void printMCU6050conf();
extern void printMCU6050data(struct MCU6050data_S *data);
extern void startMCU6050();
