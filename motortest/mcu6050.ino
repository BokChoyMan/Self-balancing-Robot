//#include "mcu6050.h"

/****** WIRING
 *  SDA : Uno A4 = bidirectional level shifter = Breakout Board SDA
 *  SCL : Uno A5 = bidirectional level shifter = Breakout Board SCL
 ******/


// status & data
#define MCU6050_ADDR 0b1101000   // pin Ad0 logic low
#define MCU6050_REG_CHIPID     0x75

#define MCU6050_REG_DATA 0x3B

MCU6050data_S data;

void resetMCU6050data(struct MCU6050data_S *data)
{
  int *mbuffer=&(data->accX_max);
  for (int i=0;i<7;i++) 
     *mbuffer++=0;
}

void startMCU6050()
{
  Wire.begin();                // join i2c bus

  // set I2C speed (from http://forum.arduino.cc/index.php?topic=39772.0)
  TWBR = ((F_CPU / TWI_FREQ_MCU6050) - 16) / 2;

  // power management, clock source, & reset device
#define MCU6050_REG_PWR_MGMT_1 0x6B
#define MCU6050_PWR_MGMT_RESET_GYROXCLK      0b10000001
#define MCU6050_PWR_MGMT_NORESET_GYROXCLK    0b00000001
#define MCU6050_PWR_MGMT_RESET_INTERNALCLK   0b10000000
  writeRegister(MCU6050_ADDR,MCU6050_REG_PWR_MGMT_1,MCU6050_PWR_MGMT_NORESET_GYROXCLK);
  delay(500); // wait for reset

  Serial.print("Default ");
  printMCU6050conf();
  
  // set sample rate
#define MCU6050_REG_SMPRT_DIV      0x19
#define MCU6050_SMPRT_DIV_8KHz     0
  writeRegister(MCU6050_ADDR,MCU6050_REG_SMPRT_DIV,MCU6050_SMPRT_DIV_8KHz);

  // disable FSYNC & disable low pass filter
#define MCU6050_REG_CONFIG    0x1A
#define MCU6050_CONFIG_DISABLE_FSYNC 0b0
#define MCU6050_CONFIG_DLPF_260Hz    0b000
#define MCU6050_CONFIG_DLPF_184Hz    0b001
#define MCU6050_CONFIG_DLPF_94Hz     0b010
  writeRegister(MCU6050_ADDR,MCU6050_REG_CONFIG,MCU6050_CONFIG_DISABLE_FSYNC|MCU6050_CONFIG_DLPF_94Hz);

  // disable FIFO & reset registers
#define MCU6050_REG_FIFO_EN 0x23
  writeRegister(MCU6050_ADDR,MCU6050_REG_FIFO_EN,0b0);
#define MCU6050_REG_USER_CTRL 0x6A
  writeRegister(MCU6050_ADDR,MCU6050_REG_USER_CTRL,0b0);

  // gyro config
#define MCU6050_REG_GYRO_CONFIG    0x1B
#define MCU6050_GYRO_CONFIG_RANGE_FINEST       0b00000
#define MCU6050_GYRO_RANGE_FINEST_LSB_PERRADS   (32767.0/250.0*180.0/M_PI)
#define MCU6050_GYRO_CONFIG_RANGE_MID          0b01000
#define MCU6050_GYRO_RANGE_MID_LSB_PERRADS      (32767.0/500.0*180.0/M_PI)
#define MCU6050_GYRO_CONFIG_RANGE_COARSE       0b10000
#define MCU6050_GYRO_RANGE_COARSE_LSB_PERRADS   (32767.0/1000.0*180.0/M_PI)
#define MCU6050_GYRO_CONFIG_RANGE_COARSEST     0b11000
#define MCU6050_GYRO_RANGE_COARSEST_LSB_PERRADS (32767.0/2000.0*180.0/M_PI)
  writeRegister(MCU6050_ADDR,MCU6050_REG_GYRO_CONFIG,MCU6050_GYRO_CONFIG_RANGE_COARSE);

  // accel config
#define MCU6050_REG_ACCEL_CONFIG    0x1C
#define MCU6050_ACCEL_CONFIG_RANGE_FINEST       0b00000
#define MCU6050_ACCEL_CONFIG_RANGE_MID          0b01000
#define MCU6050_ACCEL_CONFIG_RANGE_COARSE       0b10000
#define MCU6050_ACCEL_CONFIG_RANGE_COARSEST     0b11000
  writeRegister(MCU6050_ADDR,MCU6050_REG_ACCEL_CONFIG,MCU6050_ACCEL_CONFIG_RANGE_MID);

  // set interrupt configuration
#define MCU6050_REG_INT_PIN_CFG    0x37
#define MCU6050_INT_PUSHPULL_HIGH_AUTOCLEAR  0b10110000
  writeRegister(MCU6050_ADDR,MCU6050_REG_INT_PIN_CFG,MCU6050_INT_PUSHPULL_HIGH_AUTOCLEAR);

  // attach INT1 to Data Ready
#define MCU6050_REG_INT_ENABLE    0x38
#define MCU6050_INT_DATA_RDY      0b1
  writeRegister(MCU6050_ADDR,MCU6050_REG_INT_ENABLE,MCU6050_INT_DATA_RDY);

  Serial.println("Computing bias");
  
  // find bias by averaging N measurements
  data.acc_angleY_bias=0;
  data.gyroY_bias=0;

  int N=1000/SAMPLING_TIME;  // 1 sec of measurements
  for (int i=0;i<N;i++) {
    readMCU6050data();
    data.acc_angleY_bias -=atan2(data.accX,data.accZ);
    data.gyroY_bias += data.gyroY;
    delay(SAMPLING_TIME);
  }
  data.acc_angleY_bias /= N;
  data.gyroY_bias /=N;

  // initialize integrator
  data.gyro_angleY=0;

  // initialize filter
  data.fused_angleY=0;

  // attach interrupt for data ready
  //pinMode(13,INPUT); // just to see interrupt if device connect to pin 13

  Serial.println("Computed bias");
  //attachInterrupt(digitalPinToInterrupt(DATAREADY_INT_PIN), readMCU6050data, RISING);
  //attachInterrupt(digitalPinToInterrupt(DATAREADY_INT_PIN), readMCU6050data, HIGH);
 
  Serial.print("Post setup ");
  printMCU6050conf();

  // wait for IC to stabilize
  delay(1000);

}

void readMCU6050data()//struct MCU6050data_S *data)
{
  // local time stamp
  long local_time_us=micros();

  // read device
  Wire.beginTransmission(MCU6050_ADDR); 
  Wire.write(MCU6050_REG_DATA);
  Wire.endTransmission();

  int *buffer=&(data.accX);
  Wire.requestFrom(MCU6050_ADDR, 14);
  for (int i=0;i<7;i++)
    *(buffer++) = (((long)Wire.read())<<8)+(long)Wire.read();

  // update time stamps
  data.local_deltatime_us=local_time_us-data.local_time_us;
  data.local_time_us=local_time_us;

  // update maximums
  buffer=&(data.accX);
  int *mbuffer=&(data.accX_max);
  for (int i=0;i<7;i++,buffer++,mbuffer++) {
    if (*buffer > *mbuffer)
      *mbuffer=*buffer;
    if (-*buffer > *mbuffer)
      *mbuffer=-*buffer;
  }

  // update accel angle
  data.acc_angleY=-atan2(data.accX,data.accZ)-data.acc_angleY_bias;

  float dt=data.local_deltatime_us/1e6;

  // update angle derivative from gyro
  //data.gyro_rateY=((float)data.gyroY-data.gyroY_bias)/MCU6050_GYRO_RANGE_FINEST_LSB_PERRADS;
  //data.gyro_rateY=((float)data.gyroY-data.gyroY_bias)/MCU6050_GYRO_RANGE_MID_LSB_PERRADS;
  data.gyro_rateY=((float)data.gyroY-data.gyroY_bias)/MCU6050_GYRO_RANGE_COARSE_LSB_PERRADS;

  // update angle integration
  data.gyro_angleY += data.gyro_rateY*dt;

  /* complementary filter
   * hat angle = tau/(tau s+1) \dot angle  + 1/(tau s+1) angle
   * <=> (tau s+1) \hat angle = \tau \dot angle + angle
   * <=> \tau \dot \hat angle = -\hat angle + \tau \dot angle + angle 
   * <=> \dot \hat angle = (angle-\hat angle)/tau + \dot angle
   * <=> \hat angle^+ = \hat angle+ [ (angle-\hat angle)/tau + \dot angle ] * dT
   */

  data.fused_angleY += ( (data.acc_angleY-data.fused_angleY)/TAU+data.gyro_rateY )*dt;
  
  // keep time of compute time
  data.local_readtime_us=micros()-local_time_us;
}

void printMCU6050conf()
{
  // display status & configuration
  Serial.print("device ID: ");
  Serial.print(readRegister(MCU6050_ADDR,MCU6050_REG_CHIPID),BIN);
  Serial.print(", power management: ");
  Serial.print(readRegister(MCU6050_ADDR,MCU6050_REG_PWR_MGMT_1),BIN);
  Serial.print(", sample rate: ");
  Serial.println(readRegister(MCU6050_ADDR,MCU6050_REG_SMPRT_DIV),BIN);

#define MCU6050_REG_FIFO_COUNT 0x72
  Serial.print("FIFO enable: ");
  Serial.print(readRegister(MCU6050_ADDR,MCU6050_REG_FIFO_EN),BIN);
  Serial.print(", FIFO length: ");
  Serial.println(read2Registers(MCU6050_ADDR,MCU6050_REG_FIFO_COUNT));

#define MCU6050_REG_INT_STATUS 0x3A
  Serial.print("INT status: ");
  Serial.println(readRegister(MCU6050_ADDR,MCU6050_REG_INT_STATUS),BIN);

  Serial.print("Config: ");
  Serial.println(readRegister(MCU6050_ADDR,MCU6050_REG_CONFIG),BIN);
  Serial.print("Gyro config: ");
  Serial.println(readRegister(MCU6050_ADDR,MCU6050_REG_GYRO_CONFIG),BIN);
  Serial.print("Accel config: ");
  Serial.println(readRegister(MCU6050_ADDR,MCU6050_REG_ACCEL_CONFIG),BIN);
}

void printMCU6050data(struct MCU6050data_S *data)
{
  Serial.print("local_time_us = ");
  Serial.println(data->local_time_us);  
  
  Serial.print("temp = ");
  Serial.print(data->temp);
  Serial.println("");
  Serial.print("gyr = (");
  Serial.print(data->gyroX);
  Serial.print(",");
  Serial.print(data->gyroY);
  Serial.print(",");
  Serial.print(data->gyroZ);
  Serial.println(")");
  Serial.print("acc = (");
  Serial.print(data->accX);
  Serial.print(",");
  Serial.print(data->accY);
  Serial.print(",");
  Serial.print(data->accZ);
  Serial.println(")");
  
  Serial.print("max temp = ");
  Serial.print(data->temp_max);
  Serial.println("");
  Serial.print("max gyr = (");
  Serial.print(data->gyroX_max);
  Serial.print(",");
  Serial.print(data->gyroY_max);
  Serial.print(",");
  Serial.print(data->gyroZ_max);
  Serial.println(")");
  Serial.print("max acc = (");
  Serial.print(data->accX_max);
  Serial.print(",");
  Serial.print(data->accY_max);
  Serial.print(",");
  Serial.print(data->accZ_max);
  Serial.println(")");

  Serial.print("INT status: ");
  Serial.print(readRegister(MCU6050_ADDR,MCU6050_REG_INT_STATUS),BIN);
  Serial.println("");
}
