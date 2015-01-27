// MPU-6050 Accelerometer + Gyro
// -----------------------------
//
// By arduino.cc user "Krodal".
//
// June 2012
//      first version
// July 2013 
//      The 'int' in the union for the x,y,z
//      changed into int16_t to be compatible
//      with Arduino Due.
//
// Open Source / Public Domain
//
// Using Arduino 1.0.1
// It will not work with an older version, 
// since Wire.endTransmission() uses a parameter 
// to hold or release the I2C bus.
//
// Documentation:
// - The InvenSense documents:
//   - "MPU-6000 and MPU-6050 Product Specification",
//     PS-MPU-6000A.pdf
//   - "MPU-6000 and MPU-6050 Register Map and Descriptions",
//     RM-MPU-6000A.pdf or RS-MPU-6000A.pdf
//   - "MPU-6000/MPU-6050 9-Axis Evaluation Board User Guide"
//     AN-MPU-6000EVB.pdf
// 
// The accuracy is 16-bits.
//
// Temperature sensor from -40 to +85 degrees Celsius
//   340 per degrees, -512 at 35 degrees.
//
// At power-up, all registers are zero, except these two:
//      Register 0x6B (PWR_MGMT_2) = 0x40  (I read zero).
//      Register 0x75 (WHO_AM_I)   = 0x68.
// 
/*
L293D Shield (see "293D Shield.tif")

Drives L239 H-Bridge for 1 motor, looks for push-button to change directions.

PWM Pins: 3, 5, 6, 9, 10, and 11

 */

#include <Wire.h>

int PauseTime = 2000;
int M1_CWPin = 2;
int M1_CCWPin = 3;
int M1_EnablePin = 6;
int M2_CWPin = 4;
int M2_CCWPin = 5;
int M2_EnablePin = 9;
int MotorSpeed = 255;  // 75 = 1.4V up tilt, 230 = 3.49 V
int IntThresh = 1000;  // min error for integer gain to turn on
int SetPt = 0;  // servo value
int Kp = 2;
int Ki = 0;
int Kd = 6;
int Last = SetPt;
int Integral = 0;

void setup()
{      
  int error;
  uint8_t c;
  
  pinMode(M1_EnablePin, OUTPUT);
  analogWrite(M1_EnablePin,0);
  analogWrite(M1_CWPin,0);
  analogWrite(M1_CCWPin,0);

  Serial.begin(9600);

  // Initialize the 'Wire' class for the I2C-bus.
  Wire.begin();

  // default at power-up:
  //    Gyro at 250 degrees second
  //    Acceleration at 2g
  //    Clock source at internal 8MHz
  //    The device is in sleep mode.
  //

  error = MPU6050_read (MPU6050_WHO_AM_I, &c, 1);
  Serial.print(F("WHO_AM_I : "));
  Serial.print(c,HEX);
  Serial.print(F(", error = "));
  Serial.println(error,DEC);

  // According to the datasheet, the 'sleep' bit
  // should read a '1'.
  // That bit has to be cleared, since the sensor
  // is in sleep mode at power-up. 
  error = MPU6050_read (MPU6050_PWR_MGMT_1, &c, 1);
  Serial.print(F("PWR_MGMT_1 : "));
  Serial.print(c,HEX);
  Serial.print(F(", error = "));
  Serial.println(error,DEC);


  // Clear the 'sleep' bit to start the sensor.
  MPU6050_write_reg (MPU6050_PWR_MGMT_1, 0);
}


void loop()
{
  int error;
  double dT;
  accel_t_gyro_union accel_t_gyro;
  float gyro_mag;
  int Actual;
  int P;
  int I;
  int D;
  int accel_err;
  int control;
  int control_mag;

  // Read the raw values.
  // Read 14 bytes at once, 
  // containing acceleration, temperature and gyro.
  // With the default settings of the MPU-6050,
  // there is no filter enabled, and the values
  // are not very stable.
  error = MPU6050_read (MPU6050_ACCEL_XOUT_H, (uint8_t *) &accel_t_gyro, sizeof(accel_t_gyro));
  
  // Swap all high and low bytes.
  // After this, the registers values are swapped, 
  // so the structure name like x_accel_l does no 
  // longer contain the lower byte.
  uint8_t swap;
  #define SWAP(x,y) swap = x; x = y; y = swap

  SWAP (accel_t_gyro.reg.x_accel_h, accel_t_gyro.reg.x_accel_l);
  SWAP (accel_t_gyro.reg.y_accel_h, accel_t_gyro.reg.y_accel_l);
  SWAP (accel_t_gyro.reg.z_accel_h, accel_t_gyro.reg.z_accel_l);
  SWAP (accel_t_gyro.reg.t_h, accel_t_gyro.reg.t_l);
  SWAP (accel_t_gyro.reg.x_gyro_h, accel_t_gyro.reg.x_gyro_l);
  SWAP (accel_t_gyro.reg.y_gyro_h, accel_t_gyro.reg.y_gyro_l);
  SWAP (accel_t_gyro.reg.z_gyro_h, accel_t_gyro.reg.z_gyro_l);

  // Read Y accel value and control based on results
  // if within 1000, stop the motor
//  Serial.print(accel_t_gyro.value.y_accel);
//  Serial.println();
  
  //-------------------- Calculates the PID drive value  --------------------
  Actual = accel_t_gyro.value.y_accel;
  accel_err = SetPt - Actual;
  
  if (abs(accel_err) < IntThresh)
  {
    Integral = Integral + accel_err;
  } 
  else 
  {
    Integral=0;
  }
  
  control = accel_err*Kp + Integral*Ki + (Last-Actual)*Kd;
  control_mag = abs(control);
  MotorSpeed = map(control_mag,0,15500,0,255);  //scale control to PWM range  
  
  Serial.print(control_mag, DEC);
  Serial.print(F(", "));
  Serial.print(MotorSpeed, DEC);
  Serial.println();

  if (control > 0)
  {
    CCW();
  }
  if (control < 0)
  {
    CW();
  }
  Last = Actual;
  
//  if (accel_t_gyro.value.y_accel < 1000 && accel_t_gyro.value.y_accel > -1000)
//  {
//    analogWrite(M1_EnablePin,LOW);
//    }
//  if (accel_t_gyro.value.y_accel > 1000)
//  {
//    CCW();
//  }
//  if (accel_t_gyro.value.y_accel < -1000)
//  {
//    CW();
//  }
}


// --------------------------------------------------------
// MPU6050_read
//
// This is a common function to read multiple bytes 
// from an I2C device.
//
// It uses the boolean parameter for Wire.endTransMission()
// to be able to hold or release the I2C-bus. 
// This is implemented in Arduino 1.0.1.
//
// Only this function is used to read. 
// There is no function for a single byte.
//
int MPU6050_read(int start, uint8_t *buffer, int size)
{
  int i, n, error;

  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  n = Wire.write(start);
  if (n != 1)
    return (-10);

  n = Wire.endTransmission(false);    // hold the I2C-bus
  if (n != 0)
    return (n);

  // Third parameter is true: relase I2C-bus after data is read.
  Wire.requestFrom(MPU6050_I2C_ADDRESS, size, true);
  i = 0;
  while(Wire.available() && i<size)
  {
    buffer[i++]=Wire.read();
  }
  if ( i != size)
    return (-11);

  return (0);  // return : no error
}


// --------------------------------------------------------
// MPU6050_write
//
// This is a common function to write multiple bytes to an I2C device.
//
// If only a single register is written,
// use the function MPU_6050_write_reg().
//
// Parameters:
//   start : Start address, use a define for the register
//   pData : A pointer to the data to write.
//   size  : The number of bytes to write.
//
// If only a single register is written, a pointer
// to the data has to be used, and the size is
// a single byte:
//   int data = 0;        // the data to write
//   MPU6050_write (MPU6050_PWR_MGMT_1, &c, 1);
//
int MPU6050_write(int start, const uint8_t *pData, int size)
{
  int n, error;

  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  n = Wire.write(start);        // write the start address
  if (n != 1)
    return (-20);

  n = Wire.write(pData, size);  // write data bytes
  if (n != size)
    return (-21);

  error = Wire.endTransmission(true); // release the I2C-bus
  if (error != 0)
    return (error);

  return (0);         // return : no error
}

// --------------------------------------------------------
// MPU6050_write_reg
//
// An extra function to write a single register.
// It is just a wrapper around the MPU_6050_write()
// function, and it is only a convenient function
// to make it easier to write a single register.
//
int MPU6050_write_reg(int reg, uint8_t data)
{
  int error;

  error = MPU6050_write(reg, &data, 1);

  return (error);
}

void CW() {
  analogWrite(M1_EnablePin,MotorSpeed);
  digitalWrite(M1_CCWPin,LOW);
  digitalWrite(M1_CWPin,HIGH);
}

void CCW(){
  analogWrite(M1_EnablePin,MotorSpeed);
  digitalWrite(M1_CWPin,LOW);
  digitalWrite(M1_CCWPin,HIGH);
}

void readsensor() { 
  // Print the raw acceleration values
  int error;
  double dT;
  accel_t_gyro_union accel_t_gyro;
  
  Serial.print(F("accel x,y,z: "));
  Serial.print(accel_t_gyro.value.x_accel, DEC);
  Serial.print(F(", "));
  Serial.print(accel_t_gyro.value.y_accel, DEC);
  Serial.print(F(", "));
  Serial.print(accel_t_gyro.value.z_accel, DEC);
  Serial.println(F(""));


  // Print the raw gyro values.
  Serial.print(accel_t_gyro.value.y_gyro, DEC);
  Serial.println(F(""));

  delay(1000);
 }
