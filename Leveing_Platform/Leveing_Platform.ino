/*
Leveling_Platform.ino

Controls single axis leveling platform for a GoPro camera, maintaining it
horizontal.

L293D Shield (see "293D Shield.tif")

Drives L239 H-Bridge for 1 motor, looks for push-button to change directions.

PWM Pins: 3, 5, 6, 9, 10, and 11

 */
 
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t ax, ay, az;
int16_t gx, gy, gz;

// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO


#define LED_PIN 13
bool blinkState = false;

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

// join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(38400);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
}


void loop()
{
  int error;
  double dT;
  float gyro_mag;
  int Actual;
  int P;
  int I;
  int D;
  int accel_err;
  int control;
  int control_mag;

    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);

    #ifdef OUTPUT_READABLE_ACCELGYRO
        // display tab-separated accel/gyro x/y/z values
        Serial.print("a/g:\t");
        Serial.print(ax); Serial.print("\t");
        Serial.print(ay); Serial.print("\t");
        Serial.print(az); Serial.print("\t");
        Serial.print(gx); Serial.print("\t");
        Serial.print(gy); Serial.print("\t");
        Serial.println(gz);
    #endif

    #ifdef OUTPUT_BINARY_ACCELGYRO
        Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
        Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
        Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
        Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
        Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
        Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
    #endif
    
  // Read Y accel value and control based on results
  // if within 1000, stop the motor
//  Serial.print(accel_t_gyro.value.y_accel);
//  Serial.println();
  
  //-------------------- Calculates the PID drive value  --------------------
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
