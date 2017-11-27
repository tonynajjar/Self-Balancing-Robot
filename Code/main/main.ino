#include <Wire.h>
#include <Encoder.h>
#include "Kalman.h"
#include <PID_v1.h>
#include <LMotorController.h>
#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead
Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;
Encoder left(5, 6);
Encoder right(3, 11);
long oldPosition  = -999;

#define MIN_ABS_SPEED 70
/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data
unsigned long counter;

// TODO: Make calibration routine

//PID

const int AVERAGE_COUNT = 20;
float speed_values[AVERAGE_COUNT];
int filter_pos = 0;

double angleSetpoint = 0;
double angleInput, angleOutput;
double speedSetpoint = 0;
double speedInput, speedOutput;
int moveState = 0; //0 = balance; 1 = back; 2 = forth

double angleKp = 17.5;
double angleKd = 5;
double angleKi = 0.5;

double speedKp = 0.5;
double speedKd = 0;
double speedKi = 0;

PID anglePID(&angleInput, &angleOutput, &angleSetpoint, angleKp, angleKi, angleKd, DIRECT);
PID speedPID(&speedInput, &speedOutput, &speedSetpoint, speedKp, speedKi, speedKd, DIRECT);


double motorSpeedFactorLeft = 1;
double motorSpeedFactorRight = 1;
double realOutput = MIN_ABS_SPEED + 1;
//MOTOR CONTROLLER
const int ENA = 10;
const int IN1 = 7;
const int IN2 = 4;
const int IN3 = 8;
const int IN4 = 12;
const int ENB = 9;
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);


void setup() {
  Serial.begin(115200);
  Wire.begin();
  anglePID.SetTunings(angleKp, angleKd, angleKi);
  
  pinMode(13, OUTPUT);
  TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (i2cData[0] << 8) | i2cData[1];
  accY = (i2cData[2] << 8) | i2cData[3];
  accZ = (i2cData[4] << 8) | i2cData[5];

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();

  anglePID.SetMode(AUTOMATIC);
  anglePID.SetSampleTime(10);
  anglePID.SetOutputLimits(-255, 255);
  speedPID.SetMode(AUTOMATIC);
  speedPID.SetSampleTime(10);
  speedPID.SetOutputLimits(-5, 5);
}


long r_wheel, r_wheel_velocity, r_last_wheel;
long l_wheel, l_wheel_velocity, l_last_wheel;

void loop() {

  
  // calculate speed
  l_wheel = left.read();
  r_wheel = right.read();

  l_wheel_velocity = l_wheel - l_last_wheel;
  l_last_wheel = l_wheel;
  r_wheel_velocity = r_wheel - r_last_wheel;
  r_last_wheel = r_wheel;

  int speed = ((l_wheel_velocity + r_wheel_velocity));

  // let's make an average speed for the last N smaples
  int out = 0;
  speed_values[filter_pos] = speed;
  filter_pos = (filter_pos + 1) % AVERAGE_COUNT;
  for (int i = 0; i < AVERAGE_COUNT; i++) {
    out += speed_values[i];
  }
  speedInput = (out / AVERAGE_COUNT);
  
  /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  gyroX = (i2cData[8] << 8) | i2cData[9];
  gyroY = (i2cData[10] << 8) | i2cData[11];
  gyroZ = (i2cData[12] << 8) | i2cData[13];

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);

#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  /* Print Data 
#if 0 // Set to 1 to activate
  Serial.print(accX); Serial.print("\t");
  Serial.print(accY); Serial.print("\t");
  Serial.print(accZ); Serial.print("\t");

  Serial.print(gyroX); Serial.print("\t");
  Serial.print(gyroY); Serial.print("\t");
  Serial.print(gyroZ); Serial.print("\t");

  Serial.print("\t");
#endif

#if 0
  Serial.print(roll); Serial.print("\t");
  Serial.print(gyroXangle); Serial.print("\t");
  Serial.print(compAngleX); Serial.print("\t");
  Serial.print(kalAngleX); Serial.print("\t");

  Serial.print("\t");

  Serial.print(pitch); Serial.print("\t");
  Serial.print(gyroYangle); Serial.print("\t");
  Serial.print(compAngleY); Serial.print("\t");
  Serial.print(kalAngleY); Serial.print("\t");
#endif
*/

/*
  if (Serial.available()) {
    counter = millis();
    char c = Serial.read();

    Serial.println(c);

    switch (c) {

      case 'w':
        angleSetpoint = -3;
        break;

      case 's':
        angleSetpoint = -13;
        break;

      case 'a':
        digitalWrite(13, HIGH);
        break;

      case 'd':
        digitalWrite(13, HIGH);
        break;
    }
  }
  else {
    if (millis() - counter > 500) {
      digitalWrite(13, LOW);
      angleSetpoint = originalSetpoint;
    }

  }
  */
  
  speedPID.SetTunings(speedKp, speedKd, speedKi);
  speedPID.Compute();
  
  angleInput = kalAngleY;
  angleSetpoint= speedOutput;
  
  angleKp = map(analogRead(A0), 0, 1023, 0, 30);
  angleKd = double(analogRead(A2)) / 1023.0 * 20.0;
  angleKi = double(analogRead(A1)) / 1000.0;
  anglePID.SetTunings(angleKp, angleKd, angleKi);
  
  anglePID.Compute();
  
  Serial.print("angleKp= ");
  Serial.print(angleKp);
  Serial.print("; ");
  Serial.print("angleKi= ");
  Serial.print(angleKi);
  Serial.print("; ");
  Serial.print("angleKd= ");
  Serial.print(angleKd);
  Serial.print("; ");
  Serial.print("speedInput= ");
  Serial.print(speedInput);
  Serial.print(": ");
  Serial.print("speedOutput= ");
  Serial.print(speedOutput);
  Serial.print(": ");
  Serial.print("angleInput= ");
  Serial.print(angleInput);
  Serial.print(": ");
  Serial.print("angleOutput= ");
  Serial.print(angleOutput);
  Serial.print(": ");

  /*
    if(abs(angleOuput)>MIN_ABS_SPEED)
    Serial.println(angleOuput);
    else if(angleOuput>0)
    Serial.println(MIN_ABS_SPEED);
    else
    Serial.println(-MIN_ABS_SPEED);
  */
  //realOutput = motorController.getSpeed(angleOutput, MIN_ABS_SPEED);

  if (angleInput > angleSetpoint+0.5 || angleInput < angleSetpoint-0.5) {

    //if(realOutput> MIN_ABS_SPEED || realOutput< -MIN_ABS_SPEED){
    
    Serial.println(motorController.move(angleOutput,  MIN_ABS_SPEED));
    //}
    //else
    //Serial.println();

  }
  else if(angleInput>angleSetpoint) {
    Serial.println(motorController.move(-30));
  }
  else
  Serial.println(motorController.move(30));
  
}
