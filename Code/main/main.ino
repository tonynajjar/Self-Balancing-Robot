#include <Wire.h>
#include "Kalman.h"
#include <PID_v1.h>
#include <LMotorController.h>
#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead
#define enc1_outA 3
#define enc1_outB 11
#define enc2_outA 5
#define enc2_outB 6
Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

int pos = 0;
int aState;
int aLastState;

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
double originalSetpoint = 0;
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1;
double input, output;
double realOutput = MIN_ABS_SPEED + 1;
int moveState = 0; //0 = balance; 1 = back; 2 = forth
double Kp = 15;
double Kd = 0;
double Ki = 0.1;

PID innerPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);


double motorSpeedFactorLeft = 1;
double motorSpeedFactorRight = 1;
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
  innerPID.SetTunings(Kp, Kd, Ki);
  pinMode(13, OUTPUT);
  pinMode(enc1_outA, INPUT);
  pinMode(enc1_outB, INPUT);
  pinMode(enc2_outA, INPUT);
  pinMode(enc2_outB, INPUT);
  aLastState = digitalRead(outputA);
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

  innerPID.SetMode(AUTOMATIC);
  innerPID.SetSampleTime(10);
  innerPID.SetOutputLimits(-255, 255);
}

void loop() {
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

  /* Print Data */
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

  if (Serial.available()) {
    counter = millis();
    char c = Serial.read();

    Serial.println(c);

    switch (c) {

      case 'w':
        setpoint = -3;
        break;

      case 's':
        setpoint = -13;
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
      setpoint = originalSetpoint;
    }

  }


  aState = digitalRead(outputA); // Reads the "current" state of the outputA
  // If the previous and the current state of the outputA are different, that means a Pulse has occured
  if (aState != aLastState) {
    // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
    if (digitalRead(outputB) != aState) {
      pos ++;
    } else {
      pos --;
    }
  }
  aLastState = aState; // Updates the previous state of the outputA with the current state



  input = kalAngleY;
  Kp = map(analogRead(A0), 0, 1023, 0, 30);
  Kd = double(analogRead(A2)) / 1023.0 * 20.0;
  Ki = double(analogRead(A1)) / 1000.0;
  innerPID.SetTunings(Kp, Kd, Ki);
  innerPID.Compute();
  Serial.print("Kp= ");
  Serial.print(Kp);
  Serial.print("; ");
  Serial.print("Ki= ");
  Serial.print(Ki);
  Serial.print("; ");
  Serial.print("Kd= ");
  Serial.print(Kd);
  Serial.print("; ");
  Serial.print(input);
  Serial.print(": ");
  Serial.print("Position: ");
  Serial.print(pos);
  Serial.print(": ");
  /*
    if(abs(output)>MIN_ABS_SPEED)
    Serial.println(output);
    else if(output>0)
    Serial.println(MIN_ABS_SPEED);
    else
    Serial.println(-MIN_ABS_SPEED);
  */
  realOutput = motorController.getSpeed(output, MIN_ABS_SPEED);

  if (input > 0.3 || input < -0.3) {

    //if(realOutput> MIN_ABS_SPEED || realOutput< -MIN_ABS_SPEED){
    motorController.move(realOutput);
    Serial.println(realOutput);
    //}
    //else
    //Serial.println();

  }
  else {
    motorController.move(40 * realOutput / abs(realOutput));
    Serial.println();
  }



}
