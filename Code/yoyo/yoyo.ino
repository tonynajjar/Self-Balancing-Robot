#include <Wire.h>
#include <PID_v1.h>
#include <Encoder.h>
#include "Kalman.h"
#include <LMotorController.h>
#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead
Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

double accX, accY, accZ;
double gyroX, gyroY, gyroZ;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data
unsigned long counter;


//Define Variables we'll be connecting to
double speedSetpoint = 0; // in a perfect world, 0 would be straight up and down
double speedInput;
double speedOutput;

double angleSetpoint = 0;
double angleInput;
double angleOutput;

const int AVERAGE_COUNT = 20;
float speed_values[AVERAGE_COUNT];
int filter_pos = 0;


//Specify the links and initial tuning parameters
PID speedPID(&speedInput, &speedOutput, &speedSetpoint, 0, 0, 0, DIRECT);
PID anglePID(&angleInput, &angleOutput, &angleSetpoint, 0, 0, 0, REVERSE);



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

Encoder left(5, 6);
Encoder right(3, 11);

int count = 0;

void setup() {

  pinMode(13, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(9, OUTPUT);
  
  pinMode(3, INPUT);
  pinMode(11, INPUT);
  pinMode(5, INPUT);
  pinMode(6, INPUT);

  for (int i = 0; i < AVERAGE_COUNT; i++) {
    speed_values[i] = 0; // fill the array with 0's
  }


  Serial.begin(115200);
  Wire.begin();
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

  // turn the PIDs on
  anglePID.SetMode(AUTOMATIC);
  anglePID.SetSampleTime(10);
  anglePID.SetOutputLimits(-100, 100);
  anglePID.SetTunings(25, 10, 0);

  speedPID.SetMode(AUTOMATIC);
  speedPID.SetSampleTime(10);
  speedPID.SetOutputLimits(-300, 300); // -3 and +3 degrees, respectively
  speedPID.SetTunings(25, 50, 0);

}

long r_wheel, r_wheel_velocity, r_last_wheel;
long l_wheel, l_wheel_velocity, l_last_wheel;

void loop(){

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


  // read knobs for angle pid
  double knob0 = analogRead(0);
  double knob1 = analogRead(1);
  double knob2 = analogRead(2);
  double akp = map(knob0, 0, 1023, 0, 30);
  double aki = map(knob1, 0, 1023, 0, 100);
  double akd = knob2/682;

  // read knobs for speed pid
  //double knob2 = analogRead(2);
  double knob3 = 200;
  double skp = 0;
  double ski = 0;

 //best tunings till now: 5.3 45 0.3
 anglePID.SetTunings(5.6, 48, 0.33);
 
 speedPID.SetTunings(0, 0, 0);

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

  // calculate angle setpoint
  speedPID.Compute();
  //angleSetpoint = speedOutput / 100;
  angleSetpoint = -1.3;

 
  /*
    // panic!
    if(abs(angle) >= 20) {
    stop_robot(5000);
    }
  */

  angleInput = kalAngleY;
  anglePID.Compute();

  float go = angleOutput;

  if (go > 0) {
    int speed = map(go, 0, 100, 10, 255);
    motorController.move(speed);
  } else {
    int speed = abs(go);
    speed = map(speed, 0, 100, 10, 255);
    motorController.move(-speed);
  }

  //if(0 == count % 10) {
  Serial.print(akp);
  Serial.print("\t");
  Serial.print(aki);
  Serial.print("\t");
  Serial.print(akd);
  Serial.print("\t");
  /*
  Serial.print(skp);
  Serial.print("\t");
  Serial.print(ski);
  Serial.print("\t");
  */
  Serial.print(angleInput);
  Serial.print("\t");
  Serial.print(angleOutput);
  Serial.print("\t");
  Serial.print(angleSetpoint);
  Serial.print("\t");
  Serial.print(speedInput);
  Serial.print("\t");
  Serial.println(speed);

  //}
  count++;
}

/*
void stop_robot(int duration) {
  motorOff(0);
  motorOff(1);
  delay(duration);
}

void motorOff(int motor) {
  // Initialize braked
  for (int i = 0; i < 2; i++) {
    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], LOW);
  }
  analogWrite(pwmpin[motor], 0);


void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm) {
  if (motor <= 1) {
    if (direct <= 4) {

      // Set inA[motor]
      if (direct <= 1)
        digitalWrite(inApin[motor], HIGH);
      else
        digitalWrite(inApin[motor], LOW);

      // Set inB[motor]
      if ((direct == 0) || (direct == 2))
        digitalWrite(inBpin[motor], HIGH);
      else
        digitalWrite(inBpin[motor], LOW);

      analogWrite(pwmpin[motor], pwm);

    }
  }
}
}
*/


