#include <LMotorController.h>
#include <Encoder.h>

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder left(9, 6);
Encoder right(10, 11);

//   avoid using pins with LEDs attached



double motorSpeedFactorLeft = 1;
double motorSpeedFactorRight = 1;
//MOTOR CONTROLLER
const int ENA = 3;
const int IN1 = 4;
const int IN2 = 7;
const int IN3 = 8;
const int IN4 = 12;
const int ENB = 5;
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

const int AVERAGE_COUNT = 20;
float speed_values[AVERAGE_COUNT];
int filter_pos = 0;
void setup() {
  Serial.begin(115200);
  Serial.println("Basic Encoder Test:");
  pinMode(10, INPUT);
  pinMode(11, INPUT);
  pinMode(6, INPUT);
  pinMode(9, INPUT);

   for (int i = 0; i < AVERAGE_COUNT; i++) {
    speed_values[i] = 0; // fill the array with 0's
  }

  //motorController.move(255);
}


long r_wheel, r_wheel_velocity, r_last_wheel=0;
long l_wheel, l_wheel_velocity, l_last_wheel=0;

void loop() {
  l_wheel = left.read();
  r_wheel = right.read();
   Serial.print(l_wheel);
  Serial.print(" : ");
  Serial.println(r_wheel);

  l_wheel_velocity = l_wheel - l_last_wheel;
  l_last_wheel = l_wheel;
  r_wheel_velocity = r_wheel - r_last_wheel;
  r_last_wheel = r_wheel;

  int speed = ((l_wheel_velocity + r_wheel_velocity));

  // let's make an average speed for the last N smaples
  int out = 0;
  speed_values[filter_pos] = speed;
  filter_pos = (filter_pos + 1) % AVERAGE_COUNT;
  for (int i=0; i<AVERAGE_COUNT; i++) {
    out += speed_values[i];
  }
  //Serial.println(out / AVERAGE_COUNT);


}
