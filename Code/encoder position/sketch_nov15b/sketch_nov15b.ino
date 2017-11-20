/*     Arduino Rotary Encoder Tutorial
 *      
 *  by Dejan Nedelkovski, www.HowToMechatronics.com
 *  
 */
 #include <LMotorController.h>
 
 #define outputA 3
 #define outputB 11

//MOTOR CONTROLLER
const int ENA = 10;
const int IN1 = 7;
const int IN2 = 4;
const int IN3 = 8;
const int IN4 = 12;
const int ENB = 9;
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, 1, 1);
 
 int counter = 0; 
 int aState;
 int aLastState;  
 void setup() { 
   pinMode (outputA,INPUT);
   pinMode (outputB,INPUT);
   
   Serial.begin (115200);
   // Reads the initial state of the outputA
   aLastState = digitalRead(outputA); 
   motorController.move(255);
 } 
 void loop() { 
   aState = digitalRead(outputA); // Reads the "current" state of the outputA
   // If the previous and the current state of the outputA are different, that means a Pulse has occured
   if (aState != aLastState){     
     // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
     if (digitalRead(outputB) != aState) { 
       counter ++;
     } else {
       counter --;
     }
     Serial.print("Position: ");
     Serial.println(counter);
   } 
   aLastState = aState; // Updates the previous state of the outputA with the current state
 }
