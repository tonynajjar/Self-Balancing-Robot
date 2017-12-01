#include <SoftwareSerial.h>

SoftwareSerial mySerial(13, 1);
//SoftwareSerial mySerial1(0, 1);
unsigned long timer;
void setup() {
  // put your setup code here, to run once:
mySerial.begin(115200);
//Serial.begin(9600);
//mySerial.listen();
}

void loop() {
  if(mySerial.available()){
      timer= millis();
      char c= mySerial.read();
      
      mySerial.println(c);
      
  }
  else {
    if(millis()-timer > 500){
    }
    
  }

}

