unsigned long timer;
void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
pinMode(13,OUTPUT);
}

void loop() {
  if(Serial.available()){
      timer= millis();
      char c= Serial.read();
      
      Serial.println(c);
      
      switch(c){
        
        case 'w':
          digitalWrite(13,HIGH);
          break;

         case 's':
         digitalWrite(13,HIGH);
         break;

         case 'a':
         digitalWrite(13,HIGH);
         break;

         case 'd':
         digitalWrite(13,HIGH);
         break;
      }
  }
  else {
    if(millis()-timer > 500){
      digitalWrite(13,LOW);
    }
    
  }

}

