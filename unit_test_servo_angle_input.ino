#include <ESP32Servo.h>

Servo myServo;
int servoPin=18;
void setup() {
  // put your setup code here, to run once:
  myServo.attach(servoPin);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available()){
    int angle = Serial.parseInt();
   // String str = Serial.readString();
    myServo.write(angle);

    Serial.print("set angle to ");
    Serial.println(angle, DEC);
  }
delay(20);
}
