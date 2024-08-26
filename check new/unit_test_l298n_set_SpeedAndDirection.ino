// Include the library
#include <L298N.h>

// Pin definition
const unsigned int IN1 = 27;
const unsigned int IN2 = 26;
const unsigned int EN = 14;

// Create one motor instance
L298N motor(EN, IN1, IN2);


void setup() {
  // Used to display information
  Serial.begin(9600);

  // Wait for Serial Monitor to be opened
  while (!Serial)
  {
    //do nothing
  }
  // Set initial speed
  motor.setSpeed(70);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available()){
    int num = Serial.parseInt();
    Serial.print("num parsed is: ");
    Serial.println(num,DEC);
    bool isForward = 0;
    int speed = 0;
    if(num>999){
      isForward = num/1000 == 1? 1 : 0;
      speed = num%1000;
      
    }else if(num > 99){
      isForward = num/100 == 1? 1 : 0;
      speed = num%100;
    }else{
      isForward = num/10 == 1? 1 : 0;
      speed = num%10;
    }

          Serial.print("isForward is: ");
    Serial.println(isForward,DEC);
              Serial.print("speed is: ");
    Serial.println(speed,DEC);


    motor.setSpeed(speed);
    printSomeInfo();
    if(isForward){
      // Tell the motor to go forward (may depend by your wiring)
  motor.forward();

    }else{
        // Tell the motor to go back (may depend by your wiring)
         motor.backward();
    }
     printSomeInfo();
     delay(3000);
   

    // Stop
  motor.stop();

  printSomeInfo();

  delay(3000);

  }
delay(20);
}
/*
Print some informations in Serial Monitor
*/
void printSomeInfo()
{
  Serial.print("Motor is moving = ");
  Serial.print(motor.isMoving());
  Serial.print(" at speed = ");
  Serial.println(motor.getSpeed());
}
