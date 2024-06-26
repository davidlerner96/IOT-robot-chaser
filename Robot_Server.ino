/*
 WiFi Web Server For 

web server that lets you send commands to the Robot via the web.
 This sketch will print the IP address of your WiFi Shield (once connected)
 to the Serial monitor. From there, you can open that address in a web browser
 to set and activate the motors speed and direction.

 If the IP address of your shield is yourAddress:
 http://yourAddress/H turns the LED on
 http://yourAddress/L turns it off
 http://yourAddress/dsssDSSS sets the first motor direction as d and speed as sss, second motor direction as D and speed as SSS.


 Circuit:
 * WiFi shield attached
 * LED attached to pin 2


 */
#include <L298NX2.h>
#include <WiFi.h>
#include <ESP32Servo.h>


Servo myServo;
const unsigned int servoPin=18;
// Motors Pin definition
const unsigned int IN1 = 27;
const unsigned int IN2 = 26;
const unsigned int EN_A = 14;



const unsigned int IN3 = 25;
const unsigned int IN4 = 33;
const unsigned int EN_B = 12;

// Initialize both motors
L298NX2 motors(EN_A, IN1, IN2, EN_B, IN3, IN4);

const char *ssid = "Maor";
const char *password = "0549277459";

NetworkServer server(80);

void setup() {
  delay(1000);
  Serial.begin(115200);
  myServo.attach(servoPin);
  delay(2000);
  pinMode(2, OUTPUT);  // set the LED pin mode

  delay(10);
  motors.setSpeed(70);
  // We start by connecting to a WiFi network


  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  server.begin();
}

void loop() {
    bool isForward_1 = 0;
    bool isForward_2 = 0;
    int speed_1 = 0;
    int speed_2 = 0;
    bool is_angle = 0;
int reconnect_count=1;
      while (WiFi.status() != WL_CONNECTED) {
        motors.stop(); 
        WiFi.reconnect();
        delay(500);
        Serial.print("reconnection attempt: ");
        Serial.print(reconnect_count, DEC);
        Serial.println(" failed. ");
        reconnect_count++;
  }
   
  NetworkClient client = server.accept();  // listen for incoming clients

  if (client) {                     // if you get a client,
    Serial.println("New Client.");  // print a message out the serial port
    String currentLine = "";        // make a String to hold incoming data from the client
    while (client.connected()) {    // loop while the client's connected
      if (client.available()) {     // if there's bytes to read from the client,
        char c = client.read();     // read a byte, then
        Serial.write(c);            // print it out the serial monitor
        if (c == '\n') {            // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            // the content of the HTTP response follows the header:
            client.print("Click <a href=\"/H\">here</a> to turn the LED on pin 2 on.<br>");
            client.print("Click <a href=\"/L\">here</a> to turn the LED on pin 2 off.<br>");
            client.print("Click <a href=\"/F254F254A120A\">here</a> to drive forward fast.<br>");
            client.print("Click <a href=\"/F150F150A120A\">here</a> to drive forward slow.<br>");
            client.print("Click <a href=\"/B254B254A120A\">here</a> to drive backward fast.<br>");
            client.print("Click <a href=\"/B150B150A120A\">here</a> to drive backward slow.<br>");
            client.print("Click <a href=\"/F000F000A120A\">here</a> to stop.<br>");             
            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            break;
          } else {  // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }

        // Check to see if the client request was "GET /H" or "GET /L":
        if (currentLine.endsWith("GET /H")) {
          digitalWrite(2, HIGH);  // GET /H turns the LED on
        }
        if (currentLine.endsWith("GET /L")) {
          digitalWrite(2, LOW);  // GET /L turns the LED off
        }
        if(currentLine.length()==18&&currentLine.startsWith("GET")){
  //          Serial.println();
  //                     Serial.print("str is: ");
  //           Serial.println(currentLine);
  //           Serial.print("str len is: ");
  // Serial.println(currentLine.length(),DEC);

          char M1_Direction = currentLine[5];
          String M1_Speed = currentLine.substring(6,9);
          char M2_Direction = currentLine[9];
          String M2_Speed = currentLine.substring(10,13);
          String angle_wrap = currentLine.substring(13);
          String angle_str =  currentLine.substring(14,17);
          if(angle_wrap=="NNNNN")
            is_angle = 0;
          else if(angle_wrap[0]=='A' && angle_wrap[4]=='A')
            is_angle = 1;
          else
            continue;
          //   Serial.print("angle_wrap is "+ angle_wrap+ ", is_angle: ");
          //  Serial.println(is_angle);
          if(M1_Direction == 'F')
            isForward_1 = 1;
           else if(M1_Direction == 'B')
            isForward_1 = 0;
           else
            continue;
          if(M2_Direction == 'F')
            isForward_2 = 1;
           else if(M2_Direction == 'B')
            isForward_2 = 0;
           else
            continue;

          int speed_1 = M1_Speed.toInt();
          int speed_2 = M2_Speed.toInt();
          int angle = angle_str.toInt();
          if(is_angle){
            Serial.print("angle set to: ");
            Serial.println(angle);
            myServo.write(angle);
          }
                               Serial.print("speed_1 set to: ");
           Serial.println(speed_1);
                                          Serial.print(", isForward_1: ");
           Serial.println(isForward_1);
                               Serial.print("speed_2 set to: ");
           Serial.println(speed_2);
                                          Serial.print(", isForward_2: ");
           Serial.println(isForward_2);
           

          motors.setSpeedA(speed_1);
          motors.setSpeedB(speed_2);
          if(isForward_1)
            motors.forwardA();
          else
            motors.backwardA();

          if(isForward_2)
            motors.forwardB();
          else
            motors.backwardB();
          // delay(3000);
          // motors.stop();  
        }  
      }
    }
    // close the connection:
    client.stop();
    Serial.println("Client Disconnected.");
  }
}
