#include <WiFiManager.h>
#include <L298NX2.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <Preferences.h>

hw_timer_t * timerDCMotor = NULL;
hw_timer_t * timerServo = NULL;
Servo myServo;
const unsigned int servoPin = 18;
//const unsigned int servoPWMChannel = 0; // Use PWM channel 0 for servo

// Motors Pin definition
const unsigned int IN1 = 27;
const unsigned int IN2 = 26;
const unsigned int EN_A = 14;
//const unsigned int motorPWMChannelA = 1; // Use PWM channel 1 for motor A

const unsigned int IN3 = 25;
const unsigned int IN4 = 33;
const unsigned int EN_B = 12;
//const unsigned int motorPWMChannelB = 2; // Use PWM channel 2 for motor B

// Initialize both motors
L298NX2 motors(EN_A, IN1, IN2, EN_B, IN3, IN4);

WiFiManager wifiManager;
WebServer server(80);
Preferences preferences;

bool ledState = LOW; // Maintain the LED state

    volatile char M1_Direction = 'F';
    
    volatile char M2_Direction = 'F';

    volatile int speed_1 = 0;
    volatile int speed_2 = 0;
    volatile int angle = 90;

void IRAM_ATTR onDCTimer() {
  // Control DC motor speed
    motors.setSpeedA(speed_1);
    motors.setSpeedB(speed_2);
    
    if (M1_Direction == 'F') {
        motors.forwardA();
    } else {
        motors.backwardA();
    }

    if (M2_Direction == 'F') {
        motors.forwardB();
    } else {
        motors.backwardB();
    }
}

void IRAM_ATTR onServoTimer() {
  // Control Servo position
  myServo.write(angle);
}


void handleMotionCommand(String uri) {
    if (uri.length() != 14) return; // Validate length

    M1_Direction = uri.charAt(1);
    String M1_Speed = uri.substring(2, 5);
    M2_Direction = uri.charAt(5);
    String M2_Speed = uri.substring(6, 9);
    String angle_wrap = uri.substring(9, 10) + uri.substring(13, 14);
    String angle_str = uri.substring(10, 13);

    if (angle_wrap != "AA") return; // Validate angle wrap

    if ((M1_Direction != 'F' && M1_Direction != 'B') || (M2_Direction != 'F' && M2_Direction != 'B')) return; // Validate directions

    speed_1 = M1_Speed.toInt();
    speed_2 = M2_Speed.toInt();
    
    //bool ServoExecuted = false;
    if (angle >= 60 && angle <= 120) {
        Serial.print("angle set to: ");
        Serial.println(angle);
        angle = angle_str.toInt();
    } else {
        return; // Invalid angle
    }

    Serial.print("speed_1 set to: ");
    Serial.println(speed_1);
    Serial.print(", isForward_1: ");
    Serial.println(M1_Direction == 'F');
    Serial.print("speed_2 set to: ");
    Serial.println(speed_2);
    Serial.print(", isForward_2: ");
    Serial.println(M2_Direction == 'F');



}

void setup() {
    Serial.begin(115200);
    myServo.attach(servoPin);
    pinMode(2, OUTPUT);

    motors.setSpeed(70);


    // Initialize timer for DC motor
    timerDCMotor = timerBegin(0, 80, true); // Timer 0, prescaler 80, count up
    timerAttachInterrupt(timerDCMotor, &onDCTimer, true); // Attach ISR to timer
    timerAlarmWrite(timerDCMotor, 5000, true); // Set alarm to 5ms (5000us) for faster updates
    timerAlarmEnable(timerDCMotor); // Enable timer alarm

    // Initialize timer for Servo
    timerServo = timerBegin(1, 80, true); // Timer 1, prescaler 80, count up
    timerAttachInterrupt(timerServo, &onServoTimer, true); // Attach ISR to timer
    timerAlarmWrite(timerServo, 20000, true); // Set alarm (20000us for 50Hz servo control)
    timerAlarmEnable(timerServo); // Enable timer alarm

    // Try to connect to saved WiFi credentials, if it fails, start AP
    if (!wifiManager.autoConnect("ESP32_AP", "12345678")) {
        Serial.println("Failed to connect to WiFi. Resetting...");
        delay(3000);
        ESP.restart();
    }

    Serial.println("WiFi connected.");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    // Start the mDNS responder for esp32.local
    if (!MDNS.begin("esp32")) { // Domain name esp32.local
        Serial.println("Error setting up MDNS responder!");
    }
    Serial.println("mDNS responder started");

    server.on("/", HTTP_GET, []() {
        server.send(200, "text/html",
                    "<h1>Welcome to ESP32</h1>"
                    "<p><a href=\"/H\">Turn LED on</a></p>"
                    "<p><a href=\"/L\">Turn LED off</a></p>"
                    
                    "<p> deg 60 </p>"
                    "<p><a href=\"/F254F254A060A\">Drive forward fast</a></p>"
                    "<p><a href=\"/F150F150A060A\">Drive forward slow</a></p>"
                    "<p><a href=\"/B254B254A060A\">Drive backward fast</a></p>"
                    "<p><a href=\"/B150B150A060A\">Drive backward slow</a></p>"
                    "<p><a href=\"/F000F000A060A\">Stop</a></p>"

                    "<p> deg 90 </p>"
                    "<p><a href=\"/F254F254A090A\">Drive forward fast</a></p>"
                    "<p><a href=\"/F150F150A090A\">Drive forward slow</a></p>"
                    "<p><a href=\"/B254B254A090A\">Drive backward fast</a></p>"
                    "<p><a href=\"/B150B150A090A\">Drive backward slow</a></p>"
                    "<p><a href=\"/F000F000A090A\">Stop</a></p>"

                    "<p> deg 120 </p>"
                    "<p><a href=\"/F254F254A120A\">Drive forward fast</a></p>"
                    "<p><a href=\"/F150F150A120A\">Drive forward slow</a></p>"
                    "<p><a href=\"/B254B254A120A\">Drive backward fast</a></p>"
                    "<p><a href=\"/B150B150A120A\">Drive backward slow</a></p>"
                    "<p><a href=\"/F000F000A120A\">Stop</a></p>"

                    "<p>IP Address: " + WiFi.localIP().toString() + "</p>"
                    "<p><a href=\"/reset\">Reset WiFi Settings</a></p>"
                    "<h2>Control Robot</h2>"
                    "<form action=\"/send_command\" method=\"GET\">"
                    "Speed 1: <input type=\"text\" name=\"speed1\" maxlength=\"3\"><br>"
                    "Speed 2: <input type=\"text\" name=\"speed2\" maxlength=\"3\"><br>"
                    "Angle: <input type=\"text\" name=\"angle\" maxlength=\"3\"><br>"
                    "<input type=\"submit\" value=\"Send\">"
                    "</form>");
    });

    server.on("/H", HTTP_GET, []() {
        ledState = HIGH;
        digitalWrite(2, ledState);
        server.send(200, "text/html", "<p>LED is now ON</p><p><a href=\"/\">Go Back</a></p>");
    });

    server.on("/L", HTTP_GET, []() {
        ledState = LOW;
        digitalWrite(2, ledState);
        server.send(200, "text/html", "<p>LED is now OFF</p><p><a href=\"/\">Go Back</a></p>");
    });

    server.on("/reset", HTTP_GET, []() {
        server.send(200, "text/html",
                    "<html><body><h1>Resetting WiFi settings...</h1></body></html>");
        wifiManager.resetSettings();
        ESP.restart();
    });

    server.on("/send_command", HTTP_GET, []() {
        String speed1 = server.arg("speed1");
        String speed2 = server.arg("speed2");
        String angle = server.arg("angle");

        // Validate inputs
        if (speed1.length() == 3 && speed2.length() == 3 && angle.length() == 3) {
            int speed1Val = speed1.toInt();
            int speed2Val = speed2.toInt();
            int angleVal = angle.toInt();

            if ((speed1Val >= 0 && speed1Val <= 255) && (speed2Val >= 0 && speed2Val <= 255) && (angleVal >= 60 && angleVal <= 120)) {
                String command = "/F" + speed1 + "F" + speed2 + "A" + angle + "A";
                handleMotionCommand(command);
                server.send(200, "text/html", "<p>Command Sent: " + command + "</p><p><a href=\"/\">Go Back</a></p>");
            } else {
                server.send(400, "text/html", "<p>Invalid values. Speed should be between 0 and 255. Angle should be between 060 and 120.</p><p><a href=\"/\">Go Back</a></p>");
            }
        } else {
            server.send(400, "text/html", "<p>Invalid input length. Each field should be 3 characters long.</p><p><a href=\"/\">Go Back</a></p>");
        }
    });

    server.on("/wifi_credentials", HTTP_GET, []() {
        String html = "<html><body><h1>Stored WiFi Credentials</h1><form method='POST' action='/clear_credentials'>";
        preferences.begin("nvs.net80211", true);
        size_t ssid_len = preferences.getBytesLength("sta.ssid");
        if (ssid_len > 0) {
            char ssid[ssid_len + 1];
            preferences.getBytes("sta.ssid", ssid, ssid_len);
            ssid[ssid_len] = '\0'; // Null-terminate the string
            html += "<p>SSID: " + String(ssid) + "<input type='checkbox' name='ssid' value='" + String(ssid) + "'></p>";
        } else {
            html += "<p>No WiFi credentials found.</p>";
        }
        preferences.end();
        html += "<p><input type='checkbox' id='select_all' onclick='toggle(this)'>Select All</p>"
                "<p><input type='submit' value='Clear Selected'></p>"
                "</form>"
                "<script>"
                "function toggle(source) {"
                "  checkboxes = document.getElementsByName('ssid');"
                "  for(var i=0, n=checkboxes.length;i<n;i++) {"
                "    checkboxes[i].checked = source.checked;"
                "  }"
                "}"
                "</script>"
                "</body></html>";
        server.send(200, "text/html", html);
    });

    server.on("/clear_credentials", HTTP_POST, []() {
        String ssid_to_clear = server.arg("ssid");
        if (!ssid_to_clear.isEmpty()) {
            preferences.begin("nvs.net80211", false);
            preferences.clear();
            preferences.end();
        }
        server.send(200, "text/html",
                    "<html><body><h1>WiFi credentials cleared. Rebooting...</h1></body></html>");
        ESP.restart();
    });

    server.onNotFound([]() {
        String currentLine = server.uri();
        Serial.println(currentLine);
        handleMotionCommand(currentLine);
        server.send(200, "text/plain", "Command Executed");
    });

    server.begin();
}

void loop() {
    // Handle WiFi reconnection if the connection is lost
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi connection lost. Stopping motors and attempting to reconnect...");
        motors.stop();
        while (WiFi.status() != WL_CONNECTED) {
            digitalWrite(2, HIGH); // Turn the LED on
            delay(250);
            digitalWrite(2, LOW); // Turn the LED off
            delay(250);
            WiFi.reconnect();
            Serial.print(".");
            delay(3000);
        }
        Serial.println("Reconnected to WiFi");
    }

    // Check WiFi status and handle client requests
    server.handleClient();

    // Ensure the LED state is maintained correctly
    digitalWrite(2, ledState);
}


