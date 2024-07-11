#include <WiFiManager.h>
#include <L298NX2.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <Preferences.h>

Servo myServo;
const unsigned int servoPin = 18;

// Motors Pin definition
const unsigned int IN1 = 27;
const unsigned int IN2 = 26;
const unsigned int EN_A = 14;

const unsigned int IN3 = 25;
const unsigned int IN4 = 33;
const unsigned int EN_B = 12;

// Initialize both motors
L298NX2 motors(EN_A, IN1, IN2, EN_B, IN3, IN4);

WiFiManager wifiManager;
WebServer server(80);
Preferences preferences;

bool ledState = LOW; // Maintain the LED state

void handleMotionCommand(const String& currentLine) {
    if (currentLine.length() == 18 && currentLine.startsWith("/GET")) {
        char M1_Direction = currentLine[5];
        String M1_Speed = currentLine.substring(6, 9);
        char M2_Direction = currentLine[9];
        String M2_Speed = currentLine.substring(10, 13);
        String angle_wrap = currentLine.substring(13);
        String angle_str = currentLine.substring(14, 17);

        bool isForward_1 = (M1_Direction == 'F');
        bool isForward_2 = (M2_Direction == 'F');
        bool is_angle = (angle_wrap == "NNNNN") ? false : (angle_wrap[0] == 'A' && angle_wrap[4] == 'A');

        if (is_angle) {
            int angle = angle_str.toInt();
            Serial.print("angle set to: ");
            Serial.println(angle);
            myServo.write(angle);
        }

        int speed_1 = M1_Speed.toInt();
        int speed_2 = M2_Speed.toInt();

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

        if (isForward_1) {
            motors.forwardA();
        } else {
            motors.backwardA();
        }

        if (isForward_2) {
            motors.forwardB();
        } else {
            motors.backwardB();
        }
    }
}

void setup() {
    Serial.begin(115200);
    myServo.attach(servoPin);
    pinMode(2, OUTPUT);

    motors.setSpeed(70);

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
                    "<p><a href=\"/F254F254A120A\">Drive forward fast</a></p>"
                    "<p><a href=\"/F150F150A120A\">Drive forward slow</a></p>"
                    "<p><a href=\"/B254B254A120A\">Drive backward fast</a></p>"
                    "<p><a href=\"/B150B150A120A\">Drive backward slow</a></p>"
                    "<p><a href=\"/F000F000A120A\">Stop</a></p>"
                    "<p>IP Address: " + WiFi.localIP().toString() + "</p>"
                    "<p><a href=\"/reset\">Reset WiFi Settings</a></p>"
                    "<p><a href=\"/wifi_credentials\">Show WiFi Credentials</a></p>");
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
  delay(3000);
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
        }
        Serial.println("Reconnected to WiFi");
    }

    // Check WiFi status and handle client requests
    server.handleClient();

    // Ensure the LED state is maintained correctly
    digitalWrite(2, ledState);
}
