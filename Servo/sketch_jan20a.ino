#include <WiFi.h>
#include <ESP32Servo.h>

const char* ssid = "ESP32-Gripper";
const char* password = "12345678";

WiFiServer server(80);
Servo myservo;

// Pin configuration
const int servoPin = 13;

// Servo angles
const int openAngle = 0;
const int closeAngle = 180;

void setup() {
  Serial.begin(115200);
  
  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  myservo.setPeriodHertz(50);    // standard 50 hz servo
  myservo.attach(servoPin, 1000, 2000); // attach the servo on pin 18 to the servo object
  
  // Create WiFi Access Point
  Serial.println();
  Serial.print("Configuring Access Point...");
  
  // You can remove the password argument to make it an open network
  WiFi.softAP(ssid, password);

  Serial.println("AP Created");
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());
  
  server.begin();
}

void loop() {
  WiFiClient client = server.available();   // listen for incoming clients

  if (client) {                             // if you get a client,
    Serial.println("New Client.");           // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        if (c == '\n') {                    // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            // the content of the HTTP response follows the header:
            client.print("Click <a href=\"/open\">here</a> to open gripper.<br>");
            client.print("Click <a href=\"/close\">here</a> to close gripper.<br>");

            // The HTTP response ends with another blank line:
            client.println();
            break;
          } else {    // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }

        // Check to see if the client request was "GET /open" or "GET /close":
        if (currentLine.endsWith("GET /open")) {
          myservo.write(openAngle);
          Serial.println("Gripper OPEN");
        }
        if (currentLine.endsWith("GET /close")) {
          myservo.write(closeAngle);
          Serial.println("Gripper CLOSED");
        }
      }
    }
    // close the connection:
    client.stop();
    Serial.println("Client Disconnected.");
  }
}
