#include <WiFi.h>
#include <WebServer.h>

const char* ssid = "OMOTEC 2.4G";
const char* password = "Omo.3567dadar";

WebServer server(80);
#define IN1 16
#define IN2 17
// Motor B pins
#define IN3 18
#define IN4 19

void handlePost() {
String body = server.arg("plain");  // Raw body
  Serial.println("Raw body: " + body);

  if (server.hasArg("value")) {
    String value = server.arg("value");
//    Serial.println("Received value: " + value);
    server.send(200, "text/plain", "Received: " + value);
if (value == "0") {
  motorA_forward();
  motorB_forward();
  delay(500);
  stopMotors();
  Serial.println("FORWARD");
}
else if (value == "1") {
  motorA_reverse();
  motorB_reverse();
  delay(500);
  stopMotors();
  Serial.println("REVERSE");
}
else if (value == "2") {
  motorA_forward();
  motorB_reverse();
  delay(500);
  stopMotors();
  Serial.println("LEFT");
}
else if (value == "3") {
  motorA_reverse();
  motorB_forward();
  delay(500);
  stopMotors();
  Serial.println("RIGHT");
}
else if (value == "4") {
  stopMotors();
  Serial.println("STOP");
}
else {
  Serial.println("Invalid value received: " + value);
}
  } else {
    server.send(400, "text/plain", "Body not received");
  }
}

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("Connected to WiFi");
  Serial.println("IP Address:");
  Serial.println(WiFi.localIP());
  server.on("/send", HTTP_POST, handlePost);
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
}

void motorA_forward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  
}

void motorA_reverse() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
}

// Motor B controls
void motorB_forward() {
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void motorB_reverse() {
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

// Stop both motors
void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}