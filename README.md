Making Your own Hand-Gesture RC Car

Have you ever wanted to control a robot just by waving your hand? In this article, I’ll walk you through how I built a gesture-controlled robotic car using an ESP32 NodeMCU, a gesture recognition model, and a powerful motor driver (L298N). This project combines computer vision, embedded systems, and real-time control — and it’s surprisingly modular and customizable.

🧠 The Idea
The goal was simple: recognize hand gestures via webcam, and use them to wirelessly control a car’s movement — forward, backward, left, right, and more.

I used a pre-trained hand gesture recognition model from GitHub that can recognize 6 unique gestures. Once a gesture is detected, the model outputs a number between 0 and 5. Each of these numbers corresponds to a specific car movement.

Here is the link to the project i used for gesture recognition model .

GitHub - Sousannah/real-time-hand-gestures-recognition-using-cvzone
Contribute to Sousannah/real-time-hand-gestures-recognition-using-cvzone development by creating an account on GitHub.
github.com

You can train your own model with the help of these tutorials but the most important part is you need a numerical output from that model.



I have also added the Link to my Github Repo at the end of this

🧰 What You Need
ESP32 NodeMCU (or any microcontroller with Wi-Fi capability)

L298N Motor Driver Module

4 DC Motors (high torque, 12V recommended)

Laptop with a webcam (for gesture recognition)
Python (for the gesture recognition script)

Common jumper wires, breadboard, and power supply

12V rechargable battery

🖥️ Libs required
Tensorflow 2.10.0
Mediapipe 0.8.10.1
Python 3.7 or 3.10.0
Protobuf 3.19.6
OpenCv
Numpy
Note: make sure to install this specific versions of these libs

🖥️ Gesture Recognition Model
I started by running a pre-trained gesture recognition model (from GitHub) on my computer. It uses the webcam to track hand positions and outputs a number from 0 to 5 depending on the gesture shown.

These numbers are then transmitted to the ESP32 via WebSocket, enabling low-latency, bidirectional communication.

You can also train your own custom gesture model if you need more specific controls. This makes the system extremely flexible.

📡 WebSocket Communication
Both the laptop (running the recognition script) and the ESP32 must be connected to the same Wi-Fi network. The ESP32 runs a small WebSocket server that listens for incoming gesture codes.

Once it receives a number (say 3), it interprets it as a command (e.g., move left) and sets its output pins accordingly.

🔌 ESP32 and Motor Driver Wiring
The ESP32 uses 4 GPIO output pins (16, 17, 18, and 19). These pins are connected to the input pins of the L298N Motor Driver:

ESP32 PinL298N IN

Pin 16->Input 1

Pin17->Input 2

Pin 18->Input 3

Pin 19->Input 4

🔌 L298N Connections
Connect the negative of both motors from one side to Output 1 of the L298N and positive to the Output 2 of the L298N. Do the same for the other motors as well negative of both motors from other side will go on Output 3 and positives of both motors will go on output 4 of the L298N

Each combination of HIGH/LOW on these pins controls the motors in different ways:

Forward
Backward
Turn Left
Turn Right
Finally connect the 12V battery on the 12v input

🔌 Code
This is the code for the model use

Note: I have used a premade model (Github link provided above) so if you try to change the model you need to make changes in the code accordingly

import requests
import cv2
from cvzone.HandTrackingModule import HandDetector
from cvzone.ClassificationModule import Classifier
import numpy as np
import math


# Replace with the IP address of your ESP32
esp32_ip = "http://***.***.*.**"  # Make sure this matches the ESP32's IP
endpoint = "/send"


#//------------------------------
cap = cv2.VideoCapture(0)
detector = HandDetector(maxHands=1)
classifier = Classifier('keras_model.h5', "labels.txt")
offset = 20
imgSize = 300
counter = 0
labels =  ["Down", "Up", "Right", "Back", "Left","Front" ]

while True:
    success, img = cap.read()
    imgOutput = img.copy()
    hands, img = detector.findHands(img, draw=False)
    if hands:
        hand = hands[0]
        x, y, w, h = hand['bbox']

        imgWhite = np.ones((imgSize, imgSize, 3), np.uint8) * 255
        imgCrop = img[y - offset:y + h + offset, x - offset:x + w + offset]

        imgCropShape = imgCrop.shape

        aspectRatio = h / w

        if aspectRatio > 1:
            k = imgSize / h
            wCal = math.ceil(k * w)
            imgResize = cv2.resize(imgCrop, (wCal, imgSize))
            imgResizeShape = imgResize.shape
            wGap = math.ceil((imgSize - wCal) / 2)
            imgWhite[:, wGap:wCal + wGap] = imgResize
            prediction, index = classifier.getPrediction(imgWhite, draw=True)

        else:
            k = imgSize / w
            hCal = math.ceil(k * h)
            imgResize = cv2.resize(imgCrop, (imgSize, hCal))
            imgResizeShape = imgResize.shape
            hGap = math.ceil((imgSize - hCal) / 2)
            imgWhite[hGap:hCal + hGap, :] = imgResize
            prediction, index = classifier.getPrediction(imgWhite, draw=True)

        cv2.rectangle(imgOutput, (x - offset, y - offset-50),
                      (x - offset+90, y - offset-50+50), (255, 0, 255), cv2.FILLED)
        cv2.putText(imgOutput, labels[index], (x, y -26), cv2.FONT_HERSHEY_COMPLEX, 1.7, (255, 255, 255), 2)
        cv2.rectangle(imgOutput, (x-offset, y-offset),
                      (x + w+offset, y + h+offset), (255, 0, 255), 4)

        url = esp32_ip + endpoint
        data = {'value': str(index)}  # Replace with the value you want to send

        try:
            response = requests.post(url, data=data)
            print("Status Code:", response.status_code)
            print("Response:", response.text)
        except requests.exceptions.RequestException as e:
            print("Failed to send data:", e)

        
        # cv2.imshow("ImageCrop", imgCrop)
        # cv2.imshow("ImageWhite", imgWhite)

    cv2.imshow("Image", imgOutput)
    cv2.waitKey(1)
     # Exit when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
# Release the webcam and close all windows
cap.release()
cv2.destroyAllWindows()
Following is the code for the car’s movement using Esp32S . It’s simple and can be modified to fit your requirements if you decide to change something.

#include <WiFi.h>
#include <WebServer.h>

const char* ssid = "Wifi-Name";
const char* password = "Wifi-Password";

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
// Motor A controls
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
I chose the L298N because I was using larger motors that required higher voltage and current. This dual-channel motor driver is ideal for a 4-wheel drive robotic platform.

How to start the project
Once everything is done run the esp32 code and get the IP address form the serial monitor and paste it in the python code above . Make sure both devices (Laptop and esp) are connected to the same wifi.

🛠️ Customizing the Gestures
The beauty of this project is its customizability:

Map any of the 6 gestures to any direction or action.
Use all 4 motors, or adapt for 2-wheel drive.
Replace the model with one you train using your own custom signs.
This flexibility makes it suitable not just for robotics hobbyists, but also for teaching embedded AI, gesture control, and IoT integration.

🎯 Final Thoughts
This was a fun and rewarding project that blends computer vision with robotics. You don’t need expensive sensors — just a camera and a bit of smart code.
