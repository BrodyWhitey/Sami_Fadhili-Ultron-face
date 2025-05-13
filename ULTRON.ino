#include "Arduino.h"
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"
#include <Servo.h>

// Servo Configuration
const int servo1Pin = 2;  // X-axis (left/right)
const int servo2Pin = 3;  // Y-axis (up/down)
Servo myservo1, myservo2;

// Servo positions with smoothing
int servo_X = 75;  // Left:60, Center:75, Right:90
int servo_Y = 95;  // Up:70, Center:95, Down:110
const int numReadings = 5;  // Number of samples for smoothing
int xReadings[numReadings], yReadings[numReadings];
int readIndex = 0;
int xTotal = 75 * numReadings;
int yTotal = 95 * numReadings;

// DFPlayer Mini configuration
SoftwareSerial mySoftwareSerial(10, 11); // RX, TX
DFRobotDFPlayerMini myDFPlayer;

// Finger tracking
bool currentFingers[5] = {false};  // Tracks current finger states
unsigned long lastUpdateTime = 0;   // For debouncing
const unsigned long debounceDelay = 300;  // Minimum ms between finger updates

void setup() {
  // Initialize servos with smoothing buffers
  myservo1.attach(servo1Pin);
  myservo2.attach(servo2Pin);
  
  // Fill smoothing buffers with initial values
  for (int i = 0; i < numReadings; i++) {
    xReadings[i] = servo_X;
    yReadings[i] = servo_Y;
  }
  
  // Move to center position
  updateServos();
  delay(1000);  // Allow servos to settle

  // Initialize DFPlayer Mini
  mySoftwareSerial.begin(9600);
  Serial.begin(115200);
  
  if (!myDFPlayer.begin(mySoftwareSerial)) {
    Serial.println(F("DFPlayer initialization failed!"));
    while(true);  // Halt if player fails
  }
  
  // Configure player settings
  myDFPlayer.volume(5);  // Medium volume (0-30)
  myDFPlayer.EQ(DFPLAYER_EQ_NORMAL);
  myDFPlayer.outputDevice(DFPLAYER_DEVICE_SD);
  Serial.println(F("System ready"));
}

void loop() {
  handleSerialCommands();  // Process incoming serial data
  updateServos();         // Smoothly update servo positions
  
  
  delay(10);  // Small delay for stability
}

void handleSerialCommands() {
  // Process any available serial commands
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    if (input.startsWith("SERVO,")) {
      processServoCommand(input);
    }
    else if (input.length() >= 9) {  // Minimum length for finger command "0,0,0,0,0"
      processFingerCommand(input);
    }
  }
}

void processServoCommand(String input) {
  // Parse servo position command (format: "SERVO,X,Y")
  input = input.substring(6);  // Remove "SERVO,"
  int commaPos = input.indexOf(',');
  
  if (commaPos != -1) {
    // Extract and constrain X position (60-90)
    int newX = constrain(input.substring(0, commaPos).toInt(), 50, 100);
    
    // Extract and constrain Y position (70-110)
    int newY = constrain(input.substring(commaPos + 1).toInt(), 60, 120);
    
    // Update smoothing buffers
    xTotal = xTotal - xReadings[readIndex] + newX;
    yTotal = yTotal - yReadings[readIndex] + newY;
    
    xReadings[readIndex] = newX;
    yReadings[readIndex] = newY;
    
    readIndex = (readIndex + 1) % numReadings;  // Circular buffer
  }
}

void updateServos() {
  // Calculate smoothed positions
  int smoothX = xTotal / numReadings;
  int smoothY = yTotal / numReadings;
  
  // Only update if position changed significantly
  if (abs(smoothX - servo_X) > 1 || abs(smoothY - servo_Y) > 1) {
    servo_X = smoothX;
    servo_Y = smoothY;
    myservo1.write(servo_X);
    myservo2.write(servo_Y);
  }
}

void processFingerCommand(String input) {
  // Only process if debounce delay has passed
  if (millis() - lastUpdateTime < debounceDelay) return;
  
  bool newFingers[5] = {false};
  
  // Parse each finger state (format: "1,0,1,0,1")
  for (int i = 0; i < 5; i++) {
    int commaPos = input.indexOf(',');
    String valStr = (i < 4) ? input.substring(0, commaPos) : input;
    newFingers[i] = (valStr.toInt() == 1);
    
    // Play sound if finger just opened
    if (newFingers[i] && !currentFingers[i]) {
      myDFPlayer.play(i + 1);  // Play corresponding track (1-5)
      delay(2000);
    }
    
    input = (i < 4) ? input.substring(commaPos + 1) : "";
  }
  
  // Update current finger states
  memcpy(currentFingers, newFingers, sizeof(currentFingers));
  lastUpdateTime = millis();
}