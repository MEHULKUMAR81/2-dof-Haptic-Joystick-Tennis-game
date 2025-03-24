#include <Arduino.h>

// Define sensor pins
const int verticalSensorPin = A1;  // Vertical sensor connected to A1
const int horizontalSensorPin = A2; // Horizontal sensor connected to A2

// Motor pins
int motor1Pin1 = 5;
int motor1Pin2 = 8;
int motor2Pin1 = 6;
int motor2Pin2 = 7;

// Position tracking variables x axis
int updatedPosx = 0;     // keeps track of the latest updated value of the MR sensor reading
int rawPosx = 0;         // current raw reading from MR sensor
int lastRawPosx = 0;     // last raw reading from MR sensor
int lastLastRawPosx = 0; // last last raw reading from MR sensor
int flipNumberx = 0;     // keeps track of the number of flips over the 180deg mark
int tempOffsetx = 0;
int rawDiffx = 0;
int lastRawDiffx = 0;
int rawOffsetx = 0;
int lastRawOffsetx = 0;
int forcex = 0;

// Position tracking variables x axis
int updatedPosy = 0;     // keeps track of the latest updated value of the MR sensor reading
int rawPosy = 0;         // current raw reading from MR sensor
int lastRawPosy = 0;     // last raw reading from MR sensor
int lastLastRawPosy = 0; // last last raw reading from MR sensor
int flipNumbery = 0;     // keeps track of the number of flips over the 180deg mark
int tempOffsety = 0;
int rawDiffy = 0;
int lastRawDiffy = 0;
int rawOffsety = 0;
int lastRawOffsety = 0;
int forcey = 0;

const int flipThreshx = 700;  // threshold to determine whether or not a flip over the 180 degree mark occurred
boolean flippedx = false;
double OFFSETx = 980;
double OFFSET_NEGx = 15;

const int flipThreshy = 700;  // threshold to determine whether or not a flip over the 180 degree mark occurred
boolean flippedy = false;
double OFFSETy = 980;
double OFFSET_NEGy = 15;

// Kinematics variables x axis
double xh = 0;           // position of the handle [m]
double lastxh = 0;       // last calculated value of xh
double lastLastxh = 0;   // last last calculated value of xh
double dxh= 0;           // Velocity of the handle
double lastdxh= 0;      // last calculated value of dxh
double lastLastdxh= 0;   // last last calculated value of dxh
double dxh_filt= 0;              // Filtered velocity of the handle

// Kinematics variables y axis
double yh = 0;           // position of the handle [m]
double lastyh = 0;       // last calculated value of xh
double lastLastyh = 0;   // last last calculated value of xh
double dyh= 0;           // Velocity of the handle
double lastdyh= 0;      // last calculated value of dxh
double lastLastdyh= 0;   // last last calculated value of dxh
double dyh_filt= 0;              // Filtered velocity of the handle

// Feedback variables
unsigned long previousMillis = 0;
unsigned long interval = 50;  // Interval for sine wave oscillation
float sineValue = 0.0;  // To store the sine wave value
float sineFrequency = 0.2;  // Frequency of the sine wave oscillation
float sineAmplitude = 100.0;  // Reduced amplitude for subtle sine wave feedback
float triangleAmplitude = 50.0; // Reduced amplitude for subtle sharp texture bumps

void setup() {
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  Serial.begin(115200);  // Start serial communication for debugging
}

void loop() {
  unsigned long currentMillis = millis();

  // Check for serial commands from Processing
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "BallHit") {
      triggerHighIntensityFeedback(200);  // High-intensity feedback for ball hit
    } else if (command == "WallHitTop") {
      triggerHighIntensityFeedbackYTop(200);  // Longer feedback for wall/half-court hit
    } else if (command == "WallHitBottom") {
      triggerHighIntensityFeedbackYBottom(200);  // Longer feedback for wall/half-court hit
    }else if (command == "WallHitBack") {
      triggerHighIntensityFeedbackXBack(200);  // Longer feedback for wall/half-court hit
    } else if (command == "WallHitNet") {
      triggerHighIntensityFeedbackXNet(200);  // Longer feedback for wall/half-court hit
    }else if (command == "UserMoving") {
      generateTextureFeedback(currentMillis);  // Continuous feedback
    } else if (command == "Stop") {
      stopMotors();  // Stop all motor feedback
    }
  }

  // Serial debugging for sensor values
  rawPosy = analogRead(verticalSensorPin);
  rawDiffy = rawPosy - lastRawPosy;          //difference btwn current raw position and last raw position
  lastRawDiffy = rawPosy - lastLastRawPosy;  //difference btwn current raw position and last last raw position
  rawOffsety = abs(rawDiffy);
  lastRawOffsety = abs(lastRawDiffy);
  
  // Update position record-keeping vairables
  lastLastRawPosy = lastRawPosy;
  lastRawPosy = rawPosy;
  
  // Keep track of flips over 180 degrees
  if((lastRawOffsety > flipThreshy) && (!flippedy)) { // enter this anytime the last offset is greater than the flip threshold AND it has not just flipped
    if(lastRawDiffy > 0) {        // check to see which direction the drive wheel was turning
      flipNumbery--;              // cw rotation 
    } else {                     // if(rawDiff < 0)
      flipNumbery++;              // ccw rotation
    }
    flippedy = true;            // set boolean so that the next time through the loop won't trigger a flip
  } else {                        // anytime no flip has occurred
    flippedy = false;
  }
   updatedPosy = rawPosy + flipNumbery*OFFSETy;

  rawPosx = analogRead(horizontalSensorPin);
  rawDiffx = rawPosx - lastRawPosx;          //difference btwn current raw position and last raw position
  lastRawDiffx = rawPosx - lastLastRawPosx;  //difference btwn current raw position and last last raw position
  rawOffsetx = abs(rawDiffx);
  lastRawOffsetx = abs(lastRawDiffx);
  
  // Update position record-keeping vairables
  lastLastRawPosx = lastRawPosx;
  lastRawPosx = rawPosx;
  
  // Keep track of flips over 180 degrees
  if((lastRawOffsetx > flipThreshx) && (!flippedx)) { // enter this anytime the last offset is greater than the flip threshold AND it has not just flipped
    if(lastRawDiffx > 0) {        // check to see which direction the drive wheel was turning
      flipNumberx--;              // cw rotation 
    } else {                     // if(rawDiff < 0)
      flipNumberx++;              // ccw rotation
    }
    flippedx = true;            // set boolean so that the next time through the loop won't trigger a flip
  } else {                        // anytime no flip has occurred
    flippedx = false;
  }
   updatedPosx = rawPosx + flipNumberx*OFFSETx;

  updatedPosy = 130-(-0.148*(updatedPosy)+61.7);
  updatedPosx = 0.19*(updatedPosx) +70.3;
  Serial.print(updatedPosx);
  Serial.print(",");
  Serial.println(updatedPosy);

  delay(20);  // Short delay to avoid overwhelming the serial connection

  dxh = (double)(xh - lastxh) / 0.001; 

  // Calculate the filtered velocity of the handle using an infinite impulse response low pass filter
  dxh_filt = .9*dxh + 0.1*lastdxh; 
    
  // Record the previous positions and velocities
  lastLastxh = lastxh;
  lastxh = xh;
  
  lastLastdxh = lastdxh;
  lastdxh = dxh;

  dyh = (double)(yh - lastyh) / 0.001; 

  // Calculate the filtered velocity of the handle using an infinite impulse response low pass filter
  dyh_filt = .9*dyh + 0.1*lastdyh; 
    
  // Record the previous positions and velocities
  lastLastyh = lastyh;
  lastyh = yh;
  
  lastLastdyh = lastdyh;
  lastdyh = dyh;
}

// Function to trigger ball-hit feedback using decay formula
void triggerBallHitFeedback(int duration) {
  int F_0 = 255; // Maximum intensity
  float beta = 0.02; // Damping factor
  float omega = 10.0; // Frequency (rad/s)

  unsigned long startTime = millis();

  while (millis() - startTime < duration) {
    float t = (millis() - startTime) / 1000.0; // Convert time to seconds
    int feedbackForce = F_0 * exp(-beta * t) * cos(omega * t); // Calculate force

    // Ensure feedbackForce is within motor control limits (0 to 255)
    feedbackForce = constrain(feedbackForce, 0, 255);

    // Apply feedback force to motors
    analogWrite(motor1Pin1, feedbackForce);
    analogWrite(motor1Pin2, 0);
    analogWrite(motor2Pin1, feedbackForce);
    analogWrite(motor2Pin2, 0);
  }

  // Stop motors after feedback
  stopMotors();
}

// Function to trigger high-intensity feedback top wall
void triggerHighIntensityFeedbackYTop(int duration) {
  int intensity = 150; // Maximum intensity

  // Apply high-intensity feedback to both motors
  analogWrite(motor1Pin1, 0);
  analogWrite(motor1Pin2, 0);
  analogWrite(motor2Pin1, intensity);
  analogWrite(motor2Pin2, HIGH);

  delay(duration);

  // Stop feedback after the duration
  stopMotors();
}

// Function to trigger high-intensity feedback bottom wall
void triggerHighIntensityFeedbackYBottom(int duration) {
  int intensity = 150; // Maximum intensity

  // Apply high-intensity feedback to both motors
  analogWrite(motor1Pin1, 0);
  analogWrite(motor1Pin2, 0);
  analogWrite(motor2Pin1, intensity);
  analogWrite(motor2Pin2, LOW);

  delay(duration);

  // Stop feedback after the duration
  stopMotors();
}

// Function to trigger high-intensity feedback back wall
void triggerHighIntensityFeedbackXBack(int duration) {
  int intensity = 150; // Maximum intensity

  // Apply high-intensity feedback to both motors
  analogWrite(motor1Pin1, intensity);
  analogWrite(motor1Pin2, 0);
  analogWrite(motor2Pin1, 0);
  analogWrite(motor2Pin2, 0);

  delay(duration);

  // Stop feedback after the duration
  stopMotors();
}

// Function to trigger high-intensity feedback bottom wall
void triggerHighIntensityFeedbackXNet(int duration) {
  int intensity = 150; // Maximum intensity

  // Apply high-intensity feedback to both motors
  analogWrite(motor1Pin1, intensity);
  analogWrite(motor1Pin2, 0);
  analogWrite(motor2Pin1, 0);
  analogWrite(motor2Pin2, 0);

  delay(duration);

  // Stop feedback after the duration
  stopMotors();
}
// Function to generate continuous texture feedback
void generateTextureFeedback(unsigned long currentMillis) {
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    forcex = 0.001*dxh_filt*(sin(100*3.14159*xh));
    forcey = 0.001*dyh_filt*(sin(100*3.14159*yh));

    // Apply feedback to both motors
    analogWrite(motor1Pin1, forcex);
    analogWrite(motor1Pin2, 0);
    analogWrite(motor2Pin1, forcey);
    analogWrite(motor2Pin2, 0);
  }
  }

// Function to stop all motor feedback
void stopMotors() {
  analogWrite(motor1Pin1, 0);
  analogWrite(motor1Pin2, 0);
  analogWrite(motor2Pin1, 0);
  analogWrite(motor2Pin2, 0);
}