#include <Servo.h>

// PIN DEFINITIONS

#define FORWARD_R 14 
#define FORWARD_L 16
#define REVERSE_R 15
#define REVERSE_L 17
#define SPEED_R 6
#define SPEED_L 5
#define TRIG_PIN 7
#define ECHO_PIN 8
#define VISION_SERVO 9
#define BUZZER 4
#define LED 12

// STATE VARIABLES

double stopTime = 0;

// Sonic Vision
Servo visionServo;

// BEHAVIOR FUNCTIONS

void moveVision(double degree) {
  degree = 0.8556*degree + 17; // Corrects vision angle based on calibration
  visionServo.write(degree);
}

void setMovementSpeed(int speed) {
  analogWrite(SPEED_R, (int) (0.988235294 * speed)); // based on calibration
  analogWrite(SPEED_L, speed);
}

void startMovement(int direction) {
  if (direction >= 0) {
    digitalWrite(FORWARD_R, HIGH);
    digitalWrite(FORWARD_L, HIGH);
  } else {
    digitalWrite(REVERSE_R, HIGH);
    digitalWrite(REVERSE_L, HIGH);
  }
}

void stopMovement() {
  digitalWrite(FORWARD_R, LOW);
  digitalWrite(FORWARD_L, LOW);
  digitalWrite(REVERSE_R, LOW);
  digitalWrite(REVERSE_L, LOW);
}

long getMovementMS(double cm) {
  cm -= 10.05416667; // break distance at 255 speed based on calibration
  return 13.602*cm + 133.58; // based on calibration
}

void setup() {
  Serial.begin(9600);

  pinMode(FORWARD_R, OUTPUT);
  pinMode(FORWARD_L, OUTPUT);
  pinMode(REVERSE_R, OUTPUT);
  pinMode(REVERSE_L, OUTPUT);
  pinMode(SPEED_R, OUTPUT);
  pinMode(SPEED_L, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(VISION_SERVO, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(LED, OUTPUT);

  visionServo.attach(VISION_SERVO);
  moveVision(90);
}

double moveCM = 254;
void loop() {
  
  // Initialize
  if (stopTime == 0) {
    stopTime = millis() + getMovementMS(moveCM);

    setMovementSpeed(255);
    startMovement(-1);
    
    digitalWrite(LED, HIGH);
  }

  // Stop motion
  if (millis() >= stopTime) {
    stopMovement();

    digitalWrite(LED, LOW);
  }
}
