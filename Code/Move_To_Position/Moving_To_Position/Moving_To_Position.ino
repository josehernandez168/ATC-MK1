#include <MPU9250.h>

// Movement pins
#define FORWARD_R 14 
#define FORWARD_L 16
#define REVERSE_R 15
#define REVERSE_L 17
#define SPEED_R 6
#define SPEED_L 5

// Signaling LED
#define LED 12

// 9-Axis Sensor
MPU9250 IMU(Wire, 0x68); // an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68

//  STATE
float displacement[] = {0, 0};
float orientation[] = {0, 1}; // Car is assumed to always start facing positive Y direction

void setup() {
  Serial.begin(9600);

  pinMode(FORWARD_R, OUTPUT);
  pinMode(FORWARD_L, OUTPUT);
  pinMode(REVERSE_R, OUTPUT);
  pinMode(REVERSE_L, OUTPUT);
  pinMode(SPEED_R, OUTPUT);
  pinMode(SPEED_L, OUTPUT);

  pinMode(LED, OUTPUT);
  
  // start communication with IMU and attemp to calibrate biases
  // see https://github.com/bolderflight/MPU9250 for details
  int IMUstatus = IMU.begin();
  if (IMUstatus < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(IMUstatus);
    while(1) {} // TODO beep annoyingly
  }
  // Set accelerometer range
  IMUstatus = IMU.setAccelRange(MPU9250::ACCEL_RANGE_4G);
  if (IMUstatus < 0) {
    Serial.println("Failed to set acceleration range");
    Serial.println(IMUstatus);
    while(1) {} // TODO beep annoyingly
  }
  // Set gyroscope range
  IMUstatus = IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  if (IMUstatus < 0) {
    Serial.println("Failed to set gyroscope range");
    Serial.println(IMUstatus);
    while(1) {} // TODO beep annoyingly
  }

  // RUN ONCE

  float p1[] = {0, 100};
  float p2[] = {50, 100};
  float p3[] = {50, 0};
  float p4[] = {-50, 0};
  float p5[] = {-50, -100};
  float p6[] = {0, -100};
  float p7[] = {0, 0};

  goTo(p1);
  goTo(p2);
  goTo(p3);
  goTo(p4);
  goTo(p5);
  goTo(p6);
  goTo(p7);
}

void loop() {
}

// ---------- FUNCTIONS ---------- //

float randToDegree(float rads) {
  return rads * (180.0 / PI);
}

float angleBetween(float a[], float b[]) { // vector a and b must be 2D
  return acos( (dotProduct(a, b)) / (magnitude(a) * magnitude(b)) );
}

float dotProduct(float a[], float b[]) { // vector a and b must be 2D
  return a[0]*b[0] + a[1]*b[1];
}

float magnitude(float a[]) { // vector a must be 2D
  return sqrt(a[0]*a[0] + a[1]*a[1]);
}

// Returns Z component of cross product a X b, assuming z = 0 for both
// Use this to know shortest turn between to orientation vectors
float crossProductZ(float a[], float b[]) { // vector a and b must be 2D
  return a[0]*b[1] - a[1]*b[0];
}

// Rotates a vector counterclockwise for positive angle in radians
void rotateVector(float angle, float vector[]) {
  float inputVector[] = {vector[0], vector[1]};
  vector[0] = inputVector[0]*cos(angle) - inputVector[1]*sin(angle);
  vector[1] = inputVector[0]*sin(angle) + inputVector[1]*cos(angle);
}

void align(float alignToVector[]) { // vector must be 2D
  float turnSpeedRatio = 0.6;
  float radError = 0.001745; // radian difference between orientation and alignToAngle to stop motion at
  float correctSlidingRad = 0; //0.0872665;

  unsigned long lastTime;
  float radialVelocity;
  if (angleBetween(orientation, alignToVector) < radError) {
    return; // no turn required
  } else if (crossProductZ(orientation, alignToVector) > 0) {
    // turn left
    analogWrite(SPEED_R, 252 * turnSpeedRatio);
    analogWrite(SPEED_L, 255 * turnSpeedRatio);
    digitalWrite(FORWARD_R, HIGH);
    digitalWrite(REVERSE_L, HIGH);
  } else {
    // turn right
    analogWrite(SPEED_R, 252 * turnSpeedRatio);
    analogWrite(SPEED_L, 255 * turnSpeedRatio);
    digitalWrite(FORWARD_L, HIGH);
    digitalWrite(REVERSE_R, HIGH);
  }
  digitalWrite(LED, HIGH); 

  lastTime = micros();
  while (angleBetween(orientation, alignToVector) > radError + correctSlidingRad) {
    // read the sensor
    IMU.readSensor();
    
    unsigned long endTime = micros();
    unsigned long lapseTime = endTime - lastTime;
    lastTime = endTime;
    
    radialVelocity = -IMU.getGyroZ_rads();

    rotateVector(radialVelocity * (lapseTime / 1000000.0), orientation);
  } 

  stopMotion();
  // Ensure motion has stopped, while recording orientation updates.
  do {
    // read the sensor
    IMU.readSensor();
    
    unsigned long endTime = micros();
    unsigned long lapseTime = endTime - lastTime;
    lastTime = endTime;
    
    radialVelocity = -IMU.getGyroZ_rads();
    
    rotateVector(radialVelocity * (lapseTime / 1000000.0), orientation);
  } while (abs(radialVelocity) > 0.001);
  digitalWrite(LED, LOW);
} // end align

void stopMotion() {
  analogWrite(SPEED_R, 0);
  analogWrite(SPEED_L, 0);

  digitalWrite(FORWARD_R, LOW);
  digitalWrite(FORWARD_L, LOW);
  digitalWrite(REVERSE_R, LOW);
  digitalWrite(REVERSE_L, LOW);
}

// Normalize the referenceVector and update results to saveVector.
void normalize(float referenceVector[], float saveVector[]) { // Vectors must be 2D
  float refMagnitude = magnitude(referenceVector);
  saveVector[0] = referenceVector[0] / refMagnitude;
  saveVector[1] = referenceVector[1] / refMagnitude;
}

void forward(float dist) {
  // ms = 13.602*cm + 133.58
  // cm = 0.0734*ms - 9.5645
  // empirically calibrated and tested on multiple floor materials. May change with battery levels.
  
  float correctSliddingDist = 10; // cm, empirically calibrated
  
  if (dist == 0) { return; }
  
  analogWrite(SPEED_R, 252);
  analogWrite(SPEED_L, 255);
  digitalWrite(FORWARD_R, HIGH);
  digitalWrite(FORWARD_L, HIGH);
  
  digitalWrite(LED, HIGH);

  float normalizedOrientation[2];
  normalize(orientation, normalizedOrientation);

  unsigned long travelTime = (13.602*(dist - correctSliddingDist) + 133.58) * 1000;
  unsigned long lastTime = micros();
  do {
  } while (micros() < lastTime + travelTime);
  lastTime = micros();

  // Update position state
  displacement[0] += (dist) * normalizedOrientation[0];
  displacement[1] += (dist) * normalizedOrientation[1];

  stopMotion();
  delay(500);
  digitalWrite(LED, LOW);
} // end forward

// Used to tell car to go to specific location by a direct line from its current position to pos
void goTo(float pos[]) {
  float requiredMovement[] = {pos[0] - displacement[0], pos[1] - displacement[1]};
  float distance = magnitude(requiredMovement);

  align(requiredMovement);
  forward(distance);
}
