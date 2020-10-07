// LIBRARIES

#include "SR04.h" // library for ultrasonic sensor.
#include <Servo.h> // library for servo motor.
#include <MPU9250.h>

// PIN DEFINITIONS

// Movement pins
#define FORWARD_R 14 
#define FORWARD_L 16
#define REVERSE_R 15
#define REVERSE_L 17
#define SPEED_R 6
#define SPEED_L 5
// Vision pins
#define TRIG_PIN 7
#define ECHO_PIN 8
#define VISION_SERVO 9
#define BUZZER 4
#define LED 12

// CALIBRATION VARIABLES

// Values empirically calibrated to maintain a straight line
int RIGHT_MOTOR_MAX_SPEED = 252;
int LEFT_MOTOR_MAX_SPEED = 255;
int TIME_PER_CM = 14.25741; //in ms/cm

// STATE VARIABLES

boolean USE_SERIAL = true;
double posX = 0; // cm
double posY = 0; // cm
double speedX = 0; // cm/s
double speedY = 0; // cm/s
long stopMovementTime = 0;
long movementStage = 1;

// Sonic Vision
SR04 sonicVision = SR04(ECHO_PIN, TRIG_PIN);
Servo visionServo;

// 9-Axis Sensor
// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire,0x68);
double ACC_OFFSET_X;
double ACC_OFFSET_Y;
double ACC_OFFSET_Z;


// BEHAVIOR FUNCTIONS

void moveVision(double degree) {
  degree = 0.8556*degree + 17; // Corrects vision angle based on calibration
  visionServo.write(degree);
}

void stopMovement() {
  digitalWrite(FORWARD_R, LOW);
  digitalWrite(FORWARD_L, LOW);
  digitalWrite(REVERSE_R, LOW);
  digitalWrite(REVERSE_L, LOW);
}

void setHighSpeed() {
  analogWrite(SPEED_R, RIGHT_MOTOR_MAX_SPEED); 
  analogWrite(SPEED_L, LEFT_MOTOR_MAX_SPEED);
}

// distance in centimeters
// Start moving forward and return time in future when movement should stop
long moveForward(double distance) {
  setHighSpeed();
  
  digitalWrite(FORWARD_R, HIGH);
  digitalWrite(FORWARD_L, HIGH);

  long time = TIME_PER_CM*distance;
  return time + millis();
}

// TODO calibrate going backwards

// TODO FIX RIGHT AND LEFT TURNS

//void turnRight(double degree) {
//  delay(500);
//  stopMovement();
//  setHighSpeed();
//  
//  //double time = 4.6271*degree + 12.471; first try
//  double time = 4.6535 * degree + 6.6495;
//  
//  //-----------------TURN RIGHT-----------------
//  digitalWrite(FORWARD_L, HIGH);
//  digitalWrite(REVERSE_R, HIGH);
//  
//  delay(time);
//  stopMovement();
//}
//
//void turnLeft(double degree) { // TODO recalibrate left turn
//  delay(500);
//  stopMovement();
//  setHighSpeed();
//
//  //double time = 4.4174*degree + 20.763; actual
//  //double time = 4.5211*degree + 16.495; second try
//  //double time = 4.348 * degree + 39.387; third try
//  double time = 4.3897 * degree + 30.489;
//  
//  
//  //-----------------TURN LEFT-----------------
//  digitalWrite(FORWARD_R, HIGH);
//  digitalWrite(REVERSE_L, HIGH);
//
//  delay(time);
//  stopMovement();
//}

void setup() {
  if (USE_SERIAL) {
    Serial.begin(9600);
    while(!Serial) {}
  }
  
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
  
  stopMovement();

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
  // Set Digital Low Pass Filter (DLPF) bandwidth in Hz, the lower the number, the more noise reduction. Note loop() runs at best at 87Hz.
  IMUstatus = IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_41HZ);
  if (IMUstatus < 0) {
    Serial.println("Failed to set Digital Low Pass Filter (DLPF) bandwidth");
    Serial.println(IMUstatus);
    while(1) {} // TODO beep annoyingly
  }
  IMUstatus = IMU.setSrd(9);
  if (IMUstatus < 0) {
    Serial.println("Failed to set data sample rate");
    Serial.println(IMUstatus);
    while(1) {} // TODO beep annoyingly
  }

  IMUstatus = IMU.calibrateAccel();
  double x = 0; double y = 0; double z = 0;
  for (int i = 0; i < 100; i++) {
    // read the sensor
    IMU.readSensor();
    x += IMU.getAccelX_mss();
    y += IMU.getAccelY_mss();
    z += IMU.getAccelZ_mss();
    delay(20);
  }
  ACC_OFFSET_X = x / 100;
  ACC_OFFSET_Y = y / 100;
  ACC_OFFSET_Z = z / 100;
}

//long count = 0;
//long startTime = millis();

int decelerateCounter = 0;
double lastTime;
void loop() { // Around 87 runs per second on timing only load, i.e. all movements have > 11 ms resolution

  double stopDistance = 100; // cm
  if (movementStage == 1) {
    // Start moving forward at high speed
    analogWrite(SPEED_R, (int) RIGHT_MOTOR_MAX_SPEED);
    analogWrite(SPEED_L, (int) LEFT_MOTOR_MAX_SPEED);

    digitalWrite(FORWARD_R, HIGH);
    digitalWrite(FORWARD_L, HIGH);

    movementStage += 1;
    lastTime = ((double) millis()) / 1000;

    digitalWrite(LED, HIGH);
  } else {
    // Find delta time since last loop
    double currentTime = ((double) millis()) / 1000;
    double deltaT = currentTime - lastTime;
    lastTime = currentTime;

    // Read accelerometer sensor data
    IMU.readSensor();
    double accX = (IMU.getAccelX_mss() - ACC_OFFSET_X) * 100; // cm/s^2
    double accY = (IMU.getAccelY_mss() - ACC_OFFSET_Y) * 100; // cm/s^2

    // Calculate new speed state
    double deltaSpeedX = accX * deltaT;
    double deltaSpeedY = accY * deltaT;
    speedX += deltaSpeedX;
    speedY += deltaSpeedY;

    // Calculate new position state
    double deltaPosX = speedX * deltaT;
    double deltaPosY = speedY * deltaT;
    posX += deltaPosX;
    posY += deltaPosY;

    // Check if destination has been reached
    double totalDisplacement = sqrt(posX*posX + posY*posY);
    if (totalDisplacement >= stopDistance) {
      stopMovement();

      digitalWrite(LED, LOW);
    }

//    // Accelerate in first 50 cm linearly from 75% PWM to 100% PWM
//    if (totalDisplacement <= 50) {
//      double pwm = 0.005*totalDisplacement + 0.75;
//      analogWrite(SPEED_R, (int) (RIGHT_MOTOR_MAX_SPEED * pwm));
//      analogWrite(SPEED_L, (int) (LEFT_MOTOR_MAX_SPEED * pwm));
//    }

    // Decelerate in last 50 cm linearly from 100% PWM to 5%PWM, PWM% = 0.019*remainingDist + 0.05;
//    double remainingDist = stopDistance - totalDisplacement;
//    if (remainingDist < 50) {
//      double pwm = 0.019*remainingDist + 0.05;
//      analogWrite(SPEED_R, (int) (RIGHT_MOTOR_MAX_SPEED * pwm));
//      analogWrite(SPEED_L, (int) (LEFT_MOTOR_MAX_SPEED * pwm));
//    }


//    Serial.println(remainingDist);
//    if (remainingDist < 50 && decelerateCounter == 0) {
//      decelerateCounter = 1;
//    }
//    if (decelerateCounter > 0) {
//      decelerateCounter += 1;
//      double pwm = -0.00857*decelerateCounter + 1;
//      if (pwm < 0.4) {
//        pwm = 0.4;
//      }
//      analogWrite(SPEED_R, (int) (RIGHT_MOTOR_MAX_SPEED * pwm));
//      analogWrite(SPEED_L, (int) (LEFT_MOTOR_MAX_SPEED * pwm));
//    }  
//  }

//  if (USE_SERIAL) {
//    Serial.print("POS: (");
//    Serial.print(posX);
//    Serial.print(", ");
//    Serial.print(posY);
//    Serial.println(")");
//  }
//
//  if (USE_SERIAL) {
//    Serial.print("SPEED: (");
//    Serial.print(speedX);
//    Serial.print(", ");
//    Serial.print(speedY);
//    Serial.println(")");
//  }


//  if (millis() > stopMovementTime) {
//    stopMovement();
//  }
//
//  if (stopMovementTime == 1) {
//    stopMovementTime = moveForward(100);
//    movementStage += 1;
//  }


//  // read the sensor
  IMU.readSensor();
  Serial.println(IMU.getAccelY_mss(), 6);
  // display the data
//  Serial.print("AccelX: ");
//  Serial.print(IMU.getAccelX_mss() - ACC_OFFSET_X,6);
//  Serial.print("  ");
//  Serial.print("AccelY: ");  
//  Serial.print(IMU.getAccelY_mss() - ACC_OFFSET_Y,6);
//  Serial.print("  ");
//  Serial.print("AccelZ: ");  
//  Serial.println(IMU.getAccelZ_mss() - ACC_OFFSET_Z,6);
  
//  Serial.print("GyroX: ");
//  Serial.print(IMU.getGyroX_rads(),6);
//  Serial.print("  ");
//  Serial.print("GyroY: ");  
//  Serial.print(IMU.getGyroY_rads(),6);
//  Serial.print("  ");
//  Serial.print("GyroZ: ");  
//  Serial.println(IMU.getGyroZ_rads(),6);

//  Serial.print("MagX: ");  
//  Serial.print(IMU.getMagX_uT(),6);
//  Serial.print("  ");  
//  Serial.print("MagY: ");
//  Serial.print(IMU.getMagY_uT(),6);
//  Serial.print("  ");
//  Serial.print("MagZ: ");  
//  Serial.println(IMU.getMagZ_uT(),6);
  
//  Serial.print("Temperature in C: ");
//  Serial.println(IMU.getTemperature_C(),6);
//  Serial.println();
//  delay(500);
  
//  count += 1;
//  //Serial.println(((double) (millis() - startTime)) / 1000);
//  if (USE_SERIAL) {
//    Serial.print(count / (((double) (millis() - startTime)) / 1000));
//    Serial.println(" FPS");
//  }
  
//  double visionAverageNum = 5;
//  double distance;
//
//  // The following code calculated distance once per loop correcting for calibration
//  // Reported value is from front bumper of car
//  distance = 0;
//  for (int i = 0; i < visionAverageNum; i++) {
//    distance += sonicVision.Distance();
//    delay(5);
//  }
//  distance /= visionAverageNum;
//  distance = (distance - 1.9548) / 0.9542;
//
//  // TODO logging, remove in final version to lower CPU usage, i.e. delay
//  Serial.print(distance);
//  Serial.println("cm");

//  // Test vision servo
//  moveVision(90);
//  delay(3000);
//  moveVision(0);
//  delay(3000);
//  moveVision(180);
//  delay(3000);

  
//  // put your main code here, to run repeatedly:
//  
//  //--------------SETTING SPEED----------------
//  analogWrite(SPEED_R, 255); 
//  analogWrite(SPEED_L, 255);
//  
//  //---------------FORWARD---------------------
//  digitalWrite(FORWARD_R, HIGH);//HIGH = 255 in analogWrite. 
//  digitalWrite(FORWARD_L, HIGH);
//  delay(1000);//TIME MOVING FORWARD
//  digitalWrite(FORWARD_R, LOW);//LOW = 0 in analogWrite.
//  digitalWrite(FORWARD_L, LOW);
//  delay(1000);// half second
//  
//  //---------------REVERSE----------------------
//  digitalWrite(REVERSE_R, HIGH);
//  digitalWrite(REVERSE_L, HIGH);
//  delay(1000);//TIME DOING REVERSE
//  digitalWrite(REVERSE_R, LOW);
//  digitalWrite(REVERSE_L, LOW);
//  delay(1000);
//  
//  //-----------------TURN RIGHT-----------------
//  digitalWrite(FORWARD_R, HIGH);
//  digitalWrite(REVERSE_L, HIGH);
//  delay(1000);//TIME TURNING
//  digitalWrite(FORWARD_R, LOW);
//  digitalWrite(REVERSE_L, LOW);
//  delay(1000);
//  
//  //-----------------TURN LEFT-------------------
//  digitalWrite(FORWARD_L, HIGH);
//  digitalWrite(REVERSE_R, HIGH);
//  delay(1000); //TIME TURNING
//  digitalWrite(FORWARD_L, LOW);
//  digitalWrite(REVERSE_R, LOW);
//  delay(1000);
}
