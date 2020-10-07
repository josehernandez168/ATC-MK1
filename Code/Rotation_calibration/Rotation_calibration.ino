#include <MPU9250.h>

#define FORWARD_R 14
#define FORWARD_L 16
//------------------------
#define REVERSE_R 15
#define REVERSE_L 17
//------------------------
#define SPEED_R 6
#define SPEED_L 5
//------------------------
#define TRIG_PIN 7
#define ECHO_PIN 8
//------------------------
#define VISION_SERVO 9
//------------------------
#define BUZZER 4
#define LED 12
  
  MPU9250 IMU(Wire, 0x68);
  int status;
  float calibration;
  
  void setup() {
    // put your setup code here, to run once:
  
    pinMode(FORWARD_R, OUTPUT);
    pinMode(FORWARD_L, OUTPUT);
    pinMode(REVERSE_R, OUTPUT);
    pinMode(REVERSE_L, OUTPUT);
    pinMode(SPEED_R, OUTPUT);
    pinMode(SPEED_L, OUTPUT);
    pinMode(BUZZER, OUTPUT);
    
    Serial.begin(9600);
    
    while(!Serial){digitalWrite(BUZZER, HIGH);}
    status = IMU.begin(); 

    if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
    } 

    IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
    
// moveForward (distance you want the car to move in cm)
// moveReverse (distance you want the car to move in cm)
// turnRight (angle in degrees, calibration in degrees)
// turnLeft (angle in degrees, calibration in degrees) 

//    moveForward(80, 10, 252, 255); // -1.5 for calibration if distance is 50
//    moveForward(70, -1.5, 50, 255);
//    moveForward(90, -1.5, 255, 50);
//    moveForward(80, 10, 252, 255); 

//    turnLeft(90, 85.2);
//    turnLeft(90, 85.2);
//    turnLeft(90, 85.2);
//    turnLeft(90, 85.2);

//    moveForward(100, 3.5, 252, 255);//move forward 10th floor
//    moveReverse(100, 3.5 + (3.5*0.176));//move backward 10th floor

//-----------------------USING AUTOMATIC CALIBRATION-----------------------------

  calibration = calibrate_left_turn(10, 90);
  digitalWrite(BUZZER, HIGH);
  delay(5000);
  digitalWrite(BUZZER, LOW);
  
    moveForward(100, 3.5, 252, 255);
    delay(500);
    turnLeft(90, calibration);
    delay(500);
    moveForward(100, 3.5, 252, 255);
    delay(500);
    turnLeft(90, calibration);
    delay(500);
    moveForward(100, 3.5, 252, 255);
    delay(500);
    turnLeft(90, calibration);
    delay(500);
    moveForward(100, 3.5, 252, 255);
    delay(500);
    turnLeft(90, calibration); 
    
  }
  
  void loop() {
    
  }

  void moveForward(float distance, float calibration, int rspeed, int lspeed)
  { 
    analogWrite(SPEED_R, rspeed);// Remember: 75 is the minimum so motors can run
    analogWrite(SPEED_L, lspeed);
    
    digitalWrite(FORWARD_R, HIGH);
    digitalWrite(FORWARD_L, HIGH);

    delay(13.88 * (distance + calibration)); 
    
    digitalWrite(FORWARD_R, LOW);
    digitalWrite(FORWARD_L, LOW); 

  } 

  void moveReverse(float distance, float calibration)
  {
    analogWrite(SPEED_R, 252);
    analogWrite(SPEED_L, 255);
    
    digitalWrite(REVERSE_R, HIGH);
    digitalWrite(REVERSE_L, HIGH);

    delay(13.88 * (distance + calibration)); 
    
    digitalWrite(REVERSE_R, LOW);
    digitalWrite(REVERSE_L, LOW); 

  }

  void turnRight (float angle, float calibration)
  { 
    analogWrite(SPEED_R, 200);
    analogWrite(SPEED_L, 200);
    
    digitalWrite(FORWARD_L, HIGH);
    digitalWrite(REVERSE_R, HIGH);

    delay(4.6535 * (angle + calibration) + 6.6495); 
    
    digitalWrite(FORWARD_L, LOW);
    digitalWrite(REVERSE_R, LOW); 

  }

  
  void turnLeft (float angle, float calibration)
  { 
    unsigned long start_time, end_time, lapse_time;
    float radial_velocity, end_angle = 0;
    
    analogWrite(SPEED_R, 255);
    analogWrite(SPEED_L, 255);
    digitalWrite(FORWARD_R, HIGH);
    digitalWrite(REVERSE_L, HIGH);

        start_time = micros();
        end_angle = 0;
        
        while(end_angle < (angle - calibration))
        {
          IMU.readSensor();
          
          end_time = micros();
          lapse_time = end_time - start_time;
          start_time = end_time;
    
          radial_velocity = - IMU.getGyroZ_rads();
          end_angle += (radial_velocity * (lapse_time / 1000000.0))*(180.0/PI);
          
          //Serial.print(end_angle, 6);
          //Serial.print("\n");
        }       
    digitalWrite(FORWARD_R, LOW);
    digitalWrite(REVERSE_L, LOW); 
  }  

  void halt()
  {
    analogWrite(SPEED_R, 0);
    analogWrite(SPEED_L, 0);
    digitalWrite(FORWARD_L, LOW);
    digitalWrite(FORWARD_R, LOW);
    digitalWrite(REVERSE_L, LOW);
    digitalWrite(REVERSE_R, LOW);
  } 

  float calibrate_left_turn (int presicion, float angle)
  { 
    int activator;
    unsigned long start_time, end_time, lapse_time;
    float radial_velocity, end_angle = 0, average_calibration;
    float turn_to_angle = angle, slidding = 0, centinel, calibration = 0;

    for (int i = 0; i < presicion; i++)
    {
        analogWrite(SPEED_R, 255);
        analogWrite(SPEED_L, 255);
        digitalWrite(FORWARD_R, HIGH);
        digitalWrite(REVERSE_L, HIGH);
    
        start_time = micros();
        end_angle = 0;
        slidding = 0;
        
        while(end_angle < (turn_to_angle - calibration) and activator == 0)
        {
          IMU.readSensor();
          
          end_time = micros();
          lapse_time = end_time - start_time;
          start_time = end_time;
    
          radial_velocity = - IMU.getGyroZ_rads();
          end_angle += (radial_velocity * (lapse_time / 1000000.0))*(180.0/PI);
          
          //Prints are for testing purposes
          //Serial.print(end_angle, 6);
          //Serial.print("\n"); 
    
          if (end_angle >= turn_to_angle - calibration)
          {
            halt();
            //Prints are for testing purposes
            //Serial.print(IMU.getGyroZ_rads());
            //Serial.print("_____________________");
            //Serial.print("\n");
            centinel = IMU.getGyroZ_rads();
            
            while(centinel < -0.001 or centinel > 0.001)
            { 
              IMU.readSensor();
              //Prints are for testing purposes
              //Serial.print(IMU.getGyroZ_rads());
              //Serial.print("*******************");
              //Serial.print("\n");
              digitalWrite(BUZZER, HIGH);
              
              end_time = micros();
              lapse_time = end_time - start_time;
              start_time = end_time;
              activator++;
              
              radial_velocity = - IMU.getGyroZ_rads();
              slidding += ((radial_velocity * (lapse_time / 1000000.0))*(180.0/PI));//check this calculation sliding is not being corrected
              calibration = slidding;
              
              Serial.print(calibration);
              Serial.print(" This is calibration\n");
              
              centinel = IMU.getGyroZ_rads();
            }
            activator = 1;
          }
        }
    
        activator = 0;
        Serial.print(turn_to_angle);
        Serial.print("This is turn_to_angle\n");
        
        digitalWrite(LED, HIGH);
        digitalWrite(BUZZER, LOW);
        Serial.print("***********Vicotory!**********");
        Serial.print("\n");
        delay(500);
        digitalWrite(LED, LOW);

        average_calibration += calibration;
    }

    return average_calibration / presicion;
  }
