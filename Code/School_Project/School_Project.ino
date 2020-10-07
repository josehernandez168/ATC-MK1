
//School project obstacle avoidance based avrage distance decision.
#include "SR04.h"
#include <Servo.h>
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

  MPU9250 IMU(Wire, 0x68);
  int status;
  
  SR04 sr04 = SR04 (ECHO_PIN, TRIG_PIN);
  long distance; 
  bool runProgram = true; 

  float average_right_distance,
        average_left_distance;
//------------------------

  Servo looking_angle;
  int angle;

//------------------------
#define BUZZER 4
#define LED 12 

// moveForward (distance you want the car to move in cm)
// moveReverse (distance you want the car to move in cm)
// turnRight (angle in degrees, calibration in degrees)
// turnLeft (angle in degrees, calibration in degrees)

void moveForward(int spe_ed);
void moveReverse(void);
void turnRight (float angle, float calibration);
void turnLeft (float angle, float calibration);
void halt (void);
float look_left (int data_points, float *turn_angle);
float look_right (int data_points, float *turn_angle);
void turn_left (float angle);
void turn_right (float angle); 

 float velocity, 
       start_distance = 0, 
       last_distance = 0,
       calculation_interval = 2,
       acceleration; 
          
 unsigned long time_trigger,
               lapse_time,  
               start_time, end_time,
               time_activator;


void setup() {
  // put your setup code here, to run once:

   pinMode(FORWARD_R, OUTPUT);
   pinMode(FORWARD_L, OUTPUT);
   pinMode(REVERSE_R, OUTPUT);
   pinMode(REVERSE_L, OUTPUT);
   pinMode(SPEED_R, OUTPUT);
   pinMode(SPEED_L, OUTPUT); 

   pinMode(LED, OUTPUT);
   pinMode(BUZZER, OUTPUT);

   angle = 93; 
   looking_angle.attach(9);
   looking_angle.write(angle);

   Serial.begin(9600);//display distance in port 9600 on the Serial Monitor.

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
    IMU.setAccelRange (MPU9250:: ACCEL_RANGE_4G);
    IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
    IMU.setSrd(19);
                  
   delay(3000);
   start_time = micros();
}

void loop() { 

   analogWrite(SPEED_R, 255);
   analogWrite(SPEED_L, 255);
   digitalWrite(FORWARD_R, HIGH);
   digitalWrite(FORWARD_L, HIGH);
   
   IMU.readSensor();

   acceleration = -(IMU.getAccelX_mss()+ 0.09856586);
   end_time = micros();
   lapse_time = end_time - start_time;
   start_time = end_time; 
   velocity += ((acceleration * ((lapse_time / 1000000.0))) * 100) + 0.20360516; 

   Serial.print("Acceleration is --> ");
   Serial.print(acceleration, 6);
   Serial.print("\n");
   
//   Serial.print("Velocity is --> ");
//   Serial.print(velocity, 6);
//   Serial.print("\n");
//   
   int activator = 0;
   float turn_angle = 0;

  distance = sr04.Distance();
  //Serial.print(distance);
  //Serial.println("cm"); 

  angle = 93;
  looking_angle.write(angle);
  digitalWrite(LED, LOW);
   
//Put a delay here if you want to analyze data from the Ultasonic sensor 
 
  if (distance >= 500)
  {
    while(distance >= 500)
    {
      turn_right(5);
      distance = sr04.Distance();
        //Serial.print(distance);
        //Serial.println("cm");
    }
  }
  if (distance <= 60)
  { 
    
    moveForward(80);
    
    digitalWrite(BUZZER, HIGH);
    digitalWrite(LED, HIGH);
    
    average_right_distance = look_right(10, &turn_angle);
    average_left_distance = look_left(10, &turn_angle);
    
    digitalWrite(BUZZER,LOW);
    
    if (average_right_distance > average_left_distance)
    { 
      halt();
      turn_right(45);
    } 

    if (average_left_distance > average_right_distance)
    { 
      halt();
      turn_left(45); 
    } 
  }

//  time_activator = end_time / 1000;
//  if ((velocity >= -5 and velocity <= 5) and (time_activator > 5000))
//  { 
//    halt();
//    velocity = 0;
//    moveReverse();
//    delay(500);
//    Serial.print("STUCK! DOING REVERSE --> ");
//    Serial.print(time_activator);
//    Serial.print("\n");
//    halt();
//  } 
// 
//  if (velocity >= 70 or velocity <= -70)
//  {
//    velocity = 6;
//  }

}

//--------------Function_Library----------------

void moveForward(int spe_ed)
  { 
    analogWrite(SPEED_R, spe_ed);
    analogWrite(SPEED_L, spe_ed);
    
    digitalWrite(FORWARD_R, HIGH);
    digitalWrite(FORWARD_L, HIGH);

    //delay(13.88 * distance);delay for mc/ cm from car's speed 
  }
   

 void moveReverse(void)
  {
    analogWrite(SPEED_R, 252);
    analogWrite(SPEED_L, 255);
    
    digitalWrite(REVERSE_R, HIGH);
    digitalWrite(REVERSE_L, HIGH);

    //delay(13.88 * distance);delay for mc/ cm from car's speed 
  }


  void halt (void)
  {
    analogWrite(SPEED_R, 0);
    analogWrite(SPEED_L, 0);
    digitalWrite(FORWARD_R, LOW);
    digitalWrite(REVERSE_L, LOW);
    digitalWrite(FORWARD_L, LOW);
    digitalWrite(REVERSE_R, LOW);
  }


 float look_right (int data_points, float *turn_angle)
 {  
    int i;
    float average_right_distance = 0, 
          total_distance = 0, distance, max_distance = 0;
    
    for (i = 0; i < data_points; i++)
    {
       angle -= (80 / data_points);
       looking_angle.write(angle);
       total_distance += sr04.Distance();
       distance = sr04.Distance();

       if (distance > max_distance and distance < 2100)
       {
          max_distance = distance;
          *turn_angle = angle;
       }

       //Serial.print(total_distance);
       //Serial.println(" cm  (10 times increasing distance***)");  
    } 

    //Serial.print(total_distance);
    //Serial.println(" cm  (total_distance_right)"); 

    average_right_distance = total_distance / (float) data_points;

    //Serial.print(average_right_distance);
    //Serial.println(" cm  (average_right_distance)");

    angle = 93;
    looking_angle.write(angle);
    
    return average_right_distance;
 } 


 float look_left (int data_points, float *turn_angle)
 {  
    int i;
    float average_left_distance = 0, 
          total_distance = 0, distance, max_distance = 0;
    
    for (i = 0; i < data_points; i++)
    {   
       angle += (80 / data_points);
       looking_angle.write(angle);
       total_distance += sr04.Distance();
       distance = sr04.Distance();

       if (distance > max_distance and distance < 2100)
       {
          max_distance = distance;
          *turn_angle = angle;
       }

       //Serial.print(total_distance);
       //Serial.println(" cm  (10 times increasing distance***)");
       
    } 

    //Serial.print(total_distance);
    //Serial.println(" cm  (total_distance_left)"); 

    average_left_distance = total_distance / (float) data_points;

    //Serial.print(average_left_distance);
    //Serial.println(" cm  (average_left_distance)");

    angle = 93;
    looking_angle.write(angle);
    
    return average_left_distance;
 } 


 void turn_left (float angle)
 { 
    int activator = 0;
    unsigned long start_time, end_time, lapse_time;
    float radial_velocity, end_angle = 0, average_calibration;
    float turn_to_angle = angle;
    
        analogWrite(SPEED_R, 255);
        analogWrite(SPEED_L, 255);
        digitalWrite(FORWARD_R, HIGH);
        digitalWrite(REVERSE_L, HIGH);
    
        start_time = micros();
        
        while(end_angle < (turn_to_angle) and activator == 0)
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
    
          if (end_angle >= turn_to_angle)
          { 
            halt();
            activator = 1;
          }
        }
        
        digitalWrite(LED, HIGH);
        digitalWrite(BUZZER, LOW);
        delay(250);
        digitalWrite(LED, LOW);
  }


 void turn_right (float angle)
 { 
    int activator = 0;
    unsigned long start_time, end_time, lapse_time;
    float radial_velocity, end_angle = 0, average_calibration;
    float turn_to_angle = - angle;
    
        analogWrite(SPEED_R, 255);
        analogWrite(SPEED_L, 255);
        digitalWrite(FORWARD_L, HIGH);
        digitalWrite(REVERSE_R, HIGH);
    
        start_time = micros();
        
        while(end_angle > (turn_to_angle) and activator == 0)
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
    
          if (end_angle <= turn_to_angle)
          { 
            halt();
            activator = 1;
          }
        }
        
        digitalWrite(LED, HIGH);
        digitalWrite(BUZZER, LOW);
        delay(250);
        digitalWrite(LED, LOW);
 }
