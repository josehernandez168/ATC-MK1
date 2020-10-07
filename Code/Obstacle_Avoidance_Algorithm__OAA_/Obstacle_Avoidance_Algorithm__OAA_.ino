#include <MPU9250.h>

//Obstacle Avoidance Algorithm using distance from an Ultrasonic sensor

#include "SR04.h"
#include <Servo.h>


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

void moveForward(void);
void moveReverse(void);
void turnRight (float angle, float calibration);
void turnLeft (float angle, float calibration);
void halt (void);
float look_right (int data_points); 
float look_left (int data_points); 

 float velocity, 
       start_distance = 0, 
       last_distance = 0,
       calculation_interval = 2;
          
 unsigned long time_trigger,
               last_time = 0,  
               start_time;


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
                  
   delay(3000);
}

void loop() {
  // put your main code here, to run repeatedly:

  time_trigger = millis()/ 1000.0;
  int activator = 0;

  distance = sr04.Distance();
  Serial.print(distance);
  Serial.println("cm"); 

  angle = 93;
  looking_angle.write(angle);
  digitalWrite(LED, LOW);
  moveForward();
   
//Put a delay here if you want to analyze data from the Ultasonic sensor
  
  if (distance <= 50)
  {
    halt();
    
    digitalWrite(BUZZER, HIGH);
    digitalWrite(LED, HIGH);
    
    average_right_distance = look_right(10);
    average_left_distance = look_left(10);
    
    start_distance = 0;
    activator = 0;
    
    digitalWrite(BUZZER,LOW);
    
    if (average_right_distance > average_left_distance)
    {
      turnRight(90, 6.5);
    } 

    else if (average_left_distance > average_right_distance)
    {
      turnLeft(90, 12);
    } 
  
  }

  if (start_distance == 0)
  {
    start_distance = distance; 
  }
  if ((time_trigger + (- calculation_interval - last_time)) >= 0 and start_distance > 0)
  { 
    Serial.print(last_time);
    Serial.println("THIS IS LAST TIME");
    
    last_distance = distance;
    velocity = (last_distance - start_distance) / calculation_interval; // calculating velocity
    activator = 1;
    
    Serial.print(velocity);
    Serial.println("cm per second");
    
    last_time = millis()/ 1000.0;
    start_distance = 0;
     
    Serial.print(last_time);
    Serial.println("THIS IS LAST TIME"); 

        if ((velocity >= - 10 and velocity <= 10) and 
           (activator == 1))
        { 
          activator = 0;
          
          halt();
          
          Serial.println("***MOVING REVERSE!***");//take out for performance
          moveReverse();
          digitalWrite(LED, HIGH);
          delay(1000);
          halt();
          
//          average_right_distance = look_right(10);
//          average_left_distance = look_left(10);
//          
//          if (average_right_distance > average_left_distance)
//          {
//            turnRight(90, 6.5);
//          } 
//      
//          else if (average_left_distance > average_right_distance)
//          {
//            turnLeft(90, 12);
//          } 
        } 
  }

} 

//-----------------------------Function Library-----------------------------

 void moveForward(void)
  { 
    analogWrite(SPEED_R, 252);
    analogWrite(SPEED_L, 255);
    
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


 void turnRight (float angle, float calibration)
  { 
    analogWrite(SPEED_R, 255);
    analogWrite(SPEED_L, 255);
    
    digitalWrite(FORWARD_L, HIGH);
    digitalWrite(REVERSE_R, HIGH);

    delay(4.6535 * (angle + calibration) + 6.6495); //formula for time (mc) per given angle.

    digitalWrite(FORWARD_L, LOW);
    digitalWrite(REVERSE_R, LOW);

  }

  
 void turnLeft (float angle, float calibration)
  {  
    analogWrite(SPEED_R, 255);
    analogWrite(SPEED_L, 255); 
    
    digitalWrite(FORWARD_R, HIGH);
    digitalWrite(REVERSE_L, HIGH);

    delay(4.6535 * (angle + calibration) + 6.6495); //formula for time (mc) per given angle.

    digitalWrite(FORWARD_R, LOW);
    digitalWrite(REVERSE_L, LOW);

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


 float look_right (int data_points)
 {  
    int i;
    float average_right_distance = 0, 
          total_distance = 0;
    
    for (i = 0; i < data_points; i++)
    {
       angle -= (80 / data_points);
       looking_angle.write(angle);
       total_distance += sr04.Distance();

       Serial.print(total_distance);
       Serial.println(" cm  (10 times increasing distance***)");
       
    } 

    Serial.print(total_distance);
    Serial.println(" cm  (total_distance_right)"); 

    average_right_distance = total_distance / (float) data_points;

    Serial.print(average_right_distance);
    Serial.println(" cm  (average_right_distance)");

    angle = 93;
    looking_angle.write(angle);
    
    return average_right_distance;
 } 


 float look_left (int data_points)
 {  
    int i;
    float average_left_distance = 0, 
          total_distance = 0;
    
    for (i = 0; i < data_points; i++)
    {
       angle += (80 / data_points);
       looking_angle.write(angle);
       total_distance += sr04.Distance();

       Serial.print(total_distance);
       Serial.println(" cm  (10 times increasing distance***)");
       
    } 

    Serial.print(total_distance);
    Serial.println(" cm  (total_distance_left)"); 

    average_left_distance = total_distance / (float) data_points;

    Serial.print(average_left_distance);
    Serial.println(" cm  (average_left_distance)");

    angle = 90;
    looking_angle.write(angle);
    
    return average_left_distance;
 }


 
//TO DO LIST 
/*
  1) Validate the average distance from each side. Total distance collected often bigger than it should
     add condition that will reposition the car and retake measurements again if this very big data is gathered.

  2) Make a new condition using speed of the car to let the computer know if the car got stuck. If the condition is 
     true, then the car will move reverse and recalculate. To calculate speed I will use 4 variables. Two to store 
     position 1 and 2 and the other two too calculate time 1 and 2 as the car moves, and add the average velocity.
     Posible errors may include a sudden change in velocity from a regularly small value to a big positive or a
     small negative number. If this happens then I could make a condition that will filter this data or reduce its
     size or even simply make the car slowly move in the direction from which the data was collected and explore
     the area with caution. If it crashes or gets stuch with an obstacle, it can do reverse and recheck for distance
     values.

  3) Velocity is being assigned to zero, it results in the condition of going reverse always being true, thus the car
     is never moving forward. So, I need to check on the current conditions that activate the reverse of the speed is 
     between -5 and 5. I need a condition that once the car moves backward once and turns it will not let that happen
     again unless enough data for an updated velocity measurement is achieved.
 */
 
