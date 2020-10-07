/*
  This program allows a toy car to avoid obstacles by 
  claculating obstacle distance using a distance sensor.
  The program will decide if the toy car turns left or
  right based on the average distance at each side. This
  program requires two libraries: 
  one to control the ultrasonic sensor ("SR04.h") and 
  another one to control the servo motor (Servo.h)
*/

#include "SR04.h" // library for ultrasonic sensor.
#include <Servo.h> // library for servo motor.

#define TRIG_PIN 12 //Pins for sensor.
#define ECHO_PIN 11 

SR04 sr04 = SR04(ECHO_PIN,TRIG_PIN);
long distance;
bool runProgram = true; 

float left_ave = 0; // Average distance when sensor points left.
float right_ave = 0; // Average distance when the sensor points right.

Servo myservo; // Variable to control servo.
int looking;

void setup() {
   
   #define RIGHT 3 // Pin 3 controls right motors.
   #define LEFT 5 // Pin 5 powers left motors.  
   #define LCONTROL 6 // Pin 6 cuts power to left motors.
 
   pinMode (RIGHT, OUTPUT); // These pins are outputs of power.  
   pinMode (LEFT, OUTPUT);
   pinMode (LCONTROL, OUTPUT);

   myservo.attach(9); // Pin 9 for servo.
   myservo.write(90); // Position for servo in degrees.
   Serial.begin(9600);
   delay(2500);
}

void loop() {
  
   int counter  = 0; // counter control for while loops. 
   
   looking = 90; // variable for servo position.
   
   distance = sr04.Distance(); // distance calculated by sensor. 
   Serial.print(distance);
   Serial.println("cm");
   delay(10);
  
    if (distance <= 40){ // if distance is less than or equal to 40 cm then:
    
      digitalWrite (RIGHT, LOW); //Lines 49-51 stop the toy car.
      digitalWrite (LEFT, LOW); 
      digitalWrite (LCONTROL, HIGH);
      delay(1000);

 //-------------------------------------------------------------------------------------------------------------
      while (counter < 5){ // looking to the left loop.
        
        looking += 13; // variable looking will change at a rate of 13 degrees.
        myservo.write(looking);
        
        distance = sr04.Distance();
        Serial.print(distance);
        Serial.println("cm");
        delay(100); // time servo will stay at the given angle.
        
        counter += 1;
        left_ave += distance; // Sum of distances collected.
      } //end of left looking loop.

      looking = 90; 
      myservo.write(looking);
      left_ave = left_ave / counter; // calculating the left average.
      delay(500);
      counter = 0;
//-------------------------------------------------------------------------------------------------------------- 
      while (counter < 5){ // looking to the right loop. Same as previous just servo is pointing to the right.

        looking -= 12; // changing the servo position by a rate of 12 degrees.
        myservo.write(looking);
         
        distance = sr04.Distance();
        Serial.print(distance);
        Serial.println("cm");
        delay(100);  

        counter += 1; 
        right_ave += distance; 
      } // end of right looking loop.  

      looking = 90; 
      myservo.write(looking);
      right_ave = right_ave / (counter);
      delay(500); 
//--------------------------------------------------------------------------------------------------------------

          if (right_ave > left_ave){ // If true then turn right.
            digitalWrite(LCONTROL, LOW);
            digitalWrite(LEFT, HIGH);
            delay (1000);// time turning right. *Improve (2)
            looking = 90;
          } 
          else if (left_ave > right_ave){ //if true then turn left.
            digitalWrite(RIGHT, HIGH);
            delay (1000);// time turning left. *Improve (2)
            looking = 90;  
          } 
          else if (left_ave == right_ave)
          {
             looking = 90;
             myservo.write(looking); 
             // *Improve (4): make the car randomly turn or do reverse.
          }
        
      }// end of if (distance <= 40).

    else { //when distance > 40 cm car moves forward. 
      digitalWrite (LEFT, HIGH);
      digitalWrite (LCONTROL, LOW);
      digitalWrite (RIGHT, HIGH);
      looking = 90;
      myservo.write(looking);
      int counter = 0; //counter set to 0 so loops can run again.
    } 
     
    
    /* 
      IMPROVE LIST: 
      
      1) Fix angular obstacles. --> We'll work on this. 

          To improve the detenction of angular obstacles, I plan to create another 'if' condition to 
          help detect such obstacles.It appears to be happening that everytime the soundwaves hit a 
          surface that relfects them in the wrong direction (left(angle)) or simply away from the sound 
          receiver, the distance detected by the sensor is generally greater than 2000 cm or 20 m, thing 
          that is exagerated but consisten. This wrong information makes the computer to take wrong 
          desicions while deciding where to turn, sometimes there is more space in one side than in the
          other but the car turns to the side where there is the least space beacuse the sound waves hit
          an angular obstacle and thus distance is mistakenly read as more than 2000 cm. The fact that this wrong
          reading by the sensor always detects a hughe value when there is an angular obstacle is helpful 
          because I could make a condition using 'if' that will detect when the distance is exagerated and 
          make the car to ignore that piece of data when making turning desicions. Also, I could even make the car
          turn* towards the angular obstacle to face it and aquratelly measure the dinstance. 

          1* To make the car turn towards an specific obstacle, I should be able to know what time it takes 
          the car to turn 1 degree. That way I could control the time neccessary for the vehicle to turn to
          certain angle based on the angle of the servo at the moment the sensor detects the desired obstacle to turn to.
          Also it would be very helpful to make the car to be able to do reverse, that will allow the effectiveness
          of the turning system of the vehicle.
      
      2) Fix turning time. --> We'll work on this as well.
      
          Fixing turning time is imprtant since it will allow the car to precisely avoid objects while performing
          a turn specially in tight spaces. The tighter the space (the closer the obstacles in both sides) then 
          the less time the car will turn. The greater the space, the more time the car will turn but with some
          restrictions because I don't want it to crash with another obstacle while turning. It would be helpful
          to add some code for the car to look around for obstacles as it turns. Also, it would help if the computer
          would know the specific place the car should turn (this, at a more advanced level).
          
      For later: 
      
      3) Fix deviation to the right.
      
      4) Make reverse. 
      
      5) Crash sensor. 
      
      6) Floor level sensor.
    */
}
