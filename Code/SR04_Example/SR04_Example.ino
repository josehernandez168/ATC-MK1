//www.elegoo.com
//2016.12.08
#include "SR04.h" 
#include <Servo.h>

#define TRIG_PIN 12
#define ECHO_PIN 11 

SR04 sr04 = SR04(ECHO_PIN,TRIG_PIN);
long distance; 
bool runProgram = true; 

int look_control = 0; //this variable makes sure that when the sensor points to one side and... 
//...there is no obstacle, the servo does'nt point to the other side unnecesarily. 

float left_ave = 0;
float right_ave = 0;

//int rand_look;

Servo myservo;
int looking = 20;

void setup() {
   
   #define RIGHT 3  
   #define LEFT 5 
   #define LCONTROL 6
 
   pinMode (RIGHT, OUTPUT);  
   pinMode (LEFT, OUTPUT);
   pinMode (LCONTROL, OUTPUT);

   myservo.attach(9);
   myservo.write(90);
   Serial.begin(9600);
   delay(2500);
}

void loop() { 
   int counter  = 0;
   //rand_look = random(1,3);// serlect a value from 1 to 3 excluding 3. if it's 1 then look left, if its 2 then look right.
   looking = 90;// angle to face sensor forward.
   distance = sr04.Distance();
   Serial.print(distance);
   Serial.println("cm");
   delay(10); 
  
    if (distance <= 40){ 
      digitalWrite (RIGHT, LOW);  // transistor (TIP 120) requires constant energy supply. 
      digitalWrite (LEFT, LOW); // stops charge input into musfet transistor.
      digitalWrite (LCONTROL, HIGH);//drain charge of musfet gate to close the gate.
      delay(1000);

 //-------------------------------------------------------------------------------------------------------------
      while (counter < 5){ // looking to the left loop. FIX this loop, looking its turning too...
                                                 //...often to the left instead of looking to the right. Also reglue servo in right position.
        looking += 13;
        myservo.write(looking);
        
        distance = sr04.Distance();// this line assigns the numerical value of distance from sensor to variable 'distance'.
        Serial.print(distance);
        Serial.println("cm");
        delay(100); // time servo will stay at the given angle.
        
        counter += 1;
        left_ave += distance;
      } //end of left looking while loop.

      looking = 90; 
      myservo.write(looking);
      left_ave = left_ave / counter;
      delay(500);
      counter = 0;
//-------------------------------------------------------------------------------------------------------------- 
      while (counter < 5){ // looking to the right loop.

        looking -= 12;
        myservo.write(looking);
         
        distance = sr04.Distance();// this line assigns the value of distance to variablr 'distance'.
        Serial.print(distance);
        Serial.println("cm");
        delay(100); // time servo will stay at the given angle. 

        counter += 1; 
        right_ave += distance; 
      } // end of while (counter < 10).  

      looking = 90; 
      myservo.write(looking);
      right_ave = right_ave / (counter);
      delay(500); 
//--------------------------------------------------------------------------------------------------------------

          if (right_ave > left_ave){
            digitalWrite(LCONTROL, LOW);
            digitalWrite(LEFT, HIGH);
            delay (1000);// time turning left.
            looking = 90;
          } 
          else if (left_ave > right_ave){
            digitalWrite(RIGHT, HIGH);
            delay (1000);// this is the time the vehicle will be turning left 
            looking = 90;  
          } 
          else if (left_ave == right_ave)
          {
             looking = 90;
             myservo.write(looking);  
          }
        
      }// end of if (distance <= 40).

      /*now what is left is to make it decide where to turn when
       * right and left distances are less than 40 cm. for that I
       * will make it compare the average distance from the left 
       * to the average distance collected from the right. The
       * greatest distance, that's where the car will turn by
       * shuting on the motors in the opposite side.
       * 
       * NOTE: to calculate the average distance I will collect all
       * distances by adding them (+=distance) within a variable X, 
       * then I will divide this variable by the number counter which
       * is the number of items collected for the values of distance.
      */
    else { 
      digitalWrite (LEFT, HIGH);
      digitalWrite (LCONTROL, LOW);
      digitalWrite (RIGHT, HIGH);
      looking = 90;
      myservo.write(looking);
      int counter = 0;
    } 
    // Error, car is only turning left.
}
