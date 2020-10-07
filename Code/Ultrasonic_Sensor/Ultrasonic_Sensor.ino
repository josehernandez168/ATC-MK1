
#include "SR04.h"


#define TRIG_PIN 7
#define ECHO_PIN 8

SR04 sr04 = SR04 (ECHO_PIN, TRIG_PIN);
long distance;

void setup() {
  // put your setup code here, to run once:
  
  Serial.begin (9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  distance = sr04.DistanceAvg(); //DistanceAvg is a function that returns the average distance it is just slower
  Serial.print(distance);
  Serial.println("cm");
  //delay(10);
  
}
