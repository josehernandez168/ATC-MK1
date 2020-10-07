
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

#define BUZZER 4
#define LED 12


void moveForward(float);

  void setup() {
    // put your setup code here, to run once:
  
    pinMode(FORWARD_R, OUTPUT);
    pinMode(FORWARD_L, OUTPUT);
    pinMode(REVERSE_R, OUTPUT);
    pinMode(REVERSE_L, OUTPUT);
    pinMode(SPEED_R, OUTPUT);
    pinMode(SPEED_L, OUTPUT);

// moveForward (distance you want the car to move in cm)
// moveReverse (distance you want the car to move in cm)
// turnRight (angle in degrees, calibration in degrees)
// turnLeft (angle in degrees, calibration in degrees) 

//
//      analogWrite(SPEED_R, 252);
//      analogWrite(SPEED_L, 255);
//      
//      digitalWrite(REVERSE_R, HIGH);
//      delay (3000);
//      digitalWrite(REVERSE_R, LOW);

    moveForward(80, 10, 252, 255); // -1.5 for calibration if distance is 50
    moveForward(70, -1.5, 50, 255);
    moveForward(90, -1.5, 255, 50);
    moveForward(80, 10, 252, 255);
    
//    turnRight(90, 13);//calibration = 6.5 on carpet
//    delay(500);
//    moveReverse(50, -4);
//    delay(500);
//    turnLeft(90, 26);// calibration = 13.5 on carpet
//    delay(500);
//    moveForward(50, -1.5);
    
  }
  
  void loop() {
    // put your main code here, to run repeatedly:
  
  }

  void moveForward(float distance, float calibration, int rspeed, int lspeed)
  { 
    analogWrite(SPEED_R, rspeed);//75 is the minimum so motors can run
    analogWrite(SPEED_L, lspeed);
    
    digitalWrite(FORWARD_R, HIGH);
    digitalWrite(FORWARD_L, HIGH);

    delay(13.88 * (distance - calibration)); 
    
    digitalWrite(FORWARD_R, LOW);
    digitalWrite(FORWARD_L, LOW); 

  } 

  void moveReverse(float distance, float calibration)
  {
    analogWrite(SPEED_R, 252);
    analogWrite(SPEED_L, 255);
    
    digitalWrite(REVERSE_R, HIGH);
    digitalWrite(REVERSE_L, HIGH);

    delay(13.88 * (distance - calibration)); 
    
    digitalWrite(REVERSE_R, LOW);
    digitalWrite(REVERSE_L, LOW); 

  }

  void turnRight (float angle, float calibration)
  { 
    analogWrite(SPEED_R, 252);
    analogWrite(SPEED_L, 255);
    
    digitalWrite(FORWARD_L, HIGH);
    digitalWrite(REVERSE_R, HIGH);

    delay(4.6535 * (angle + calibration) + 6.6495); 
    
    digitalWrite(FORWARD_L, LOW);
    digitalWrite(REVERSE_R, LOW); 

  }

  
  void turnLeft (float angle, float calibration)
  { 
    analogWrite(SPEED_R, 252);
    analogWrite(SPEED_L, 255);
    
    digitalWrite(FORWARD_R, HIGH);
    digitalWrite(REVERSE_L, HIGH);

    delay(4.6535 * (angle + calibration) + 6.6495);
    
    digitalWrite(FORWARD_R, LOW);
    digitalWrite(REVERSE_L, LOW); 

  } 

  //NOTES
  /*
    1) I am having issues with the rotational speed. The more I try it the less the angle I get
       This may be due to the fact that the bateries are running out of power. Also it happened 
       after I tested the rotation to the contrary side. Probably it has to do with the gears. 
       
   */
