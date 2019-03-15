#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <EEPROM.h>
#include <string.h>
//#include <SCServo.h>
//#include <ArduinoJson.h>
#include <SoftwareSerial.h>

#include <Arduino.h>

#define WAIT_ON_CALL()           \
static bool isFirstCall = false; \
if (!isFirstCall) {              \
    isFirstCall = true;          \
    return;                      \
}                                \

int pwm = 0;
int destination = 0;
int difference = 0;
int currentPosition = 0;
int precision = 5;
int startMillis = 0;

#define STRAIGHT 0
#define LEFT 1
#define RIGHT -1 

int orientation;

const int SDIG1 = 34; // Arduino pin 4 is connected to MDDS10 pin DIG1
const int SDIG2 = 12; // Arduino pin 7 is connected to MDDS10 pin DIG2.
const int SAN1 = 32; // Arduino pin 5 is connected to MDDS10 pin AN1.
const int SAN2 = 10; // Arduino pin 6 is connected to MDDS10 pin AN2.

const int MAN1 = 3; // Arduino pin 4 is connected to MDDS10 pin DIG1.
const int MAN2 = 5; // Arduino pin 7 is connected to MDDS10 pin DIG2.

void motorControlCb( const std_msgs::String&);
void steeringControlCb(const std_msgs::String&);

ros::NodeHandle nh;

template<class T>
T mapValue(const T x, T imin, T imax, T omin, T omax) {
    return (x - imin) * ((omax - omin) / (imax - imin)) + omin; 
};

////=================== STEERING CODE ============================= ///////

// void stopActuator(int id )
// {
//   analogWrite(id,0);
// //  analogWrite(SAN2,0);
// }//end stopActuator

// void pushActuator(int id)
// { 
//   analogWrite(id, 255);
//   //analogWrite(SAN2, 255);
// }//end pushActuator

// void pullActuator(int id)
// {
//   analogWrite(id,255);
//   //analogWrite(SAN2,255);
// }//end pullActuator

// void pushActuatorUntilStop(int destination)
// {
//   digitalWrite(SDIG1, HIGH);
//   digitalWrite(SDIG2, LOW);
  
//   int temp = analogRead(A0); 
//   difference = destination - temp;//check difference to see if continue moving, or stop

//   while (difference > precision || difference < -precision)
//   {
//     destination = analogRead(A0);
//     temp = analogRead(A0); //continue checking difference
//  difference = destination - temp;
//     pushActuator();
//   }//end while
  
//   delay(75);
//   stopActuator();
// }//end pushActuatorUntilStop

// void pullActuatorUntilStop(int destination)
// {
//   digitalWrite(SDIG1, LOW);
//   digitalWrite(SDIG2, HIGH);
  
//   int temp = analogRead(A10); //check difference to see if continue moving, or stop
//   difference = destination - temp;

//   while (difference > precision || difference < -precision)
//   {
//     destination = analogRead(A0);
//     temp = analogRead(A0); //continue checking difference
//     difference = destination - temp;
//     pullActuator();
//   }//end while
  
//   delay(75);
//   stopActuator();
// }//end pullActuatorUntilStop

inline void setLeft() {
  digitalWrite(SDIG1, HIGH) ;
  digitalWrite(SDIG2, LOW) ;
    
}

inline void setRight() {
  digitalWrite(SDIG1, LOW) ;
  digitalWrite(SDIG2, HIGH) ;
}

void continousSteering(int target) {
  int difference1, difference2;
  int target1, target2;

  if (target != orientation) {
    if (target == STRAIGHT) {
      target1 = 495;
      target2 = 525;
      if (orientation == LEFT) {
        setRight();
      } else {
        setLeft();
      }
    } else {
      if (target == LEFT) {
        target1 = 660;
        target2 = 305;
        setLeft();
      } else if (target == RIGHT) {
        target1 = 290;
        target2 = 690;
        setRight();
      } 
    }
  }

  analogWrite(SAN1, 255);
  analogWrite(SAN2, 255);
   
 do {  
  difference1 = target1 - analogRead(A10);
  difference2 = target2 - analogRead(A12);

  if (abs(difference1) < precision) analogWrite(SAN1, 0);
  if (abs(difference2) < precision)analogWrite(SAN2, 0);

 } while (abs(difference1) > precision || abs(difference2) > precision);

 orientation = target;
}


void steeringControlCb(const std_msgs::String& msg) {
  int direction = atoi(msg.data);

  continousSteering(direction);
 //  int steer = float(atof(msg.data));

 //  if (steer < 0) {
 //  	digitalWrite(SDIG1, HIGH);
 //  	digitalWrite(SDIG2, HIGH);
	//   analogWrite(SAN1, 255);
 //    analogWrite(SAN2, 255);
	//   delay(100);
 //  } else if (steer > 0) {
 //  	digitalWrite(SDIG1, LOW);
 //  	digitalWrite(SDIG2, LOW);
 //    analogWrite(SAN1, 255);
 //    analogWrite(SAN2, 255);
 //    delay(100);
 //  }

	// analogWrite(SAN1, 0);
 //  analogWrite(SAN2, 0);
}


////=================== END STEERING CODE ============================= ///////

void setDriveSpeed(int pwm){
  if(pwm > 0){
    //TODO go forward
    digitalWrite(MAN1, 1);
    digitalWrite(MAN2, 0);
  }else{
    //TODO go backward
    digitalWrite(MAN1, 0);
    digitalWrite(MAN2, 1);
  }
}

int currentPwm = 0;
int startTime = 0;
int currentTime = 0;
bool flippedDirection = false;

void motorControlCb( const std_msgs::String& msg) {
  int targetPwm = atoi(msg.data);

  if(targetPwm == 0){
    digitalWrite(MAN1, 0);
    digitalWrite(MAN2, 0);
    return;
  }

  if(!flippedDirection && targetPwm * currentPwm < 0){
    startTime = millis();
    flippedDirection = true;
    digitalWrite(MAN1, 0);
    digitalWrite(MAN2, 0);
  }

  currentTime = millis();
  if(!flippedDirection){ 
    setDriveSpeed(targetPwm); 
    currentPwm = targetPwm;
  }
  else if(currentTime - startTime > 5000){
    setDriveSpeed(targetPwm);
    currentPwm = targetPwm;
    flippedDirection = false;
  }
}

ros::Subscriber<std_msgs::String> motorSubscriber("motor_control", &motorControlCb );
ros::Subscriber<std_msgs::String> steeringSubscriber("steering_control", &steeringControlCb );

void setup() {    
  pinMode(SDIG1, OUTPUT); // Set Arduino pin 4 (DIG1) as output.
  pinMode(SDIG2, OUTPUT); // Set Arduino pin 7 (DIG2) as output.
  pinMode(SAN1, OUTPUT); // Set Arduino pin 5 (AN1) as output.
  pinMode(SAN2, OUTPUT); // Set Arduino pin 6 (AN2) as output.
  
  pinMode(3, OUTPUT); // Set Arduino pin 4 (DIG1) as output.
  pinMode(5, OUTPUT); // Set Arduino pin 7 (DIG2) as output.
  
  nh.initNode();

  delay(100);

  pwm  = 0;
  nh.subscribe(motorSubscriber);
  nh.subscribe(steeringSubscriber);
  pwm  = 0;

  digitalWrite(3, LOW);
  digitalWrite(5, LOW);

  startMillis = millis();

  orientation = 0;
}

void loop() {  
  pwm = 0;
  nh.spinOnce();
  delay(10);
  pwm = 0;
}










