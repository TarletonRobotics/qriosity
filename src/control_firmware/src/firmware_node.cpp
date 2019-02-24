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
int precision = 2;

const int SDIG1 = 2; // Arduino pin 4 is connected to MDDS10 pin DIG1.
const int SDIG2 = 3; // Arduino pin 7 is connected to MDDS10 pin DIG2.
const int SAN1 = 4; // Arduino pin 5 is connected to MDDS10 pin AN1.
const int SAN2 = 5; // Arduino pin 6 is connected to MDDS10 pin AN2.

const int MDIG1 = 12; // Arduino pin 4 is connected to MDDS10 pin DIG1.
const int MDIG2 = 9; // Arduino pin 7 is connected to MDDS10 pin DIG2.
const int MAN1 = 10; // Arduino pin 5 is connected to MDDS10 pin AN1.
const int MAN2 = 11; // Arduino pin 6 is connected to MDDS10 pin AN2.

void motorControlCb( const std_msgs::String&);
void steeringControlCb(const std_msgs::String&);

ros::NodeHandle nh;

template<class T>
T mapValue(const T x, T imin, T imax, T omin, T omax) {
    return (x - imin) * ((omax - omin) / (imax - imin)) + omin; 
};

////=================== STEERING CODE ============================= ///////

void stopActuator()
{
  analogWrite(SAN1,0);
  analogWrite(SAN2,0);
}//end stopActuator

void pushActuator()
{ 
  analogWrite(SAN1,255);
  analogWrite(SAN2,255);
}//end pushActuator

void pullActuator()
{
  analogWrite(SAN1,255);
  analogWrite(SAN2,255);
}//end pullActuator

void pushActuatorUntilStop(int destination)
{
  digitalWrite(SDIG1, HIGH);
  digitalWrite(SDIG2, HIGH);
  
  int temp = analogRead(A0); 
  difference = destination - temp;//check difference to see if continue moving, or stop

  while (difference > precision || difference < -precision)
  {
    destination = analogRead(A0);
    temp = analogRead(A0); //continue checking difference
    difference = destination - temp;
    pushActuator();
  }//end while
  
  delay(75);
  stopActuator();
}//end pushActuatorUntilStop

void pullActuatorUntilStop(int destination)
{
  digitalWrite(SDIG1, LOW);
  digitalWrite(SDIG2, LOW);
  
  int temp = analogRead(A0); //check difference to see if continue moving, or stop
  difference = destination - temp;

  while (difference > precision || difference < -precision)
  {
    destination = analogRead(A0);
    temp = analogRead(A0); //continue checking difference
    difference = destination - temp;
    pullActuator();
  }//end while
  
  delay(75);
  stopActuator();
}//end pullActuatorUntilStop

void steer(int destination) {
  //destination = 500; 
  currentPosition = analogRead(A0);//check where you are
  
  //Serial.print("Position    ");
  
  //Serial.println(analogRead(A0));
  
  difference = destination - currentPosition;//find out how far you are from the destination
  
  if (currentPosition > destination) { 
    pullActuatorUntilStop(destination);
    //Serial.println("pulling actuator");
  }// choose what action to take
  else if (currentPosition < destination) { 
    pushActuatorUntilStop(destination);
    //Serial.println("pushing actuator");
  }
  else if (difference < precision && difference > -precision) stopActuator();
}


void steeringControlCb(const std_msgs::String& msg) {
  destination = mapValue(int(atof(msg.data)), -1, 1, 400, 600);
  steer(destination);
}


////=================== END STEERING CODE ============================= ///////


void motorControlCb( const std_msgs::String& msg) {
  WAIT_ON_CALL()

  pwm = atoi(msg.data);
  
  if (pwm < 0) { 
    digitalWrite(MDIG1, HIGH);
    digitalWrite(MDIG2, HIGH);
  } else if (pwm > 0) { 
    digitalWrite(MDIG1, LOW);
    digitalWrite(MDIG2, LOW);
  }

  pwm = abs(pwm);
  analogWrite(MAN1, pwm);
  analogWrite(MAN2, pwm);
}

ros::Subscriber<std_msgs::String> motorSubscriber("motor_control", &motorControlCb );
ros::Subscriber<std_msgs::String> steeringSubscriber("steering_control", &steeringControlCb );

void setup() {    
  pinMode(SDIG1, OUTPUT); // Set Arduino pin 4 (DIG1) as output.
  pinMode(SDIG2, OUTPUT); // Set Arduino pin 7 (DIG2) as output.
  pinMode(SAN1, OUTPUT); // Set Arduino pin 5 (AN1) as output.
  pinMode(SAN2, OUTPUT); // Set Arduino pin 6 (AN2) as output.
  
  pinMode(MDIG1, OUTPUT); // Set Arduino pin 4 (DIG1) as output.
  pinMode(MDIG2, OUTPUT); // Set Arduino pin 7 (DIG2) as output.
  pinMode(MAN1, OUTPUT); // Set Arduino pin 5 (AN1) as output.
  pinMode(MAN2, OUTPUT); // Set Arduino pin 6 (AN2) as output.  
  
  nh.initNode();

  delay(100);

  pwm  = 0;
  nh.subscribe(motorSubscriber);
  nh.subscribe(steeringSubscriber);
  pwm  = 0;
}

void loop() {  
  pwm = 0;
  nh.spinOnce();
  delay(10);
  pwm = 0;
}










