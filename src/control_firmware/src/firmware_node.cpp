#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <EEPROM.h>
#include <string.h>
//#include <SCServo.h>
//#include <ArduinoJson.h>
#include <SoftwareSerial.h>


// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project

#include <Arduino.h>

#define WAIT_ON_CALL()           \
static bool isFirstCall = false; \
if (!isFirstCall) {              \
    isFirstCall = true;          \
    return;                      \
}                                \

int pwm = 0;
void motorControlCb( const std_msgs::String&);

ros::NodeHandle nh;

int valueMapper(const int x, int imin, int imax, int omin, int omax) {
    return (x - imin) * ((omax - omin) / (imax - imin)) + omin; 
};

void motorControlCb( const std_msgs::String& msg) {
  WAIT_ON_CALL()

  int len = strlen(msg.data);
  int steering = 0;
  uint8_t idx = len - 2;

  String msgString = String(msg.data);
  idx = msgString.indexOf(':');
  pwm = atoi(msgString.substring(0,idx).c_str());
  steering = atoi(msgString.substring(idx+1).c_str());

  if (steering == 0) {
    if (pwm < 0) {
      // motor 1
      digitalWrite(22, HIGH); // input 1 
      digitalWrite(23, LOW); // input 2
      // motor 2
      digitalWrite(24, HIGH); // input 1
      digitalWrite(25, LOW); // input 2
       
      // motor 3
      digitalWrite(26, LOW); // input 1
      digitalWrite(27, HIGH); // input 2
      // motor 4
      digitalWrite(28, HIGH); // input 1
      digitalWrite(29, LOW); // input 2
    
      pwm = abs(pwm);
      analogWrite(2, pwm);
      analogWrite(3, pwm);
      analogWrite(4, pwm);
      analogWrite(5, pwm);    
    } else {
   // motor 1
      digitalWrite(22, LOW); // input 1 
      digitalWrite(23, HIGH); // input 2
      
      // motor 2
      digitalWrite(24, LOW); // input 1
      digitalWrite(25, HIGH); // input 2
       
      // motor 3
      digitalWrite(26, HIGH); // input 1
      digitalWrite(27, LOW); // input 2
      // motor 4
      digitalWrite(28, LOW); // input 1
      digitalWrite(29, HIGH); // input 2   
  

      pwm = abs(pwm);
      analogWrite(2, pwm);
      analogWrite(3, pwm);
      analogWrite(4, pwm);
      analogWrite(5, pwm);  
    }
  } else if (steering < 0) {
   // motor 1
    digitalWrite(22, LOW); // input 1 
    digitalWrite(23, LOW); // input 2
    // motor 2
    digitalWrite(24, HIGH); // input 1
    digitalWrite(25, LOW); // input 2
     
    // motor 3
    digitalWrite(26, HIGH); // input 1
    digitalWrite(27, HIGH); // input 2
    // motor 4
    digitalWrite(28, HIGH); // input 1
    digitalWrite(29, LOW); // input 2

    pwm = abs(pwm);
    analogWrite(2, pwm);
    analogWrite(3, pwm);
    analogWrite(4, pwm);
    analogWrite(5, pwm);

  } else {
    // motor 1
    digitalWrite(22, LOW); // input 1 
    digitalWrite(23, HIGH); // input 2
    
    // motor 2
    digitalWrite(24, HIGH); // input 1
    digitalWrite(25, LOW); // input 2
     
    // motor 3
    digitalWrite(26, LOW); // input 1
    digitalWrite(27, HIGH); // input 2
    // motor 4
    digitalWrite(28, LOW); // input 1
    digitalWrite(29, HIGH); // input 2   


    pwm = abs(pwm);
    analogWrite(2, pwm);
    analogWrite(3, pwm);
    analogWrite(4, pwm);
    analogWrite(5, pwm);
  }
}

ros::Subscriber<std_msgs::String> motorSubscriber("motor_control", &motorControlCb );

void setup() {   
  pinMode(22, OUTPUT);
  pinMode(23, OUTPUT);
  pinMode(24, OUTPUT);
  pinMode(25, OUTPUT);
  pinMode(26, OUTPUT);
  pinMode(27, OUTPUT);
  pinMode(28, OUTPUT);
  pinMode(29, OUTPUT);
  
  nh.initNode();

  delay(100);

  pwm  = 0;
  nh.subscribe(motorSubscriber);
  pwm  = 0;

}

void loop() {  
  pwm = 0;
  nh.spinOnce();
  delay(10);
  pwm = 0;
}











