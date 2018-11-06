#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <EEPROM.h>
#include <string.h>
#include <SCServo.h>
#include <ArduinoJson.h>
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

  char buf[idx];

  steering = atoi((char*)&msg.data[idx+1]);
  memcpy(buf, msg.data, idx);
  pwm = atoi(buf);

  if (steering == 0) {
    if (pwm > 0) {
      digitalWrite(2, LOW);
      digitalWrite(3, HIGH);
    } else {
      digitalWrite(2, HIGH);
      digitalWrite(3, LOW);
    }

    analogWrite(4, abs(pwm));
    analogWrite(5, abs(pwm));
  } else if (steering < 0) {
    digitalWrite(2, LOW);
    digitalWrite(3, LOW);

    analogWrite(4, abs(pwm));
    analogWrite(5, abs(pwm));
  } else {
    digitalWrite(2, HIGH);
    digitalWrite(3, HIGH);

    analogWrite(4, abs(pwm));
    analogWrite(5, abs(pwm));  
  }
}

ros::Subscriber<std_msgs::String> motorSubscriber("motor_control", &motorControlCb );

void setup() {   
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  
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











