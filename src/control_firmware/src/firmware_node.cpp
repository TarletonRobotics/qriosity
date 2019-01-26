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
const int forwards = 33;
const int backwards = 35;//assign relay INx pin to arduino pin
const int forwards1 = 37;
const int backwards1 = 39;

void motorControlCb( const std_msgs::String&);
void steeringControlCb(const std_msgs::String&);

ros::NodeHandle nh;

int valueMapper(const int x, int imin, int imax, int omin, int omax) {
    return (x - imin) * ((omax - omin) / (imax - imin)) + omin; 
};

// void strokeForward(float pos) {
//   float val = map(pos, -1.0, 1.0, 370.0, 600.0);
//   while (analogRead(A0) < val) {
//     digitalWrite(forwards, HIGH);
//     digitalWrite(backwards, LOW);
//   }
//   digitalWrite(forwards, LOW);
//   digitalWrite(backwards, LOW);a
// }

// void strokeBackward(float pos) {
//   float val = map(pos, -1.0,1.0, 370.0, 600.0);
//   while (analogRead(A0) > val) {
//     digitalWrite(forwards, LOW);
//     digitalWrite(backwards, HIGH);
//   }
//   digitalWrite(forwards, LOW);
//   digitalWrite(backwards, LOW);
// }


float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void turn(float pos) {
  //for (float i = -1; i < 1; i += 0.1) {
    //pos = i;
    float precision = 5;
    float desiredPos = mapfloat(pos, -1.0, 1.0, 225, 460);

    if (pos < 0) {
         digitalWrite(forwards, HIGH);
         digitalWrite(backwards, LOW); 
         digitalWrite(forwards1, HIGH);
         digitalWrite(backwards1, LOW);
         //delay(100);
        
    } else if (pos > 0) {
      digitalWrite(forwards, LOW);
          digitalWrite(backwards, HIGH); 
         digitalWrite(forwards1, LOW);
         digitalWrite(backwards1, HIGH);
         //delay(100);
        
    } else {
        digitalWrite(forwards, HIGH);
        digitalWrite(backwards, HIGH); 
        digitalWrite(forwards1, HIGH);
        digitalWrite(backwards1, HIGH);
        //delay(100);
    }
    //delay(10);

  // while (true) {
  //   float currentPos = analogRead(A0);
  //   float difference = fabs(desiredPos - currentPos);
    
  //   //Serial.print("desired position ");
  //   //Serial.print(desiredPos);
  //   //Serial.print(" current position ");
  //   //Serial.print(currentPos);
  //   //Serial.print(" difference ");
  //   //Serial.print(difference);
    
  //   //Serial.print('\n');
    
  //   if (difference < precision) {
  //     digitalWrite(forwards, HIGH);
  //     digitalWrite(backwards, HIGH);
  //     digitalWrite(forwards1, HIGH);
  //     digitalWrite(backwards1, HIGH);
      
  //     break;
  //   }
  
  //   if (currentPos > desiredPos) {
  //     //Serial.println("moving forward");
  //     digitalWrite(forwards, HIGH);
  //     digitalWrite(backwards, LOW); 
  //     digitalWrite(forwards1, HIGH);
  //     digitalWrite(backwards1, LOW);
      
  //   //  delay(10)  ; 
  //   } else {
  //     //Serial.println("moving backward");
  //     digitalWrite(forwards, LOW);
  //     digitalWrite(backwards, HIGH);
  //     digitalWrite(forwards1, LOW);
  //     digitalWrite(backwards1, HIGH);
      
  //   //  delay(10);
  //   }

    
  // } 
  //}
}

void steeringControlCb(const std_msgs::String& msg) {
  float steering = atof(msg.data);
  turn(steering);
}

void motorControlCb( const std_msgs::String& msg) {
  WAIT_ON_CALL()

  int len = strlen(msg.data);
  int steering = 0;
  uint8_t idx = len - 2;

  String msgString = String(msg.data);
  idx = msgString.indexOf(':');
  pwm = atoi(msgString.substring(0,idx).c_str());
  //steering = atoi(msgString.substring(idx+1).c_str());

  //if (steering == 0) {
    if (pwm < 0) {
      // motor 1
      //digitalWrite(22, HIGH); // input 1 
      //digitalWrite(23, LOW); // input 2
      // motor 2
      //digitalWrite(24, HIGH); // input 1
      //digitalWrite(25, LOW); // input 2
       
      // motor 3
      digitalWrite(26, LOW); // input 1
      digitalWrite(27, HIGH); // input 2
      // motor 4
      digitalWrite(28, HIGH); // input 1
      digitalWrite(29, LOW); // input 2
    
      pwm = abs(pwm);
      //analogWrite(2, pwm);
      //analogWrite(3, pwm);
      analogWrite(4, pwm);
      analogWrite(5, pwm);    
    } else {
   // motor 1
      //digitalWrite(22, LOW); // input 1 
      //digitalWrite(23, HIGH); // input 2
      
      // motor 2
      //digitalWrite(24, LOW); // input 1
      //digitalWrite(25, HIGH); // input 2
       
      // motor 3
      digitalWrite(26, HIGH); // input 1
      digitalWrite(27, LOW); // input 2
      // motor 4
      digitalWrite(28, LOW); // input 1
      digitalWrite(29, HIGH); // input 2   
  

      pwm = abs(pwm);
      //analogWrite(2, pwm);
      //analogWrite(3, pwm);
      analogWrite(4, pwm);
      analogWrite(5, pwm);  
    }
  //} else if (steering < 0) {
  //  // motor 1
  //   digitalWrite(22, HIGH); // input 1 
  //   digitalWrite(23, LOW); // input 2
    
  //   // motor 2
  //   digitalWrite(24, LOW); // input 1
  //   digitalWrite(25, HIGH); // input 2
     
  //   // motor 3
  //   digitalWrite(26, HIGH); // input 1
  //   digitalWrite(27, LOW); // input 2
  //   // motor 4
  //   digitalWrite(28, HIGH); // input 1
  //   digitalWrite(29, LOW); // input 2   

  //   pwm = abs(pwm);
  //   analogWrite(2, pwm);
  //   analogWrite(3, pwm/2);
  //   analogWrite(4, pwm);
  //   analogWrite(5, pwm/2);

  //} else if (steering > 0) {
  //   // motor 1
  //   digitalWrite(22, LOW); // input 1 
  //   digitalWrite(23, HIGH); // input 2
     
  //   // motor 2
  //   digitalWrite(24, HIGH); // input 1
  //   digitalWrite(25, LOW); // input 2
     
  //   // motor 3
  //   digitalWrite(26, LOW); // input 1
  //   digitalWrite(27, HIGH); // input 2
  //   // motor 4
  //   digitalWrite(28, LOW); // input 1
  //   digitalWrite(29, HIGH); // input 2   


  //   pwm = abs(pwm);
  //   analogWrite(2, pwm);
  //   analogWrite(3, pwm/2);
  //   analogWrite(4, pwm);
  //   analogWrite(5, pwm/2);
  //}
}

//ros::Subscriber<std_msgs::String> motorSubscriber("motor_control", &motorControlCb );
ros::Subscriber<std_msgs::String> steeringSubscriber("steering_control", &steeringControlCb );

void setup() {   
  // pinMode(22, OUTPUT);
  // pinMode(23, OUTPUT);
  // pinMode(24, OUTPUT);
  // pinMode(25, OUTPUT);
  pinMode(forwards, OUTPUT);//set relay as an output
  pinMode(backwards, OUTPUT);//set relay as an output
  pinMode(forwards1, OUTPUT);//set relay as an output
  pinMode(backwards1, OUTPUT);//set relay as an output
  
  // pinMode(26, OUTPUT);
  // pinMode(27, OUTPUT);
  // pinMode(28, OUTPUT);
  // pinMode(29, OUTPUT);
  
  nh.initNode();

  delay(100);

  pwm  = 0;
  //nh.subscribe(motorSubscriber);
  nh.subscribe(steeringSubscriber);
  //nh.publish(positionPublisher);
  pwm  = 0;

  turn(0);
}

void loop() {  
  pwm = 0;
  nh.spinOnce();
  delay(10);
  pwm = 0;
}











