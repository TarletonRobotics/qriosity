// controls the gait of a the quadruped robot.
// create publisher that publishes a new JointAngle message on the gait topic.
/*****************************************************************************
                   gait topic
         /----------------------------------------\
Jetson ->         JointAngle array                 -> Arduinos
         \----------------------------------------/
*****************************************************************************/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Joy.h>
#include <array>
#include <thread>
#include <cmath>
#include <functional>
//#include "include/json.hpp"

#define WAIT_ON_CALL()           \
static bool isFirstCall = false; \
if (!isFirstCall) {              \
    isFirstCall = true;          \
    return;                      \
}                                \

class MotionController 
{
public:
    const float pwmMin = 255;
    const float pwmMax = 250;

    const long minAngle = -200;
    const long maxAngle = 200;

    std_msgs::String motorCommand;
   
    ros::Subscriber joystickSubscriber;
    
    ros::Publisher motorControlPublisher;
    

    float valueMapper(const float x, float imin, float imax, float omin, float omax) {
        return (x - imin) * ((omax - omin) / (imax - imin)) + omin; 
    };

    void joystickSubscriberCb(const sensor_msgs::Joy::ConstPtr& msg) {  
        WAIT_ON_CALL()
        
        const long throttle = valueMapper(
            msg->axes[1], -1.0, 1.0, -255, 255 
        );

        const long steering = valueMapper(
            msg->axes[2], -1.0, 1.0, -1, 1
        );        

        motorCommand.data = std::to_string(throttle) + ":" + std::to_string(steering);
        
        ROS_INFO("Throttle: %s\n", motorCommand.data.c_str());
    }

    MotionController(ros::NodeHandle& handle) {
        // subscriber for joystic topics.
        joystickSubscriber = 
            handle.subscribe("joy", 10, &MotionController::joystickSubscriberCb, this);

        // publisher for motor pwm values.
        motorControlPublisher = handle.advertise<std_msgs::String>("motor_control", 1000);
    }

    void publish() {
        motorControlPublisher.publish(motorCommand);
    };

    MotionController(MotionController&&) = default;   
    MotionController(const MotionController&) = default;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "qriosity/control_node");

    ros::NodeHandle nodeHandle;

    ros::Rate loop_rate(32);

    auto controller = MotionController(nodeHandle);

    while (ros::ok()) {
        ros::spinOnce();

        controller.publish();

        loop_rate.sleep();
    }

    return 0;
}
