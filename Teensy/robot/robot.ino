#define USE_TEENSY_HW_SERIAL // enables communication via wlan
#include <ros.h>
#include "std_msgs/Int16MultiArray.h"

#include "XL320.h"
//#include <Dynamixel.h>

//-----------> Declarations <-----------------------------------------------------
XL320 wheels;
XL320 joints;

int wheel_vel[8];
int joint_ang[] = {512, 512, 512};

//-----------> CallBack function of Subscriber <----------------------------------

void wheel_callback(const std_msgs::Int16MultiArray& msg)
{
    for (int i=0; i<8; i++)
      wheel_vel[i] = msg.data[i];
}

void joint_callback(const std_msgs::Int16MultiArray& msg)
{
    for (int i=0; i<3; i++)
      joint_ang[i] = msg.data[i];
}


//-----------> Initialize ROS Node <----------------------------------------------

ros::NodeHandle  nh;

//-----------> Subscriber definition <--------------------------------------------

ros::Subscriber<std_msgs::Int16MultiArray> wheel_sub("wheel_vel", wheel_callback);
ros::Subscriber<std_msgs::Int16MultiArray> joint_sub("joint_ang", joint_callback);


//-----------> setup <------------------------------------------------------------

void setup() {
    //-------------------> WiFi Serial
    Serial4.begin(57600); //38400 //115200 // ursprünglich serial 4 // ArduinoHardware.h: #elif defined(USE_TEENSY_HW_SERIAL) or defined(USE_STM32_HW_SERIAL)  ----->   iostream = &Serial4;
    delay(300);

    //-------------------> Servo Serial
    Serial2.begin(1000000);
    wheels.begin(Serial2);
    delay(300);

    Serial3.begin(1000000);
    joints.begin(Serial3);
    delay(300);
    
    //-------------------> ROS Stuff
    nh.initNode();
    nh.subscribe(wheel_sub);
    nh.subscribe(joint_sub);
    delay(300);

}


//-----------> loop <-------------------------------------------------------------

void loop() {

    // set wheel velocity
    wheels.setJointSpeed(1, wheel_vel[0]);
    wheels.setJointSpeed(2, wheel_vel[1] ^ 0x400); // move in opposite direction of wheel 1
    wheels.setJointSpeed(3, wheel_vel[2]);
    wheels.setJointSpeed(4, wheel_vel[3] ^ 0x400); // move in opposite direction of wheel 3
    wheels.setJointSpeed(5, wheel_vel[4]);
    wheels.setJointSpeed(6, wheel_vel[5] ^ 0x400); // move in opposite direction of wheel 5
    wheels.setJointSpeed(7, wheel_vel[6]);
    wheels.setJointSpeed(8, wheel_vel[7] ^ 0x400); // move in opposite direction of wheel 7

    // set joint position
    joints.moveJoint(11, 1024-joint_ang[0]);
    joints.moveJoint(12, 1024-joint_ang[1]-1);
    joints.moveJoint(13, joint_ang[2]+2);
    
    nh.spinOnce();
    delay(1);
}
