/*
The sensor outputs provided by the library are the raw
16-bit values obtained by concatenating the 8-bit high and
low accelerometer and gyro data registers. They can be
converted to units of g and dps (degrees per second) using
the conversion factors specified in the datasheet for your
particular device and full scale setting (gain).

Example: An LSM6DS33 gives an accelerometer Z axis reading
of 16276 with its default full scale setting of +/- 2 g. The
LA_So specification in the LSM6DS33 datasheet (page 15)
states a conversion factor of 0.061 mg/LSB (least
significant bit) at this FS setting, so the raw reading of
16276 corresponds to 16276 * 0.061 = 992.8 mg = 0.9928 g.
*/
#define USE_TEENSY_HW_SERIAL

#include <Wire.h>
#include <LSM6.h>  //IMU
#include <XL320.h> //motors

#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>

//-----------> Declarations <-----------------------------------------------------
LSM6 imu;
XL320 wheels;
XL320 joints;

int i = 0;

double kx = 1.0036;
double ky = 0.99293;
double kz = 0.99564;

double bx = 0.084966;
double by = 0.094076;
double bz = -0.21681;

double a_xy = -0.001849;
double a_xz = -0.0021216;
double a_yz = 0.0043796;

double bias_gyro_x = 0.0;
double bias_gyro_y = 0.0;
double bias_gyro_z = 0.0;

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

std_msgs::Float32MultiArray imu_msg;
ros::Publisher pub_imu("/testbot/imu", &imu_msg);

ros::Subscriber<std_msgs::Int16MultiArray> wheel_sub("wheel_vel", wheel_callback);
ros::Subscriber<std_msgs::Int16MultiArray> joint_sub("joint_ang", joint_callback);

void setup()
{
  //--> WiFi Serial
  Serial4.begin(57600); //38400 //115200 // ursprünglich serial 4 // ArduinoHardware.h: #elif defined(USE_TEENSY_HW_SERIAL) or defined(USE_STM32_HW_SERIAL)  ----->   iostream = &Serial4;
  delay(300);

  //--> Servo Serial
  Serial2.begin(1000000);
  wheels.begin(Serial2);
  delay(300);

  Serial3.begin(1000000);
  joints.begin(Serial3);
  delay(300);
  
  //Serial.begin(57600);
  Wire.begin();

  if (!imu.init())
  {
    //Serial.println("Failed to detect and initialize IMU!");
    while (1);
  }
  imu.enableDefault();


  // Kann man für später beibehalten. Robotor muss gut initialisiert werden (stillstehend)
  /*for (int i=0; i<100; i++)
  {
    imu.read();
    delay(100);
  } //100 mal 100ms warten? */

  for (int i=0; i<30; i++)
  {//msg.data[2]
    imu.read();
    bias_gyro_x += imu.g.x * 8.75 * 0.001 * 0.01745329251994329576923690768489;
    bias_gyro_y += imu.g.y * 8.75 * 0.001 * 0.01745329251994329576923690768489;
    bias_gyro_z += imu.g.z * 8.75 * 0.001 * 0.01745329251994329576923690768489;
    delay(100);
  }

  bias_gyro_x = bias_gyro_x/30;
  bias_gyro_y = bias_gyro_y/30;
  bias_gyro_z = bias_gyro_z/30;

  nh.initNode();

  imu_msg.layout.data_offset = 0;
  imu_msg.layout.dim = (std_msgs::MultiArrayDimension *) calloc(2, sizeof(std_msgs::MultiArrayDimension));
  imu_msg.layout.dim[0].label = "imu_values";
  imu_msg.layout.dim[0].size = 6;
  imu_msg.layout.dim[0].stride = 1 * 6;
  imu_msg.data = (float *)calloc(6,sizeof(float));
  imu_msg.data_length = 6;
  
  nh.advertise(pub_imu);

  nh.subscribe(wheel_sub);
  nh.subscribe(joint_sub);

  delay(300);
}

//-----------> loop <-------------------------------------------------------------

void loop()
{
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

  if(i == 10)
  {    
    //imu
    imu.read();
    //nh.loginfo("read imu");

    //                                    mdps -> dps -> rps
    // +- 245dps     raw reading data * 8.75 * 0.001 * 0.01745329251994329576923690768489
    imu_msg.data[0] = (float) imu.g.x * 8.75 * 0.001 * 0.01745329251994329576923690768489 - bias_gyro_x;
    imu_msg.data[1] = (float) imu.g.y * 8.75 * 0.001 * 0.01745329251994329576923690768489 - bias_gyro_y;
    imu_msg.data[2] = (float) imu.g.z * 8.75 * 0.001 * 0.01745329251994329576923690768489 - bias_gyro_z;

    // +-2g      mg  ->  g ->   m/s²
    //         0.061 * 0.001 * 9.81
    //imu.a.x *= 0.061 * 0.001 * 9.81;
    //imu.a.y *= 0.061 * 0.001 * 9.81;
    //imu.a.z *= 0.061 * 0.001 * 9.81;

    //imu_msg.data[3] = (float) kx*(imu.a.x-a_xy*imu.a.y+a_xz*imu.a.z-bx);
    //imu_msg.data[4] = (float) ky*(imu.a.y-a_yz-by);
    //imu_msg.data[5] = (float) kz*(imu.a.z-bz);

    imu_msg.data[3] = imu.a.x * 0.061 * 0.001 * 9.81;
    imu_msg.data[4] = imu.a.y * 0.061 * 0.001 * 9.81;
    imu_msg.data[5] = imu.a.z * 0.061 * 0.001 * 9.81;
    
    pub_imu.publish( &imu_msg); 

    i = 0;
  }
  
  ++i;
  nh.spinOnce();

  delay(1); // 1ms motor, 100ms imu example --> imu works with 1.66kHz ODR (output data rate)
}
