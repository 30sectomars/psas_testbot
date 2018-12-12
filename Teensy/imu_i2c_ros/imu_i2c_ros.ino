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
#include <LSM6.h>
#include <ros.h>
#include <sensor_msgs/Imu.h>

LSM6 imu;
ros::NodeHandle  nh;

sensor_msgs::Imu msg;
ros::Publisher pub_imu("/testbot/imu", &msg);

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

void setup()
{
  //-------------------> WiFi Serial
    Serial4.begin(57600); //38400 //115200 // ursprünglich serial 4 // ArduinoHardware.h: #elif defined(USE_TEENSY_HW_SERIAL) or defined(USE_STM32_HW_SERIAL)  ----->   iostream = &Serial4;
    delay(300);
  
//  Serial.begin(57600);
  Wire.begin();

  if (!imu.init())
  {
//    Serial.println("Failed to detect and initialize IMU!");
    while (1);
  }
  imu.enableDefault();

  /*for (int i=0; i<100; i++)
  {
    imu.read();
    delay(100);
  }*/

  for (int i=0; i<30; i++)
  {
    imu.read();
    bias_gyro_x = bias_gyro_x + imu.g.x * 8.75 * 0.001 * 0.01745329251994329576923690768489;
    bias_gyro_y = bias_gyro_y + imu.g.y * 8.75 * 0.001 * 0.01745329251994329576923690768489;
    bias_gyro_z = bias_gyro_z + imu.g.z * 8.75 * 0.001 * 0.01745329251994329576923690768489;
    delay(100);
  }

  bias_gyro_x = bias_gyro_x/30;
  bias_gyro_y = bias_gyro_y/30;
  bias_gyro_z = bias_gyro_z/30;

  nh.initNode();
  nh.advertise(pub_imu);
  msg.header.frame_id = "imu";
}

void loop()
{
  imu.read();

  msg.angular_velocity.x = imu.g.x * 8.75 * 0.001 * 0.01745329251994329576923690768489 - bias_gyro_x;
  msg.angular_velocity.y = imu.g.y * 8.75 * 0.001 * 0.01745329251994329576923690768489 - bias_gyro_y;
  msg.angular_velocity.z = imu.g.z * 8.75 * 0.001 * 0.01745329251994329576923690768489 - bias_gyro_z;

  imu.a.x = imu.a.x * 0.061 * 0.001 * 9.81;
  imu.a.y= imu.a.y * 0.061 * 0.001 * 9.81;
  imu.a.z = imu.a.z * 0.061 * 0.001 * 9.81;

  msg.linear_acceleration.x = kx*(imu.a.x-a_xy*imu.a.y+a_xz*imu.a.z-bx);
  msg.linear_acceleration.y = ky*(imu.a.y-a_yz-by);
  msg.linear_acceleration.z = kz*(imu.a.z-bz);

  pub_imu.publish( &msg);
  nh.spinOnce();

  delay(100);
}
