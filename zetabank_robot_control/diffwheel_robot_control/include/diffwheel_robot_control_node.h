#ifndef DIFFWHEEL_ROBOT_CONTROL_NODE_H
#define DIFFWHEEL_ROBOT_CONTROL_NODE_H

#include <ros/ros.h>
#include <ros/time.h>

#include <serial/serial.h>

#include <std_msgs/String.h>
#include <std_msgs/Char.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>

#include <boost/thread/thread.hpp>


#define __DEBUG
//#define ___DEBUG

#define LEFT                                  0
#define RIGHT                                 1 

#define LINEAR                                0
#define ANGULAR                               1

#define MAX_LINEAR_VELOCITY                2.0             // m/s
#define MAX_ANGULAR_VELOCITY               2.0             // rad/s
#define LINEAR_X_MAX_VELOCITY              2.0

#define PI  3.1415926535897932384626433832795
#define MATH_RAD2DEG    57.2957795f
#define MATH_DEG2RAD    0.0174532f

#define WHEEL_NUM                           2
#define WHEEL_RADIUS                       0.0812          // meter
#define WHEEL_SEPARATION                  0.360          // meter
#define GEARRATIO                           26
#define ENCODER4CH                          (1024*4.0)
//#define ENCODER_MIN                         -2147483648     // raw
//#define ENCODER_MAX                         2147483648      // raw
#define VELOCITY_UNIT                       2
#define DISTORPM                             ((60.0*GEARRATIO)/(2*PI*WHEEL_RADIUS))
#define RPMTODIS                             (1.0/DISTORPM)
#define CONTROL_MOTOR_SPEED_PERIOD       (1000/50)     //50hz. 20ms
#define DRIVE_INFOR_PUBLISH_PERIOD       (CONTROL_MOTOR_SPEED_PERIOD + CONTROL_MOTOR_SPEED_PERIOD/2)    //30hz. 33ms
//#define DRIVE_INFOR_PUBLISH_PERIOD       (1000/25)    //25hz. 40ms
#define ROTTODIS                             ((2.0*PI*WHEEL_RADIUS/GEARRATIO)/ENCODER4CH)
#define PULSETODIST                         ((2.0*PI*WHEEL_RADIUS)/(ENCODER4CH*GEARRATIO))
#define ST                                     0.01

#endif