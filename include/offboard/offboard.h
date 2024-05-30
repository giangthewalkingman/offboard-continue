#ifndef OFFBOARD_H_
#define OFFBOARD_H_

#include <ros/ros.h>
#include <ros/time.h>
#include <iostream>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>
#include <unistd.h>
#include <ncurses.h>
// #include <curses.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Dense> 
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <offboard/PoseRequest.h>
#include <geometry_msgs/PoseStamped.h>

#define DEVICE_ID 0x08

class OffboardControl
{
    public:
    OffboardControl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private, bool input_setpoint);
    ~OffboardControl();
    private:
    ros::NodeHandle nh_;
	ros::NodeHandle nh_private_;

    ros::Subscriber arm_mode_sub;
    ros::Subscriber subOdom ;
    ros::Subscriber odom_sub; // odometry subscriber
    ros::Subscriber current_mavros_pose_sub;
    ros::Subscriber target_yaw_sub;
    ros::Subscriber target_pose_sub;

    ros::Publisher odom_error_pub; //publish odom error before arm

    std_msgs::Bool arm_mode_;
    nav_msgs::Odometry current_odom_;
    ros::Time operation_time_1, operation_time_2;
    Eigen::Vector3d vehicle_pos_, target_pos_;

    int fd = 0;
    // for PID
    const double PI = 3.141592653589793238463; // PI
    double Kp_yaw = -30, Ki_yaw, Kd_yaw;
    double Kp_throt = 30, Ki_throt, Kd_throt; 

    double steering_value, throttle_value;
    int16_t SPEED_INCREMENT = 5;
    bool odom_received_ = false; // check received odometry or not
    bool i2c_flag_ = false; 
    bool rc_flag_ = false;
    // uint8_t flags = 0;
    double yaw_ = 0;
    const double length = 20.0;
    double steering_noise = 0.0;
    double distance_noise = 0.0;
    double steering_drift = 0.0;

    void armModeCallback(const std_msgs::Bool::ConstPtr &msg);
    void odomCallback(const nav_msgs::Odometry &odomMsg);
    void yawCallback(const std_msgs::Float32 &yawMsg);
    void targetPoseCallback(const offboard::PoseRequest &poseMsg);
    void currentMavrosPoseCallback(const geometry_msgs::PoseStamped &msg);

    void offboard();
    void landing();
    void teleopControl();
    void i2cSetup();
    void waitForArming(double hz);
    void initNcurses();
    void cleanupNcurses();
    void printPWM(int16_t throttle_val, int16_t steering_val);
    // rc_flag_ << 1; i2c_flag_ << 0;  
    void sendI2CMsg(uint8_t throttle_pwm, uint8_t steering_pwm, uint8_t flags);
    void odomHandler();
    double odomTime = 0;
    double sensorOffsetX = 0;
    double sensorOffsetY = 0;
    double roll, pitch, yaw;
    double vehicleRoll = 0, vehicleYaw = 0, vehiclePitch = 0, vehicleX = 0, vehicleY = 0, vehicleZ = 0;
    double calculateDistance(const Eigen::Vector3d& pose1, const Eigen::Vector3d& pose2);
    Eigen::Vector3d vehicleVBase = Eigen::Vector3d::Zero();
    void yawTest();
    void pidTest();
    Eigen::Vector3d target_setpoint;
    double target_yaw = 0;
    double yaw_error = 0, target_error = 0;
};

inline Eigen::Vector3d toEigen(const geometry_msgs::Point &p) {
  Eigen::Vector3d ev3(p.x, p.y, p.z);
  return ev3;
}

inline Eigen::Vector3d toEigen(const geometry_msgs::PoseStamped &p) {
  return toEigen(p.pose.position);
}

inline Eigen::Quaterniond toEigen(const geometry_msgs::Quaternion &p) {
  Eigen::Quaterniond q4(p.w ,p.x, p.y, p.z);
    return q4;
}

inline Eigen::Vector3d toEigen(const geometry_msgs::Vector3 &v3) {
  Eigen::Vector3d ev3(v3.x, v3.y, v3.z);
  return ev3;
}

inline double ToEulerYaw(const Eigen::Quaterniond& q){
    Eigen::Vector3d angles;    //yaw pitch roll
    const auto x = q.x();
    const auto y = q.y();
    const auto z = q.z();
    const auto w = q.w();
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    return yaw;
  }

#endif