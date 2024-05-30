#include "offboard/offboard.h"

OffboardControl::OffboardControl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private, bool input_setpoint) : nh_(nh),
                                                                                                                      nh_private_(nh_private)                                        
                                                                                                                      {
    arm_mode_sub = nh_.subscribe("/arm_mode", 10, &OffboardControl::armModeCallback, this);
    odom_sub = nh_.subscribe("/odom", 1, &OffboardControl::odomCallback, this);
    target_yaw_sub = nh_.subscribe("/target_yaw_sub", 1, &OffboardControl::yawCallback, this);
    target_pose_sub = nh_.subscribe("/target_pose_sub", 1, &OffboardControl::targetPoseCallback, this);
    // current_mavros_pose_sub = nh_.subscribe("/mavros/vision_pose/pose", 1, &OffboardControl::currentMavrosPoseCallback, this);

    // subOdom = nh_.subscribe<nav_msgs::Odometry> ("/state_estimation", 5, odomHandler);
    nh_private_.param<bool>("/offboard_node/arm_mode_enable", arm_mode_.data);
    
    operation_time_1 = ros::Time::now();
    i2cSetup();
    waitForArming(10);
    offboard();
}

//destructor
OffboardControl::~OffboardControl() {
}

void OffboardControl::offboard() {
    std::printf("Choose mode: \n");
    std::printf("(1) Control with keyboard: \n");
    std::printf("(2) PID Test: \n");
    std::printf("(3) Yaw Test: \n");
    std::printf("(4) Cancel: \n");
    int mode;
    std::cin >> mode;
    switch (mode)
    {
    case 1:
        teleopControl();
        break;
    case 2:
        pidTest();
        break;
    case 3:
        yawTest();
    case 4:
        landing();
    // case n will have RC flag and Cancel is case n+1
    default:
        break;
    }
}

void OffboardControl::i2cSetup() {
    fd = wiringPiI2CSetup(DEVICE_ID);
    if (fd == -1) {
        // sendI2CMsg(127, 127, 1);
        std::cout << "[ INFO] FCU not connected.\n";
    } else {
        sendI2CMsg(127, 127, 1);
        std::cout << "[ INFO] FCU connected.\n";
    }
}

void OffboardControl::waitForArming(double hz) {
    ros::Rate rate(hz);
    std::printf("[ INFO] Waiting for Odometry... \n");
    while (ros::ok() && !odom_received_) {
        ros::spinOnce();
        rate.sleep();
    }
    std::printf("[ INFO] Odometry received \n");
    std::printf("[ INFO] Waiting for Arming... \n");
    // while(ros::ok() && arm_mode_.data == false) {
    //     rate.sleep();
    //     ros::spinOnce();
    // }
    // set the default pwm rate before arming
    throttle_value = 127;
    steering_value = 127;
    std::printf("[ INFO] Armed. \n");
}

void OffboardControl::teleopControl() {
    initNcurses(); // Initialize ncurses

    int ch;
    while ((ch = getch()) != 'q') { // Loop until 'q' is pressed
        switch(ch) {
            case KEY_UP:
                throttle_value++;
                break;
            case KEY_DOWN:
                throttle_value--;
                break;
            case KEY_LEFT:
                steering_value--;
                break;
            case KEY_RIGHT:
                steering_value++;
                break;
            default:
                std::cout << "Invalid input\n";
                break;
        }
        if(steering_value > 255) {
            steering_value = 255;
        } else if (steering_value < 0) {
            steering_value = 0;
        } 
        if(throttle_value > 255) {
            throttle_value = 255;
        } else if (throttle_value < 0) {
            throttle_value = 0;
        } 
        sendI2CMsg(throttle_value, steering_value, 1);
        printPWM(throttle_value, steering_value); 
    }

    cleanupNcurses(); // Cleanup ncurses
    landing();
}

// void OffboardControl::currentMavrosPoseCallback(const geometry_msgs::PoseStamped &msg) {
//     vehicle_pos_(0) = msg.pose.position.x;
//     vehicle_pos_(1) = msg.pose.position.y;
//     vehicle_pos_(2) = msg.pose.position.z;
//     Eigen::Quaterniond quat = toEigen(msg.pose.orientation);
//     vehicleYaw = ToEulerYaw(quat);
//     odom_received_ = true;
// }

void OffboardControl::yawCallback(const std_msgs::Float32 &yawMsg) {
    target_yaw  = yawMsg.data;
}

void OffboardControl::armModeCallback(const std_msgs::Bool::ConstPtr &msg) {
    arm_mode_ = *msg;
}

void OffboardControl::odomCallback(const nav_msgs::Odometry &odomMsg){
  current_odom_ = odomMsg;

  odomTime = odomMsg.header.stamp.toSec();
  geometry_msgs::Quaternion geoQuat = odomMsg.pose.pose.orientation;
  geometry_msgs::Vector3 velocity_base = odomMsg.twist.twist.linear;
 
  vehicleVBase = toEigen(velocity_base);
  Eigen::Quaterniond quat = toEigen(odomMsg.pose.pose.orientation);
  vehicleYaw = ToEulerYaw(quat);
//   ROS_INFO_STREAM("quat"<<quat.w()<<" "<<quat.x()<<" "<<quat.y()<<" "<<quat.z()<<" "<<euler(0)<<"yaw");
  vehicle_pos_(0) = odomMsg.pose.pose.position.x - cos(yaw) * sensorOffsetX + sin(yaw) * sensorOffsetY;
  vehicle_pos_(1) = odomMsg.pose.pose.position.y - sin(yaw) * sensorOffsetX - cos(yaw) * sensorOffsetY;
  vehicle_pos_(2) = odomMsg.pose.pose.position.z;
  odom_received_ = true;
}

void OffboardControl::targetPoseCallback(const offboard::PoseRequest &poseMsg) {
    target_pos_(0) = poseMsg.positionX;
    target_pos_(1) = poseMsg.positionY;
    target_yaw = atan2(target_pos_(1),target_pos_(0));
}

// Function to initialize ncurses and keyboard input
void OffboardControl::initNcurses() {
    initscr(); // Initialize ncurses
    cbreak();  // Line buffering disabled
    noecho();  // Don't echo any keypresses)
    keypad(stdscr, TRUE); // Enable keypad mode for arrow keys
}

// Function to cleanup ncurses
void OffboardControl::cleanupNcurses() {
    endwin(); // End ncurses
}

void OffboardControl::landing() {
    sendI2CMsg(0, 0, 0);
    operation_time_2 = ros::Time::now();
    std::printf("\n[ INFO] Operation time %.1f (s)\n\n", (operation_time_2 - operation_time_1).toSec());
    ros::shutdown();
}

void OffboardControl::sendI2CMsg(uint8_t throttle_pwm, uint8_t steering_pwm, uint8_t flags) {
    i2c_flag_ = flags & 0x01;
    uint16_t throttle_steering_pwm = steering_pwm << 8 | throttle_pwm << 0;
    wiringPiI2CWriteReg16(fd, flags, throttle_steering_pwm);
    // wiringPiI2CWriteReg8(fd, throttle_pwm, steering_pwm);
}

// Function to display PWM values
void OffboardControl::printPWM(int16_t throttle_val, int16_t steering_val) {
    clear(); // Clear the screen
    mvprintw(0, 0, "PWM Values:");
    mvprintw(1, 0, "Throttle: %d", throttle_val);
    mvprintw(2, 0, "Steering: %d", steering_val);
    refresh(); // Refresh the screen
}

void OffboardControl::yawTest() {
    ros::Rate loop_rate(10);
    // std::cout << "Input yaw: ";
    // std::cin >> target_yaw;
    while(ros::ok()) {
        yaw_error = target_yaw - vehicleYaw;
        if(yaw_error < -2*PI) {
            yaw_error += 2*PI;
        } else if (yaw_error > 2*PI) {
            yaw_error -= 2*PI;
        }
        steering_value = Kp_yaw*yaw_error + 127;
        if(steering_value < 0) {
            steering_value = 0;
        } else if(steering_value > 255) {
            steering_value = 255;
        }
        ROS_INFO_STREAM("target_yaw: "<<target_yaw << "\t yaw error: "<< yaw_error << "\t vehicle yaw" << vehicleYaw << "\t steering value: " << steering_value);
        sendI2CMsg(127, steering_value, 1);
    ros::spinOnce();
    loop_rate.sleep();
    }
}

void OffboardControl::pidTest() {
    ros::Rate loop_rate(10);
    while(ros::ok()) {
        target_error = calculateDistance(vehicle_pos_, target_pos_);
        yaw_error = target_yaw - vehicleYaw;
        // yaw
        if(yaw_error < -2*PI) {
            yaw_error += 2*PI;
        } else if (yaw_error > 2*PI) {
            yaw_error -= 2*PI;
        }
        steering_value = Kp_yaw*yaw_error + 127;
        if(steering_value < 0) {
            steering_value = 0;
        } else if(steering_value > 255) {
            steering_value = 255;
        }
        ROS_INFO_STREAM("target_yaw: "<<target_yaw << "\t yaw error: "<< yaw_error << "\t vehicle yaw" << vehicleYaw << "\t steering value: " << steering_value);
        // postion
        throttle_value = Kp_throt*target_error*std::cos(yaw_error) + 127;
        ROS_INFO_STREAM("target_error: "<<target_error << "\tthrottle value: " << throttle_value);
        sendI2CMsg(throttle_value, steering_value, 1);
    ros::spinOnce();
    loop_rate.sleep();
    }
}

double OffboardControl::calculateDistance(const Eigen::Vector3d& pose1, const Eigen::Vector3d& pose2) {
    // Calculate the difference between the two poses
    Eigen::Vector3d diff = pose2 - pose1;

    // Calculate the squared Euclidean distance
    double squared_distance = diff.squaredNorm();

    // Take the square root to get the actual Euclidean distance
    double distance = std::sqrt(squared_distance);

    return distance;
}