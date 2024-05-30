#include <ros/ros.h>
#include <std_msgs/Bool.h>

std_msgs::Bool arm_mode;

int main(int argc, char **argv)
{
    arm_mode.data = false;
    ros::init(argc, argv, "setmode_offb");
    ros::NodeHandle nh;
    ros::Rate rate(10);

    ros::Publisher arm_mode_pub = nh.advertise<std_msgs::Bool>("arm_mode",1);
    std::printf("\n[ INFO]Vehicle armed\n");
    while(ros::ok()) {
        rate.sleep();
        arm_mode.data = true;
        arm_mode_pub.publish(arm_mode);
        ros::spinOnce();
    }

    return 0;
}