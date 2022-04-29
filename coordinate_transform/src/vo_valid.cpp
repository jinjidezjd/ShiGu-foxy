/*将/stereo_odometry数据转换为/stereo_odometry_valid，通过是否运行本节点，人为实现VO信号的通断
订阅/stereo_odometry话题
发布/stereo_odometry_valid话题
*/

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

static ros::Publisher gps_pub;

void callback(const nav_msgs::OdometryConstPtr &msg)
{
    gps_pub.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vo_valid");
    ros::NodeHandle node;
    gps_pub = node.advertise<nav_msgs::Odometry>("/stereo_odometry_valid", 10);
    ros::Subscriber fix_sub = node.subscribe("/stereo_odometry", 10, callback);
    ros::spin();
}
