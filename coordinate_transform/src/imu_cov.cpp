/*为imu加入协方差矩阵
 */

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

using namespace std;
static ros::Publisher enu_pub;

void callback(const sensor_msgs::ImuConstPtr &msg)
{
    //自定义协方差矩阵
    boost::array<double, 9> covariance = {{10, 0, 0,
                                           0, 10, 0,
                                           0, 0, 10}};

    //封装Imu数据
    sensor_msgs::Imu imu_data;
    imu_data.header.stamp = msg->header.stamp;
    imu_data.header.frame_id = msg->header.frame_id;
    imu_data.orientation = msg->orientation;
    imu_data.orientation_covariance = covariance;
    imu_data.angular_velocity = msg->angular_velocity;
    imu_data.angular_velocity_covariance = covariance;
    imu_data.linear_acceleration = msg->linear_acceleration;
    imu_data.linear_acceleration_covariance = covariance;

    //发布话题
    enu_pub.publish(imu_data);
}

int main(int argc, char **argv)
{
    //初始化节点
    ros::init(argc, argv, "imu_cov");
    //声明节点句柄
    ros::NodeHandle nh;
    //发布话题
    enu_pub = nh.advertise<sensor_msgs::Imu>("/imu/data_cov", 10);
    //订阅话题
    ros::Subscriber fix_sub = nh.subscribe("/imu/data", 1, callback);
    ros::spin();
    return 0;
}