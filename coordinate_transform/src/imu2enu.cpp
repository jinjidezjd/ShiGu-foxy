/*将kitti imu的参考坐标系从载体坐标系转换到enu坐标系
订阅/imu/data话题
发布/imu/data_enu话题
*/

#include "ros/ros.h"
#include "tf/tf.h"
#include "tf/transform_datatypes.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Dense>

using namespace std;

static ros::Publisher enu_pub;
Eigen::Quaterniond quat_eigen;
static int num = 0;
void callback(const sensor_msgs::ImuConstPtr &msg)
{
    if (num == 0)
    {
        //四元数转换到旋转矩阵
        tf::Quaternion quat;
        tf::quaternionMsgToTF(msg->orientation, quat); // geometry_msgs/Quaternion转换到tf::Quaternion
        //转换到eigen
        Eigen::Quaterniond quat_eigen2(quat.w(), quat.x(), quat.y(), quat.z());
        quat_eigen = quat_eigen.inverse();
        num = 1;
    }
    //角速度变换
    Eigen::Vector3d twist_angular_eigen(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    //线加速度变换
    Eigen::Vector3d linear_eigen(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    //在eigen下进行转换
    Eigen::Vector3d twist_angular_eigen2 = quat_eigen * twist_angular_eigen; //变换后的角速度
    Eigen::Vector3d linear_eigen2 = quat_eigen * linear_eigen;               //变换后的线加速度

    //转换回geometry
    geometry_msgs::Vector3 twist_angular_result;
    twist_angular_result.x = twist_angular_eigen2[0];
    twist_angular_result.y = twist_angular_eigen2[1];
    twist_angular_result.z = twist_angular_eigen2[2];
    geometry_msgs::Vector3 linear_result;
    linear_result.x = linear_eigen2[0];
    linear_result.y = linear_eigen2[1];
    linear_result.z = linear_eigen2[2];

    //封装Imu数据
    sensor_msgs::Imu imu_data; // imu数据
    imu_data.header.stamp = msg->header.stamp;
    imu_data.header.frame_id = msg->header.frame_id;
    imu_data.orientation = msg->orientation;
    imu_data.orientation_covariance = msg->orientation_covariance;
    imu_data.angular_velocity = twist_angular_result;
    imu_data.angular_velocity_covariance = msg->angular_velocity_covariance;
    imu_data.linear_acceleration = linear_result;
    imu_data.linear_acceleration_covariance = msg->linear_acceleration_covariance;
    //发布话题
    enu_pub.publish(imu_data);
}

int main(int argc, char **argv)
{
    //初始化节点
    ros::init(argc, argv, "imu2enu");
    //声明节点句柄
    ros::NodeHandle nh;
    //发布话题
    enu_pub = nh.advertise<sensor_msgs::Imu>("/imu/data_enu", 10);
    //订阅话题
    ros::Subscriber fix_sub = nh.subscribe("/imu/data", 1, callback);
    ros::spin();
    return 0;
}