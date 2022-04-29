/*用于kitti数据集,将视觉里程计的默认参考里程计为camera_color_left（坐标系为右下前），需要将其转换到前左上,并且叠加上到imu_link的平移向量
订阅/stereo_odometry话题,/imu/initial_imu话题
发布/stereo_odometry_enu话题
*/

#include "ros/ros.h"
#include "tf/tf.h"
#include "tf/transform_datatypes.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3.h"
#include "nav_msgs/Odometry.h"
#include <std_msgs/Float64.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Dense>
#include <algorithm>
#include <tf/transform_listener.h>
#include "tf/LinearMath/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include <eigen_conversions/eigen_msg.h>
using namespace std;

static ros::Publisher enu_pub;

void callback(const nav_msgs::OdometryConstPtr &msg)
{
    //读取相机坐标系下的数据
    geometry_msgs::Point Point = msg->pose.pose.position;
    geometry_msgs::Quaternion Quaternion = msg->pose.pose.orientation;
    boost::array<double, 36> pose_cov = msg->pose.covariance;
    geometry_msgs::Vector3 twist_linear = msg->twist.twist.linear;
    geometry_msgs::Vector3 twist_angular = msg->twist.twist.angular;
    boost::array<double, 36> twist_cov = msg->twist.covariance;

    //通过命令行获取
    tf::StampedTransform transform;
    tf::Transform t;
    // t.setOrigin(tf::Vector3(-1.083, 0.248, -0.730));
    geometry_msgs::Quaternion r;
    r = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0); //绕x逆时针旋转90度
    Eigen::Quaterniond e_q;                               //旋转四元数
    tf::quaternionMsgToEigen(r, e_q);
    t.setOrigin(tf::Vector3(-1.083, 0.248, -0.730));
    t.setRotation(tf::Quaternion(0, 0, 0, 0));
    transform.setData(t);
    // tf::StampedTransform转换到geometry_msgs::TransformStamped
    geometry_msgs::TransformStamped geo_transform;
    tf::transformStampedTFToMsg(transform, geo_transform);
    Eigen::Quaterniond eigen_rotation; //旋转四元数
    tf::quaternionMsgToEigen(geo_transform.transform.rotation, eigen_rotation);
    eigen_rotation = e_q * eigen_rotation;
    Eigen::Vector3d eigen_vector; //平移向量
    tf::vectorMsgToEigen(geo_transform.transform.translation, eigen_vector);
    // 进行旋转平移
    //位置
    Eigen::Vector3d eigen_point;
    tf::pointMsgToEigen(Point, eigen_point);
    eigen_point = eigen_point + eigen_vector;
    geometry_msgs::Point Point_result;
    tf::pointEigenToMsg(eigen_point, Point_result);
    //姿态
    Eigen::Quaterniond eigen_q;
    tf::quaternionMsgToEigen(Quaternion, eigen_q);
    eigen_q = eigen_q;
    geometry_msgs::Quaternion quat_result;
    tf::quaternionEigenToMsg(eigen_q, quat_result);
    //线速度
    Eigen::Vector3d eigen_velocity;
    tf::vectorMsgToEigen(twist_linear, eigen_velocity);
    eigen_velocity = eigen_velocity;
    geometry_msgs::Vector3 velocity_result;
    tf::vectorEigenToMsg(eigen_velocity, velocity_result);
    //角速度
    Eigen::Vector3d eigen_angular_velocity;
    tf::vectorMsgToEigen(twist_angular, eigen_angular_velocity);
    eigen_angular_velocity = eigen_angular_velocity;
    geometry_msgs::Vector3 angular_velocityt_result;
    tf::vectorEigenToMsg(eigen_angular_velocity, angular_velocityt_result);
    //暂未将协方差矩阵进行变换
    boost::array<double, 36>
        covariance;
    //封装odom数据
    nav_msgs::Odometry odom_data; //要发布的enu坐标系下的odom数据
    odom_data.header.stamp = msg->header.stamp;
    odom_data.header.frame_id = msg->header.frame_id;
    odom_data.child_frame_id = msg->child_frame_id;
    odom_data.pose.pose.position = Point_result;
    odom_data.pose.pose.orientation = quat_result;
    odom_data.pose.covariance = pose_cov; //暂未变换
    odom_data.twist.twist.linear = velocity_result;
    odom_data.twist.twist.angular = angular_velocityt_result;
    odom_data.twist.covariance == twist_cov;
    //发布话题
    enu_pub.publish(odom_data);
}

int main(int argc, char **argv)
{
    //初始化节点
    ros::init(argc, argv, "stereo_enu2base");
    //声明节点句柄
    ros::NodeHandle nh;
    //发布话题
    enu_pub = nh.advertise<nav_msgs::Odometry>("/stereo_odometry_enu_base", 10);
    //订阅话题
    ros::Subscriber fix_sub = nh.subscribe("/stereo_odometry_enu", 1, callback);
    // tf::TransformListener listener;
    ros::spin();
    return 0;
}