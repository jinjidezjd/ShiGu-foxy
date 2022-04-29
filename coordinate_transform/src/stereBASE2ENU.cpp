/*将视觉里程计的参考坐标系从载体坐标系转换到东北天（ENU）坐标系
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

using namespace std;

static ros::Publisher enu_pub;
tf::Matrix3x3 matrix; //旋转矩阵

void callback(const nav_msgs::OdometryConstPtr &msg)
{
    //读取载体坐标系下的数据
    geometry_msgs::Point Point = msg->pose.pose.position;
    geometry_msgs::Quaternion Quaternion = msg->pose.pose.orientation;
    boost::array<double, 36> pose_cov = msg->pose.covariance;
    geometry_msgs::Vector3 twist_linear = msg->twist.twist.linear;
    geometry_msgs::Vector3 twist_angular = msg->twist.twist.angular;
    boost::array<double, 36> twist_cov = msg->twist.covariance;
    //旋转矩阵计算到四元数
    tf::Quaternion quat2;
    matrix.getRotation(quat2);
    //转换到eigen
    Eigen::Quaterniond quat_eigen(quat2.w(), quat2.x(), quat2.y(), quat2.z());                                       //旋转矩阵变换
    Eigen::Vector3d point_eigen(Point.x, Point.y, Point.z);                                                          //位置点转换
    tf::Quaternion Quaternion_tf;                                                                                    //旋转矩阵变换
    tf::quaternionMsgToTF(Quaternion, Quaternion_tf);                                                                // geometry_msgs/Quaternion转换到tf::Quaternion
    Eigen::Quaterniond Quaternion_eigen(Quaternion_tf.w(), Quaternion_tf.x(), Quaternion_tf.y(), Quaternion_tf.z()); //姿态转换
    Eigen::Vector3d twist_linear_eigen(twist_linear.x, twist_linear.y, twist_linear.z);                              //速度转换
    Eigen::Vector3d twist_angular_eigen(twist_angular.x, twist_angular.y, twist_angular.z);                          //角速度转换
    //在eigen下进行转换
    Eigen::Vector3d point_eigen2 = quat_eigen * point_eigen;                 //变换后的位置点
    Eigen::Quaterniond Quaternion_eigen2 = quat_eigen * Quaternion_eigen;    //变换后的姿态
    Eigen::Vector3d twist_linear_eigen2 = quat_eigen * twist_linear_eigen;   //变换后的线速度
    Eigen::Vector3d twist_angular_eigen2 = quat_eigen * twist_angular_eigen; //变换后的角速度
    //转换回tf
    geometry_msgs::Point Point_result;
    Point_result.x = point_eigen2[0];
    Point_result.y = point_eigen2[1];
    Point_result.z = point_eigen2[2];
    tf::Quaternion Quaternion_tf2(Quaternion_eigen2.x(), Quaternion_eigen2.y(), Quaternion_eigen2.z(), Quaternion_eigen2.w());
    geometry_msgs::Quaternion quat_result;
    tf::quaternionTFToMsg(Quaternion_tf2, quat_result); // tf::Quaternion转换到geometry_msgs/Quaternion
    geometry_msgs::Vector3 twist_linear_result;
    twist_linear_result.x = twist_linear_eigen2[0];
    twist_linear_result.y = twist_linear_eigen2[1];
    twist_linear_result.z = twist_linear_eigen2[2];
    geometry_msgs::Vector3 twist_angular_result;
    twist_angular_result.x = twist_angular_eigen2[0];
    twist_angular_result.y = twist_angular_eigen2[1];
    twist_angular_result.z = twist_angular_eigen2[2];
    //暂未将协方差矩阵进行变换
    boost::array<double, 36> covariance;
    //封装odom数据
    nav_msgs::Odometry odom_data; //要发布的enu坐标系下的odom数据
    odom_data.header.stamp = msg->header.stamp;
    odom_data.header.frame_id = msg->header.frame_id;
    odom_data.child_frame_id = msg->child_frame_id;
    odom_data.pose.pose.position = Point_result;
    odom_data.pose.pose.orientation = quat_result;
    odom_data.pose.covariance = pose_cov; //暂未变换
    odom_data.twist.twist.linear = twist_linear_result;
    odom_data.twist.twist.angular = twist_angular_result;
    odom_data.twist.covariance == twist_cov;
    // 非完整约束
    if (abs(twist_linear.y) <= 0.1 && abs(twist_linear.z) <= 0.1)
    {
        //发布话题
        enu_pub.publish(odom_data);
    }
    // //不使用非完整约束
    // //发布话题
    // enu_pub.publish(odom_data);
}

void callback_imu(const sensor_msgs::ImuConstPtr &msg)
{
    //四元数转换到旋转矩阵
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->orientation, quat); // geometry_msgs/Quaternion转换到tf::Quaternion
    matrix.setRotation(quat);                      //四元数计算到旋转矩阵
}

int main(int argc, char **argv)
{
    //初始化节点
    ros::init(argc, argv, "stereBASE2ENU");
    //声明节点句柄
    ros::NodeHandle nh;
    //发布话题
    enu_pub = nh.advertise<nav_msgs::Odometry>("/stereo_odometry_enu", 10);
    //订阅话题
    ros::Subscriber fix_sub = nh.subscribe("/stereo_odometry", 1, callback);
    //订阅话题
    ros::Subscriber imu_sub = nh.subscribe("/imu/initial_imu", 1, callback_imu);
    ros::spin();
    return 0;
}