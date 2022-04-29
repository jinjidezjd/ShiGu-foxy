/*将农田imu的参考坐标系从上右前坐标系转换到前左右坐标系
订阅/imu/data话题
发布/imu/data_enu话题
*/

#include "ros/ros.h"
#include "tf/tf.h"
#include "tf/transform_datatypes.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"

using namespace std;

static ros::Publisher enu_pub;

void callback(const sensor_msgs::ImuConstPtr &msg)
{
    //四元数转换到旋转矩阵
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->orientation, quat);  // geometry_msgs/Quaternion转换到tf::Quaternion
    tf::Matrix3x3 matrix;                           //旋转矩阵
    matrix.setRotation(quat);                       //四元数计算到旋转矩阵
    tf::Matrix3x3 temp(0, -1, 0, 1, 0, 0, 0, 0, 1); //沿z轴逆时针旋转90度的旋转矩阵
    tf::Matrix3x3 matrix2;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            matrix2[i][j] = temp[i][0] * matrix[0][j] + temp[i][1] * matrix[1][j] + temp[i][2] * matrix[2][j];
        }
    }
    tf::Quaternion quat2;
    matrix2.getRotation(quat2); //旋转矩阵计算到四元数
    geometry_msgs::Quaternion quat3;
    tf::quaternionTFToMsg(quat2, quat3); // tf::Quaternion转换到geometry_msgs/Quaternion

    //角速度坐标变换
    geometry_msgs::Vector3 angular_vel;
    angular_vel.x = msg->angular_velocity.z;
    angular_vel.y = -msg->angular_velocity.y;
    angular_vel.z = msg->angular_velocity.x;
    //加速度坐标变换
    geometry_msgs::Vector3 linear_accel;
    linear_accel.x = msg->linear_acceleration.z;
    linear_accel.y = -msg->linear_acceleration.y;
    linear_accel.z = msg->linear_acceleration.x;
    //封装Imu数据
    sensor_msgs::Imu imu_data; // imu数据
    imu_data.header.stamp = msg->header.stamp;
    imu_data.header.frame_id = msg->header.frame_id;
    imu_data.orientation = quat3;
    imu_data.orientation_covariance = msg->orientation_covariance;
    imu_data.angular_velocity = angular_vel;
    imu_data.angular_velocity_covariance = msg->angular_velocity_covariance;
    imu_data.linear_acceleration = linear_accel;
    imu_data.linear_acceleration_covariance = msg->linear_acceleration_covariance;
    //发布话题
    enu_pub.publish(imu_data);
}

int main(int argc, char **argv)
{
    //初始化节点
    ros::init(argc, argv, "imu2flu");
    //声明节点句柄
    ros::NodeHandle nh;
    //发布话题
    enu_pub = nh.advertise<sensor_msgs::Imu>("/imu/data_enu", 10);
    //订阅话题
    ros::Subscriber fix_sub = nh.subscribe("/imu/data", 1, callback);
    ros::spin();
    return 0;
}