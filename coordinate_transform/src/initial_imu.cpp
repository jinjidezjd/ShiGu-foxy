/*保存刚上电时的九轴imu数据
订阅/imu/data话题
发布/initial_imu数据
*/


#include "ros/ros.h"
#include "tf/tf.h"
#include "tf/transform_datatypes.h"
#include "sensor_msgs/Imu.h"

using namespace std;

static ros::Publisher enu_pub;
sensor_msgs::Imu imu_data;//要封装好的imu数据
double roll,pitch,yaw;//定义变量，分别为绕x，y，z轴旋转的变量

void callback(const sensor_msgs::ImuConstPtr& msg) {
    static int num=0;
    num++;
    if(num==1){
        //封装Imu数据
        imu_data.header.stamp=msg->header.stamp;
        imu_data.header.frame_id=msg->header.frame_id;
        imu_data.orientation=msg->orientation;
        imu_data.orientation_covariance=msg->orientation_covariance;
        imu_data.angular_velocity=msg->angular_velocity;
        imu_data.angular_velocity_covariance=msg->angular_velocity_covariance;
        imu_data.linear_acceleration=msg->linear_acceleration;
        imu_data.linear_acceleration_covariance=msg->linear_acceleration_covariance;

        //四元数转换到欧拉角
        tf::Quaternion quat;
        tf::quaternionMsgToTF(msg->orientation,quat);//geometry_msgs/Quaternion转换到tf::Quaternion
        tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);//转换到欧拉角,roll-x轴,pitch-y轴,yaw-z轴

    }
    //发布话题
    enu_pub.publish(imu_data);
    ROS_INFO("roll:%f,pitch:%f,yaw:%f\r\n",roll,pitch,yaw);
    
}

int main(int argc, char **argv) {
    //初始化节点
    ros::init(argc, argv, "initial_imu");
    //声明节点句柄
    ros::NodeHandle nh;
    //发布话题
    enu_pub = nh.advertise<sensor_msgs::Imu>("/imu/initial_imu",10);
    //订阅话题
    ros::Subscriber fix_sub = nh.subscribe("/imu/data_enu", 1, callback);
    ros::spin();
  return 0;
}