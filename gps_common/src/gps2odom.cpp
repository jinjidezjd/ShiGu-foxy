#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include <gps_common/conversions.h>
// #include "nav_msgs/msg/Odometry.hpp"
#include <nav_msgs/msg/odometry.hpp>

using namespace gps_common;
using namespace std;

rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;

std::string frame_id = "odom";           //父帧
std::string child_frame_id = "gps_Link"; //子帧，仿真中
// std::string child_frame_id = "imu_link"; //子帧，kitti数据集
double rot_cov = 99999.0; //协方差，表明获取到的角度不可信
bool append_zone = false;
double northing_start, easting_start, altitude_start; //起点的坐标，作为基准

void callback(const sensor_msgs::msg::NavSatFix::SharedPtr fix) //回调函数
{
  if (fix->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "The GPS Signal is Not Valid!");
    return;
  }
  // if (fix->header.stamp == ros::Time(0))
  // {
  //   return;
  // }
  static int is_start = 0;

  double northing, easting;
  std::string zone;

  LLtoUTM(fix->latitude, fix->longitude, northing, easting, zone);
  //获取起点坐标
  if (is_start == 0)
  {
    is_start = 1;
    northing_start = northing;
    easting_start = easting;
    altitude_start = fix->altitude;
  }

  if (odom_pub)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = fix->header.stamp;
    odom.header.frame_id = frame_id;
    odom.child_frame_id = child_frame_id;
    //自己修改
    // odom.pose.pose.position.x = northing-northing_start;
    // odom.pose.pose.position.y = -(easting-easting_start);
    //原始
    odom.pose.pose.position.x = (easting - easting_start);
    odom.pose.pose.position.y = (northing - northing_start);

    odom.pose.pose.position.z = fix->altitude - altitude_start;

    odom.pose.pose.orientation.x = 0;
    odom.pose.pose.orientation.y = 0;
    odom.pose.pose.orientation.z = 0;
    odom.pose.pose.orientation.w = 1;

    // Use ENU covariance to build XYZRPY covariance
    for (size_t i = 0; i < 6; i++)
    {
      for (size_t j = 0; j < 6; j++)
      {
        if (i == j)
        {
          odom.pose.covariance[6 * i + j] = rot_cov;
        }
        else
        {
          odom.pose.covariance[6 * i + j] = 0.0;
        }
      }
    }
    odom.pose.covariance[0] = fix->position_covariance[0];
    odom.pose.covariance[0] = fix->position_covariance[1];
    odom.pose.covariance[0] = fix->position_covariance[2];
    odom.pose.covariance[6] = fix->position_covariance[3];
    odom.pose.covariance[7] = fix->position_covariance[4];
    odom.pose.covariance[8] = fix->position_covariance[5];
    odom.pose.covariance[12] = fix->position_covariance[6];
    odom.pose.covariance[13] = fix->position_covariance[7];
    odom.pose.covariance[14] = fix->position_covariance[8];

    odom_pub->publish(odom);
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("gps2odom");

  odom_pub = node->create_publisher<nav_msgs::msg::Odometry>("/gps/odom", 10);
  auto fix_sub = node->create_subscription<sensor_msgs::msg::NavSatFix>("/navsat/fix", 10, callback);

  rclcpp::spin(node);
}
