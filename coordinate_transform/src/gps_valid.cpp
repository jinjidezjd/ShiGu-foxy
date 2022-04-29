/*将/gps/odom数据转换为/gps/odom_valid，通过是否运行本节点，人为实现GPS信号的通断
订阅/gps/odom话题
发布/gps/odom_valid话题
*/

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub;

void callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) //回调函数
{
    gps_pub->publish(*msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("gps_valid");

    gps_pub = node->create_publisher<sensor_msgs::msg::NavSatFix>("/navsat/fix_valid", 10);
    auto fix_sub = node->create_subscription<sensor_msgs::msg::NavSatFix>("/navsat/fix", 10, callback);

    rclcpp::spin(node);
}
