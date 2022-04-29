#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "serial/serial.h" //ROS已经内置了的串口包
#include "std_msgs/msg/string.hpp"
// #include "std_msgs/msg/Empty.hpp"
#include <string>
#include <vector>
#include <sstream>
#include <cmath>
#include <cstdlib> //string转化为double
#include <iomanip> //保留有效小数
#include "sensor_msgs/msg/nav_sat_fix.hpp"
serial::Serial ser; //声明串口对象

//解析GPS
void RecePro(std::string s, double &lat, double &lon, double &altitude)
{
    //分割有效数据，存入vector中
    std::vector<std::string> v;
    std::string::size_type pos1, pos2;
    pos2 = s.find(",");
    pos1 = 0;
    while (std::string::npos != pos2)
    {
        v.push_back(s.substr(pos1, pos2 - pos1));
        pos1 = pos2 + 1;
        pos2 = s.find(",", pos1);
    }
    if (pos1 != s.length())
        v.push_back(s.substr(pos1));
    //解析出经纬度
    if (v.max_size() >= 6 && (v[6] == "1" || v[6] == "2" || v[6] == "3" || v[6] == "4" || v[6] == "5" || v[6] == "6" || v[6] == "8" || v[6] == "9"))
    {
        //纬度
        if (v[2] != "")
            lat = std::atof(v[2].c_str()) / 100;
        int ilat = (int)floor(lat) % 100;
        lat = ilat + (lat - ilat) * 100 / 60;
        //经度
        if (v[4] != "")
            lon = std::atof(v[4].c_str()) / 100;
        int ilon = (int)floor(lon) % 1000;
        lon = ilon + (lon - ilon) * 100 / 60;
        //经度
        if (v[9] != "")
            altitude = std::atof(v[9].c_str());
    }
}
int main(int argc, char **argv)
{
    //初始化节点
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("serial_gps_node");
    //注册Publisher到话题GPS
    auto GPS_pub = node->create_publisher<sensor_msgs::msg::NavSatFix>("/navsat/fixed", 10);
    try
    {
        //串口设置
        ser.setPort("/dev/ttyUSB1");
        ser.setBaudrate(38400);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException &e)
    {
        RCLCPP_INFO(node->get_logger(), "Failed to open gps port ");
        return -1;
    }
    if (ser.isOpen())
    {
        RCLCPP_INFO(node->get_logger(), "Serial Port initialized ");
    }
    else
    {
        return -1;
    }

    rclcpp::Rate loop_rate(50); //设置频率

    std::string strRece;
    while (rclcpp::ok())
    {

        if (ser.available())
        {
            // 1.读取串口信息：
            // ROS_INFO_STREAM("Reading from serial port\n");
            //通过ROS串口对象读取串口信息
            // std::cout << ser.available();
            // std::cout << ser.read(ser.available());
            strRece += ser.read(ser.available());
            // std::cout <<"strRece:" << strRece << "\n" ;
            // strRece = "$GNGGA,122020.70,3908.17943107,N,11715.45190423,E,1,17,1.5,19.497,M,-8.620,M,,*54\r\n";
            // 2.截取数据、解析数据：
            // GPS起始标志
            std::string gstart = "$GN";
            // GPS终止标志
            std::string gend = "\r\n";
            int i = 0, start = -1, end = -1;
            while (i < int(strRece.size()))
            {
                //找起始标志

                start = strRece.find(gstart);
                //如果没找到，丢弃这部分数据，但要留下最后2位,避免遗漏掉起始标志
                if (start == -1)
                {
                    if (strRece.length() > 2)
                        strRece = strRece.substr(strRece.length() - 3);
                    break;
                }
                //如果找到了起始标志，开始找终止标志
                else
                {
                    //找终止标志
                    end = strRece.find(gend);
                    //如果没找到，把起始标志开始的数据留下，前面的数据丢弃，然后跳出循环
                    if (end == -1 || end < start)
                    {
                        if (end != 0)
                            strRece = strRece.substr(start);
                        break;
                    }
                    //如果找到了终止标志，把这段有效的数据剪切给解析的函数，剩下的继续开始寻找
                    else
                    {
                        i = end;

                        //把有效的数据给解析的函数以获取经纬度
                        double lat, lon, altitude;
                        RecePro(strRece.substr(start, end + 2 - start), lat, lon, altitude);
                        // std::cout << std::setiosflags(std::ios::fixed)<<std::setprecision(7)<< "纬度：" << lat << " 经度："<< lon << "\n";
                        //发布消息到话题
                        sensor_msgs::msg::NavSatFix GPS_data;
                        GPS_data.header.stamp = rclcpp::Clock().now();
                        GPS_data.header.frame_id = "base_link";
                        GPS_data.latitude = lat;
                        GPS_data.longitude = lon;
                        GPS_data.altitude = altitude;
                        //协方差矩阵 float64[9]数组
                        // boost::array<double, 9> covariance = {{
                        //     0.0004,
                        //     0.0,
                        //     0.0,
                        //     0.0,
                        //     0.0004,
                        //     0.0,
                        //     0.0,
                        //     0.0,
                        //     0.0004,
                        // }};
                        for (size_t i = 0; i < 3; i++)
                        {
                            for (size_t j = 0; j < 3; j++)
                            {
                                if (i == j)
                                {
                                    GPS_data.position_covariance[3 * i + j] = 0.0004;
                                }
                                else
                                {
                                    GPS_data.position_covariance[3 * i + j] = 0.0;
                                }
                            }
                        }
                        // GPS_data.position_covariance = covariance;

                        //加入数据校验，如果经纬度均为0，说明gps未初始化完成
                        if (lat == 0 || lon == 0)
                        {
                            GPS_data.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX; //未获取到数据
                        }
                        else
                        {
                            GPS_data.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX; //获取到数据
                        }
                        GPS_pub->publish(GPS_data);
                        //如果剩下的字符大于等于4，则继续循环寻找有效数据,如果所剩字符小于等于3则跳出循环
                        if (i + 5 < int(strRece.size()))
                            strRece = strRece.substr(end + 2);
                        else
                        {
                            strRece = strRece.substr(end + 2);
                            break;
                        }
                    }
                }
            }
        }
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
}
