/**
 * 该订阅/cmd_vel话题，转换成六轮转速和四轮转向控制，从串口发送至下位机，接收下位机消息并发布出去
 */
#include <iostream>
#include <string>
#include <sstream>
#include <geometry_msgs/msg/twist.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "serial/serial.h"
// #include "base_controller/my_msg.h"
using namespace std;

#define RADIUS 0.06        //车轮半径
#define TURNING_RADIUS 0.4 //车轮与小车中心的回转半径
#define T 0.58             //左右两侧轮距
#define L 0.614            //前轮与后轮的轴距
#define L1 0.338           //前轮与中轮的轴距
#define L2 0.281           //中轮与后轮的轴距
#define MaxSteering 0.785  //最大转弯弧度为pi/4rad

#define sBUFFER_SIZE 1024                            //发送缓存长度
#define rBUFFER_SIZE 1024                            //接收缓存长度
unsigned char s_buffer[sBUFFER_SIZE] = {0};          //要发送的数据
unsigned char r_buffer[rBUFFER_SIZE] = {0};          //要接收的数据
string rec_buffer;                                   //串口数据接收变量
float x_velocity = 0, y_velocity = 0, z_angular = 0; //暂存的x,y方向线速度，z方向角速度
float wheel_velocity = 0;                            //车轮的角速度
float rMax;                                          //最大转弯半径

serial::Serial my_serial;

//反馈回的位置、速度、电流指令
union feedback // union的作用为实现char数组和float之间的转换
{
    float d;
    unsigned char data[4];
} feedback_position, feedback_velocity, feedback_current;

//发送给下位机速度指令,包括六轮转速和四轮转向
union floatData // union的作用为实现char数组和float之间的转换
{
    float d;
    unsigned char data[4];
} rotate_LF, rotate_LM, rotate_LR, rotate_RF, rotate_RM, rotate_RR, steer_LF, steer_LR, steer_RF, steer_RR;

void callback(const geometry_msgs::msg::Twist::SharedPtr msg) //订阅/cmd_vel主题回调函数
{
    x_velocity = msg->linear.x; //获取/cmd_vel的x方向速度信息
    z_angular = msg->angular.z; //获取/cmd_vel的z方向角度信息
    // ROS_INFO("Publish Control Info: x_velocity:%f  z_angular:%f", x_velocity, z_angular);
    std::cout << "x_velocity : " << x_velocity << ", z_angular : " << z_angular << std::endl;
    //将小车的速度转化为每个轮子的转速和转向
    if (z_angular != 0)
    {
        if (x_velocity != 0)
        { /*小车进行前进转向*/
            if (z_angular > MaxSteering)
            {
                z_angular = MaxSteering;
            } //限制转角
            if (z_angular < -MaxSteering)
            {
                z_angular = -MaxSteering;
            } //限制转角
            steer_RF.d = -atan(L1 / (L1 / tan(z_angular) - T / 2));
            steer_RR.d = atan(L1 / (L2 / tan(z_angular) - T / 2));
            steer_LF.d = -atan(L1 / (L1 / tan(z_angular) + T / 2));
            steer_LR.d = atan(L1 / (L2 / tan(z_angular) + T / 2));
            wheel_velocity = x_velocity / RADIUS; //转化为车轮质心角速度
            rotate_LF.d = wheel_velocity * sin(z_angular) / sin(-steer_LF.d);
            rotate_LM.d = wheel_velocity;
            rotate_LR.d = wheel_velocity * sin(z_angular) / sin(steer_LR.d);
            rotate_RF.d = wheel_velocity * sin(z_angular) / sin(-steer_RF.d);
            rotate_RM.d = wheel_velocity;
            rotate_RR.d = wheel_velocity * sin(z_angular) / sin(steer_RR.d);
        }
        else if (x_velocity == 0)
        { /*小车进行纯转向*/
            //左前、右后轮顺时针旋转1rad
            steer_LF.d = 1.0;
            steer_RR.d = 1.0;
            //左后、右前轮逆时针旋转1rad
            steer_LR.d = -1.0;
            steer_RF.d = -1.0;
            //左侧车轮速度：小车整体旋转角速度*车轮回转半径/车轮半径
            rotate_LF.d = rotate_LM.d = rotate_LR.d = -z_angular * TURNING_RADIUS / RADIUS;
            //右侧车轮相反
            rotate_RF.d = rotate_RM.d = rotate_RR.d = z_angular * TURNING_RADIUS / RADIUS;
        }
    }
    else if (z_angular == 0)
    {                                                                                                            /*小车进行直线运动*/
        rotate_LF.d = rotate_LM.d = rotate_LR.d = rotate_RF.d = rotate_RM.d = rotate_RR.d = x_velocity / RADIUS; //如何将小车速度转化为车轮转速，动力学分析！！
        steer_LF.d = steer_LR.d = steer_RF.d = steer_RR.d = 0;
    }

    //数据存入至发送数据缓存中
    for (int i = 0; i < 4; i++) //将左右轮速度存入数组中发送给串口
    {
        s_buffer[i] = rotate_LF.data[i];
        s_buffer[i + 4] = rotate_LM.data[i];
        s_buffer[i + 8] = rotate_LR.data[i];
        s_buffer[i + 12] = rotate_RF.data[i];
        s_buffer[i + 16] = rotate_RM.data[i];
        s_buffer[i + 20] = rotate_RR.data[i];
        s_buffer[i + 24] = steer_LF.data[i];
        s_buffer[i + 28] = steer_LR.data[i];
        s_buffer[i + 32] = steer_RF.data[i];
        s_buffer[i + 36] = steer_RR.data[i];
    }

    //在写入串口的数据后加入"\r\n"
    s_buffer[40] = 0x0d;
    s_buffer[41] = 0x0a;
    //写入数据到串口
    my_serial.write(s_buffer, 42);
}

int main(int argc, char **argv)
{
    // ros::init(argc, argv, "base_controller");
    // ros::NodeHandle nh;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("base_controller");
    // 打开串口
    try
    {
        my_serial.setPort("/dev/ttyUSB1"); // 这个端口号就是用cutecom看到的端口名称
        my_serial.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        my_serial.setTimeout(to);
        my_serial.open();
    }
    catch (serial::IOException &e)
    {
        RCLCPP_INFO(node->get_logger(), "Failed to open port ");
        return -1;
    }
    RCLCPP_INFO(node->get_logger(), "Succeed to open port ");
    auto motor_sub = node->create_subscription<std_msgs::msg::String>("cmd_vel", 10, callback);
    rclcpp::Rate loop_rate(100); //设置频率
    while (rclcpp::ok())
    {

        if (my_serial.available())
        {
            rec_buffer = my_serial.readline(25, "\n");    //获取串口发送来的数据
            const char *receive_data = rec_buffer.data(); //保存串口发送来的数据
            if (rec_buffer.length() == 13)                //串口接收的数据长度正确就处理并发布反馈数据消息
            {
                // ROS_INFO("serial info has been received!\r\n");
                for (int i = 0; i < 4; i++)
                {
                    feedback_position.data[i] = receive_data[i];
                    feedback_velocity.data[i] = receive_data[i + 4];
                    feedback_current.data[i] = receive_data[i + 8];
                }
            }
        }

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    // 关闭串口
    my_serial.close();
    return 0;
}