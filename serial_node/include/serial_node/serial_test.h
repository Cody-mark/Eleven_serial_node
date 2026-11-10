#ifndef __SERIAL_TEST_H_
#define __SERIAL_TEST_H_

#include <ros/ros.h>
#include <iostream>
#include <serial/serial.h>
#include <nav_msgs/Odometry.h>  // 修改包含的消息类型
#include <ros_mcu0.h>
#include <tf2/LinearMath/Matrix3x3.h>  // 新增四元数转换相关头文件
#include <tf2/LinearMath/Quaternion.h>

using namespace mcu0_serial;

class serial_test
{
private:
    ros::NodeHandle nh;
    ros::Timer timer;
    ros::Publisher puber;
    ros::Subscriber odom_sub;  // 修改订阅器名称和类型
    float msg_in[16] = {0};    // 根据实际需求调整数组大小
    float msg_out[13] = {0};   // 调整为Odometry核心数据量
    float test_num = 0;

    // 新增回调函数声明
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

public:
    serial_test();
    void loop();
    serial_mcu serial_test02;
};

#endif // __SERIAL_TEST_H_