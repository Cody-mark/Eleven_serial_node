#include <ros/ros.h>
#include <serial_node/ros_mcu0.h>
#include <geometry_msgs/PoseStamped.h>
#include <vision_msgs/BoundingBox2DArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <std_msgs/Bool.h>
#include <cmath>

using namespace mcu0_serial;

serial_mcu* serialComm;

// 全局变量存储图像尺寸
int image_width = 1440;
int image_height = 1080;

float initial_x = 0.0f;
float initial_y = 0.0f;
float initial_yaw = 0.0f;
bool has_initial = false;

// 添加姿态数据超时检测
ros::Time last_pose_time;
const double POSE_TIMEOUT = 0.5; // 0.5秒超时

// 姿态数据回调
void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // 更新最后接收时间
    last_pose_time = ros::Time::now();

    // 原始坐标转换
    float x_centered = msg->pose.position.x;
    float y_centered = msg->pose.position.y;

    // 四元数转偏航角
    tf2::Quaternion q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw_rad;
    m.getRPY(roll, pitch, yaw_rad);
    float yaw_deg = static_cast<float>(yaw_rad * 180.0 / M_PI);

    // 应用初始值
    if (has_initial) {
        x_centered += initial_x;
        y_centered += initial_y;
        yaw_deg += initial_yaw;
    }

    // 发送数据
    float send_data[5] = {x_centered, y_centered, yaw_deg , 1 , 1};
    serialComm->serial_send(2, send_data, 5);
}

// 边界框数组回调
void bboxCallback(const vision_msgs::BoundingBox2DArray::ConstPtr& msg)
{
    float image_center_x = image_width / 2.0f;
    float image_center_y = image_height / 2.0f;

    // 遍历所有检测到的边界框
    for (const auto& bbox : msg->boxes) {
        float x_centered = bbox.center.x - image_center_x;
        float y_centered = image_center_y - bbox.center.y;

        float send[2] = {
            x_centered,     // 中心坐标系X坐标
            y_centered,     // 中心坐标系Y坐标
        };
        serialComm->serial_send(1, send, 2);
    }
}

// 计算两点之间的欧氏距离
float calculateDistance(float x1, float y1, float x2, float y2) {
    float dx = x2 - x1;
    float dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mcu0_serial_node");
    ros::NodeHandle nh;

    // 获取图像尺寸参数
    nh.param<int>("image_width", image_width, 1440);
    nh.param<int>("image_height", image_height, 1080);

    // 初始化串口
    std::string port;
    nh.param<std::string>("serial_port", port, "/dev/ttyACM0");
    try {
        serialComm = new serial_mcu(port);
        ROS_INFO("Serial port initialized successfully");
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to initialize serial port: %s", e.what());
        return -1;
    }

    // 初始化最后姿态接收时间
    last_pose_time = ros::Time::now();

    // 创建发布者和订阅者
    ros::Subscriber pose_sub = nh.subscribe("/pose_stamped", 20, poseCallback);
    ros::Subscriber bbox_sub = nh.subscribe("/yolo/detections", 20, bboxCallback);
    ros::Publisher restart_pub = nh.advertise<std_msgs::Bool>("/restart_slam", 1);
    ros::Publisher status_pub = nh.advertise<std_msgs::Bool>("/send_status", 10);

    ros::Rate loop_rate(50);
    while (ros::ok()) {
        ros::spinOnce();

        // 持续发布串口状态
        std_msgs::Bool status_msg;
        status_msg.data = serialComm->isOpen();
        status_pub.publish(status_msg);

        // 处理串口接收
        uint8_t frame_id;
        float received_data[32];
        uint8_t data_length;
        if (serialComm->serial_read(&frame_id, received_data, &data_length)) {
            if (frame_id == 1 && data_length >= 3) {
                // 处理初始值设置
                initial_x = received_data[0];
                initial_y = received_data[1];
                initial_yaw = received_data[2];
                has_initial = true;

                // 发布重启信号
                std_msgs::Bool restart_msg;
                restart_msg.data = true;
                restart_pub.publish(restart_msg);

                ROS_INFO("Initial values set: X=%.4f, Y=%.4f, Yaw=%.4f",
                        initial_x, initial_y, initial_yaw);

                // 发送frame_id=6的确认数据包
                float ack_data[1] = {1.0f};  // 发送确认值1.0
                serialComm->serial_send(3, ack_data, 1);
                ROS_INFO("Sent acknowledgment packet with frame_id=6");
            }
            else if (frame_id == 4 && data_length >= 4) {
                // 处理两点坐标数据
                float x1 = received_data[0];
                float y1 = received_data[1];
                float x2 = received_data[2];
                float y2 = received_data[3];

                // 计算两点之间的距离
                float distance = calculateDistance(x1, y1, x2, y2);

                // 将距离发送回串口 (使用frame_id = 5)
                float distance_data[1] = {distance};
                serialComm->serial_send(5, distance_data, 1);

                ROS_INFO("Received points: P1(%.4f, %.4f), P2(%.4f, %.4f), Distance: %.4f",
                        x1, y1, x2, y2, distance);
            }
        }

        // 检查姿态数据是否超时
        if ((ros::Time::now() - last_pose_time).toSec() > POSE_TIMEOUT) {
            // 发送ID=3的超时数据 (5个0值)
            float timeout_data[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
            serialComm->serial_send(6, timeout_data, 5);
        }

        loop_rate.sleep();
    }

    delete serialComm;
    return 0;
}