#pragma once

#include <rclcpp/rclcpp.hpp>
#include <serial_driver/serial_driver.hpp>
#include <string>
#include <cstddef>
#include <memory>
#include <mutex>
#include <vector>

using namespace std;

namespace Eleven
{
#define FRAME_HEAD_0 0xFF
#define FRAME_HEAD_1 0xFE
#define FRAME_END_0 0xFD
#define FRAME_END_1 0xFB
#define MAX_PAYLOAD_LEN 64      // 字节数
#define MAX_DATA_LENGTH (MAX_PAYLOAD_LEN / sizeof(float))

uint16_t CRC16_Modbus(const uint8_t *p, size_t len);

typedef struct serial_frame
{
    uint8_t data_length;
    uint8_t frame_head[2];
    uint8_t frame_id;
    uint16_t crc_calculated;
    union data
    {
        float msg_get[MAX_DATA_LENGTH];
        uint8_t buff_msg[MAX_PAYLOAD_LEN];
    } data;
    union check_code
    {
        uint16_t crc_code;
        uint8_t crc_buff[2];
    } check_code;
    uint8_t frame_end[2];
} msg_frame;

class serial_mcu
{
public:
    serial_mcu(const rclcpp::Node::SharedPtr& node, const std::string& port);
    ~serial_mcu();
    size_t serial_send(uint8_t frame_id, float msgs[], uint8_t length);
    bool serial_read(uint8_t* received_frame_id, float msgs[], uint8_t* received_length);
    bool isOpen() const;
    std::string serial_port_;

private:
    msg_frame framein_, frameout_;
    rclcpp::Node::SharedPtr node_;

    // serial
    drivers::common::IoContext io_context_{1};          // 创建 I/O上下文对象
    std::unique_ptr<drivers::serial_driver::SerialPort> port_;
    std::mutex rx_mutex_;
    std::vector<uint8_t> rx_buffer_;

    int serial_baud_ = 115200;
    double dt = 0, speed_dt = 0;
    double last_update_time = 0, last_speed_update_time = 0;
    double update_time = 0.02;
    int test = 0;

    uint8_t send_length = 0;
    double last_x = 0;
    double last_y = 0;
};

}
