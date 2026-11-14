#pragma once

#include <rclcpp/rclcpp.hpp>
#include <serial_driver/serial_driver.hpp>

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
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

const size_t FRAME_OVERHEAD = 8;    // head(2) + id(1) + len(1) + crc(2) + tail(2)
const size_t MAX_FRAME_LEN = MAX_PAYLOAD_LEN + FRAME_OVERHEAD;

uint16_t CRC16_Modbus(const uint8_t *p, size_t len);

struct received_frame
{
    uint8_t id = 0;
    uint8_t len = 0;        // 有效数据字节数
    std::array<uint8_t, MAX_PAYLOAD_LEN> data{};

    // index 以 4 字节为一个单位；3 个 float + 2 个 uint32 时依次为 0,1,2 和 3,4
    float as_float(size_t index) const;
    uint32_t as_u32(size_t index) const;
};

class serial_mcu
{
public:
    serial_mcu(const rclcpp::Node::SharedPtr& node, const std::string& port);
    ~serial_mcu();

    serial_mcu(const serial_mcu&) = delete;
    serial_mcu& operator=(const serial_mcu&) = delete;

    // 非阻塞入队：队列满时丢弃最旧一帧，本帧仍会入队并返回 true。
    bool serial_send(uint8_t frame_id, const void* data, uint8_t data_len);
    bool serial_send(uint8_t frame_id, const float* msgs, uint8_t count);

    // 非阻塞取出一个已解析完成的帧。
    bool serial_read(received_frame& out);

    bool isOpen() const { return connected_.load(); }
    uint64_t tx_sent() const { return tx_sent_.load(); }
    uint64_t tx_dropped() const { return tx_dropped_.load(); }
    uint64_t rx_dropped() const { return rx_dropped_.load(); }
    uint64_t rx_crc_errors() const { return rx_crc_errors_.load(); }
    uint64_t rx_resync_bytes() const { return rx_resync_bytes_.load(); }

private:
    void tx_loop();
    void rx_loop();
    bool try_open();
    void close_port();
    void handle_rx_bytes(const uint8_t* data, size_t len);
    void push_frame(received_frame&& frame);

    drivers::serial_driver::SerialPortConfig make_config() const;
    static std::vector<uint8_t> build_frame(uint8_t frame_id, const uint8_t* data, uint8_t data_len);

    rclcpp::Node::SharedPtr node_;
    std::string serial_port_;
    int serial_baud_ = 115200;

    drivers::common::IoContext io_context_{1};
    std::unique_ptr<drivers::serial_driver::SerialPort> port_;
    std::mutex port_mutex_;     // 保护 open/close/send

    std::atomic<bool> running_{false};
    std::atomic<bool> connected_{false};

    std::thread tx_thread_;
    std::thread rx_thread_;

    // TX：生产者只入队，发送线程独占串口写
    std::mutex tx_mutex_;
    std::condition_variable tx_cv_;
    std::deque<std::vector<uint8_t>> tx_queue_;
    size_t tx_queue_capacity_ = 4;

    // RX：接收线程解析完整帧后入队，serial_read 只取帧
    std::mutex rx_mutex_;
    std::deque<received_frame> rx_frames_;
    size_t rx_frames_capacity_ = 16;
    std::vector<uint8_t> rx_accum_;     // 仅 RX 线程访问

    std::atomic<uint64_t> tx_sent_{0};
    std::atomic<uint64_t> tx_dropped_{0};
    std::atomic<uint64_t> rx_dropped_{0};
    std::atomic<uint64_t> rx_crc_errors_{0};
    std::atomic<uint64_t> rx_resync_bytes_{0};
};

}
