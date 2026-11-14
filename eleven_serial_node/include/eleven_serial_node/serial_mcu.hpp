#pragma once

#include <serial_driver/serial_driver.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace Eleven
{
#define FRAME_HEAD_0 0xFF
#define FRAME_HEAD_1 0xFE
#define FRAME_END_0 0xFD
#define FRAME_END_1 0xFB
#define MAX_PAYLOAD_LEN 64      // 字节数
#define MAX_DATA_LENGTH (MAX_PAYLOAD_LEN / sizeof(float))

inline constexpr std::size_t FRAME_OVERHEAD = 8;    // head(2) + id(1) + len(1) + crc(2) + tail(2)
inline constexpr std::size_t MAX_FRAME_LEN = MAX_PAYLOAD_LEN + FRAME_OVERHEAD;

inline uint16_t CRC16_Modbus(const uint8_t *p, std::size_t len)
{
    static const uint16_t CRC16ModbusTable[256] =
    {
        0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
        0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
        0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
        0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
        0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
        0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
        0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
        0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
        0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
        0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
        0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
        0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
        0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
        0xEE01, 0x2EC1, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
        0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
        0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
        0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
        0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
        0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
        0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
        0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
        0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
        0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
        0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
        0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
        0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
        0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
        0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
        0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
        0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
        0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
        0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040,
    };

    uint16_t crc = 0xFFFF;
    for (std::size_t i = 0; i < len; i++)
    {
        crc = static_cast<uint16_t>((crc >> 8) ^ CRC16ModbusTable[(crc ^ p[i]) & 0xFF]);
    }
    return crc;
}

struct received_frame
{
    uint8_t id = 0;
    uint8_t len = 0;        // 有效数据字节数
    std::array<uint8_t, MAX_PAYLOAD_LEN> data{};

    // index 以 4 字节为一个单位；3 个 float + 2 个 uint32 时依次为 0,1,2 和 3,4
    float as_float(std::size_t index) const
    {
        float value = 0.0f;
        const std::size_t offset = index * sizeof(float);
        if (offset + sizeof(float) <= len)
        {
            std::memcpy(&value, data.data() + offset, sizeof(float));
        }
        return value;
    }

    uint32_t as_u32(std::size_t index) const
    {
        uint32_t value = 0;
        const std::size_t offset = index * sizeof(uint32_t);
        if (offset + sizeof(uint32_t) <= len)
        {
            std::memcpy(&value, data.data() + offset, sizeof(uint32_t));
        }
        return value;
    }
};

class serial_mcu
{
public:
    serial_mcu(const std::string& port, int baud = 115200);
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
    void handle_rx_bytes(const uint8_t* data, std::size_t len);
    void push_frame(received_frame&& frame);

    drivers::serial_driver::SerialPortConfig make_config() const;
    static std::vector<uint8_t> build_frame(uint8_t frame_id, const uint8_t* data, uint8_t data_len);

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
    std::size_t tx_queue_capacity_ = 4;

    // RX：接收线程解析完整帧后入队，serial_read 只取帧
    std::mutex rx_mutex_;
    std::deque<received_frame> rx_frames_;
    std::size_t rx_frames_capacity_ = 16;
    std::vector<uint8_t> rx_accum_;     // 仅 RX 线程访问

    std::atomic<uint64_t> tx_sent_{0};
    std::atomic<uint64_t> tx_dropped_{0};
    std::atomic<uint64_t> rx_dropped_{0};
    std::atomic<uint64_t> rx_crc_errors_{0};
    std::atomic<uint64_t> rx_resync_bytes_{0};
};

inline serial_mcu::serial_mcu(const std::string& port, int baud)
{
    serial_port_ = port;
    serial_baud_ = baud;

    // 只尝试一次；失败时由 rx_loop 在后台按 200ms 间隔重连
    if (!try_open())
    {
        std::cerr << "[serial_mcu] failed to open " << serial_port_
                  << ", retrying in background" << std::endl;
    }

    running_ = true;
    rx_thread_ = std::thread(&serial_mcu::rx_loop, this);
    tx_thread_ = std::thread(&serial_mcu::tx_loop, this);
}

inline serial_mcu::~serial_mcu()
{
    running_ = false;
    tx_cv_.notify_all();

    // serial_driver 的 send/receive 没有超时，关闭端口以解除阻塞。
    // 这里只 close、不 reset：RX 线程可能正阻塞在 port_->receive() 里，
    // 提前 reset 会让它持有悬空指针。reset 放到两个线程 join 之后。
    {
        std::lock_guard<std::mutex> lock(port_mutex_);
        if (port_)
        {
            try
            {
                port_->close();
            }
            catch (const std::exception&)
            {
            }
        }
    }

    if (tx_thread_.joinable())
    {
        tx_thread_.join();
    }
    if (rx_thread_.joinable())
    {
        rx_thread_.join();
    }

    // 线程都已退出，此时 close/reset 不会再和任何线程竞争
    close_port();
}

inline drivers::serial_driver::SerialPortConfig serial_mcu::make_config() const
{
    return drivers::serial_driver::SerialPortConfig(
        static_cast<uint32_t>(serial_baud_),                // 波特率
        drivers::serial_driver::FlowControl::NONE,          // 无流控
        drivers::serial_driver::Parity::NONE,               // 无校验
        drivers::serial_driver::StopBits::ONE);             // 1位停止位
}

inline bool serial_mcu::try_open()
{
    std::lock_guard<std::mutex> lock(port_mutex_);
    try
    {
        if (port_)
        {
            try
            {
                port_->close();
            }
            catch (const std::exception&)
            {
            }
            port_.reset();
        }

        port_ = std::make_unique<drivers::serial_driver::SerialPort>(
            io_context_, serial_port_, make_config());
        port_->open();
        if (!port_->is_open())
        {
            throw std::runtime_error("serial port is not open after open()");
        }
        connected_ = true;
        rx_accum_.clear();
        std::cout << "[serial_mcu] " << serial_port_ << " open success ("
                  << serial_baud_ << " baud)" << std::endl;
        return true;
    }
    catch (const std::exception&)
    {
        connected_ = false;
        port_.reset();
        return false;
    }
}

inline void serial_mcu::close_port()
{
    std::lock_guard<std::mutex> lock(port_mutex_);
    connected_ = false;
    if (port_)
    {
        try
        {
            if (port_->is_open())
            {
                port_->close();
            }
        }
        catch (const std::exception&)
        {
        }
        port_.reset();          // 清理 SerialPort 对象
    }
}

inline void serial_mcu::tx_loop()
{
    while (running_)
    {
        std::vector<uint8_t> frame;
        {
            std::unique_lock<std::mutex> lock(tx_mutex_);
            tx_cv_.wait(lock, [this] { return !running_ || !tx_queue_.empty(); });
            if (!running_)
            {
                return;
            }
            frame = std::move(tx_queue_.front());
            tx_queue_.pop_front();
        }

        if (!connected_)
        {
            tx_dropped_++;
            continue;
        }

        try
        {
            std::lock_guard<std::mutex> lock(port_mutex_);
            if (!port_ || !port_->is_open())
            {
                tx_dropped_++;
                continue;
            }

            const std::size_t sent = port_->send(frame);
            if (sent == frame.size())
            {
                tx_sent_++;
            }
            else
            {
                // 短写会留下半帧，丢弃本帧并关端口，让阻塞中的 RX 线程醒来重连
                tx_dropped_++;
                connected_ = false;
                try
                {
                    if (port_ && port_->is_open())
                    {
                        port_->close();
                    }
                }
                catch (const std::exception&)
                {
                }
                std::cerr << "[serial_mcu] partial write: " << sent << " / "
                          << frame.size() << std::endl;
            }
        }
        catch (const std::exception& e)
        {
            if (!running_)
            {
                return;
            }
            tx_dropped_++;
            connected_ = false;
            try
            {
                if (port_ && port_->is_open())
                {
                    port_->close();
                }
            }
            catch (const std::exception&)
            {
            }
            std::cerr << "[serial_mcu] send failed: " << e.what() << std::endl;
        }
    }
}

inline void serial_mcu::rx_loop()
{
    std::vector<uint8_t> chunk(512);

    while (running_)
    {
        if (!connected_)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            if (!running_)
            {
                break;
            }
            try_open();     // 成功时由 try_open 打印一次
            continue;
        }

        try
        {
            // 故意不加 port_mutex_：receive() 会一直阻塞，持锁会让 TX 线程永远发不出去。
            // port_ 只在本线程（try_open/close_port）或析构 join 之后才会被 reset。
            const std::size_t got = port_->receive(chunk);
            if (!running_)
            {
                break;
            }
            if (got > 0)
            {
                handle_rx_bytes(chunk.data(), got);
            }
        }
        catch (const std::exception& e)
        {
            if (!running_)
            {
                break;
            }
            connected_ = false;
            std::cerr << "[serial_mcu] receive failed: " << e.what() << std::endl;
            close_port();
        }
    }
}

inline bool serial_mcu::serial_send(uint8_t frame_id, const void* data, uint8_t data_len)
{
    if (data_len > MAX_PAYLOAD_LEN)
    {
        std::cerr << "[serial_mcu] data too long: " << static_cast<unsigned>(data_len)
                  << " bytes (max " << MAX_PAYLOAD_LEN << ")" << std::endl;
        return false;
    }
    if (data_len > 0 && data == nullptr)
    {
        return false;
    }
    if (!running_)
    {
        return false;
    }

    std::vector<uint8_t> frame =
        build_frame(frame_id, static_cast<const uint8_t*>(data), data_len);

    {
        std::lock_guard<std::mutex> lock(tx_mutex_);
        if (tx_queue_.size() >= tx_queue_capacity_)
        {
            tx_queue_.pop_front();      // 全局丢最旧
            tx_dropped_++;
        }
        tx_queue_.push_back(std::move(frame));
    }
    tx_cv_.notify_one();
    return true;
}

inline bool serial_mcu::serial_send(uint8_t frame_id, const float* msgs, uint8_t count)
{
    if (static_cast<std::size_t>(count) > MAX_DATA_LENGTH)
    {
        std::cerr << "[serial_mcu] too many floats: " << static_cast<unsigned>(count)
                  << " (max " << MAX_DATA_LENGTH << ")" << std::endl;
        return false;
    }
    if (count > 0 && msgs == nullptr)
    {
        return false;
    }
    const uint8_t data_len = static_cast<uint8_t>(count * sizeof(float));
    return serial_send(frame_id, static_cast<const void*>(msgs), data_len);
}

inline bool serial_mcu::serial_read(received_frame& out)
{
    std::lock_guard<std::mutex> lock(rx_mutex_);
    if (rx_frames_.empty())
    {
        return false;
    }
    out = std::move(rx_frames_.front());
    rx_frames_.pop_front();
    return true;
}

inline void serial_mcu::handle_rx_bytes(const uint8_t* data, std::size_t len)
{
    constexpr std::size_t MAX_RX_ACCUM = 4096;   // 防止半帧/垃圾数据无限堆积

    rx_accum_.insert(rx_accum_.end(), data, data + len);

    while (rx_accum_.size() >= 4)
    {
        if (rx_accum_[0] != FRAME_HEAD_0 || rx_accum_[1] != FRAME_HEAD_1)
        {
            rx_accum_.erase(rx_accum_.begin());
            rx_resync_bytes_++;
            continue;
        }

        const uint8_t data_length = rx_accum_[3];
        if (data_length > MAX_PAYLOAD_LEN)
        {
            rx_accum_.erase(rx_accum_.begin(), rx_accum_.begin() + 2);
            rx_resync_bytes_ += 2;
            continue;
        }

        const std::size_t frame_len = static_cast<std::size_t>(data_length) + FRAME_OVERHEAD;
        if (rx_accum_.size() < frame_len)
        {
            break;      // 数据还没收全
        }

        const std::size_t crc_off = 4 + data_length;
        if (rx_accum_[crc_off + 2] != FRAME_END_0 || rx_accum_[crc_off + 3] != FRAME_END_1)
        {
            rx_accum_.erase(rx_accum_.begin());
            rx_resync_bytes_++;
            continue;
        }

        const uint16_t recv_crc = static_cast<uint16_t>(
            rx_accum_[crc_off] | (rx_accum_[crc_off + 1] << 8));
        if (CRC16_Modbus(rx_accum_.data() + 2, 2 + data_length) != recv_crc)
        {
            // 长度字段可能已损坏，只丢掉帧头重新找同步，避免吞掉后面的好帧
            rx_crc_errors_++;
            rx_accum_.erase(rx_accum_.begin(), rx_accum_.begin() + 2);
            rx_resync_bytes_ += 2;
            continue;
        }

        received_frame frame;
        frame.id = rx_accum_[2];
        frame.len = data_length;
        std::copy_n(rx_accum_.begin() + 4, data_length, frame.data.begin());
        push_frame(std::move(frame));

        rx_accum_.erase(rx_accum_.begin(), rx_accum_.begin() + frame_len);
    }

    if (rx_accum_.size() > MAX_RX_ACCUM)
    {
        const std::size_t excess = rx_accum_.size() - MAX_RX_ACCUM;
        rx_accum_.erase(rx_accum_.begin(), rx_accum_.begin() + excess);
        rx_resync_bytes_ += excess;
    }
}

inline void serial_mcu::push_frame(received_frame&& frame)
{
    std::lock_guard<std::mutex> lock(rx_mutex_);
    if (rx_frames_.size() >= rx_frames_capacity_)
    {
        rx_frames_.pop_front();
        rx_dropped_++;
    }
    rx_frames_.push_back(std::move(frame));
}

inline std::vector<uint8_t> serial_mcu::build_frame(uint8_t frame_id, const uint8_t* data,
                                                    uint8_t data_len)
{
    std::vector<uint8_t> frame(static_cast<std::size_t>(data_len) + FRAME_OVERHEAD, 0);
    frame[0] = FRAME_HEAD_0;
    frame[1] = FRAME_HEAD_1;
    frame[2] = frame_id;
    frame[3] = data_len;

    if (data_len > 0)
    {
        std::memcpy(&frame[4], data, data_len);
    }

    // CRC16-Modbus 覆盖 ID + LEN + DATA
    const uint16_t crc = CRC16_Modbus(&frame[2], static_cast<std::size_t>(data_len) + 2);
    frame[4 + data_len] = static_cast<uint8_t>(crc & 0xFF);
    frame[5 + data_len] = static_cast<uint8_t>((crc >> 8) & 0xFF);
    frame[6 + data_len] = FRAME_END_0;
    frame[7 + data_len] = FRAME_END_1;
    return frame;
}

}   // namespace Eleven
