// 串口链路测试：以固定频率发送 3 个 float + 时间戳 + 序号，验证收发是否正常。
// 用法：serial_link_test <port> [hz]，默认 100Hz。

#include <eleven_serial_node/serial_mcu.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

namespace
{

std::atomic<bool> g_running{true};

void handle_sigint(int)
{
    g_running = false;
}

// 与 MCU 约定的业务帧 ID，按固件实际值修改
constexpr uint8_t kFrameId = 0x02;

// 三个固定测试值，按需修改
constexpr float kTestX = 1.0f;
constexpr float kTestY = 2.0f;
constexpr float kTestYawDeg = 30.0f;

// 载荷布局（全部小端，共 20 字节）：x(m) / y(m) / yaw(deg) / 时间戳(ms) / 序号
struct pose_payload
{
    float x;
    float y;
    float yaw;
    uint32_t stamp_ms;
    uint32_t seq;
};
static_assert(sizeof(pose_payload) == 20, "载荷必须是 20 字节，不能有填充");

// 毫秒时间戳取 system_clock，掉电/跨设备对比时可用；uint32 约 49.7 天回绕。
// 若 MCU 只关心相对时间，可改成从程序启动开始计时。
uint32_t now_ms()
{
    return static_cast<uint32_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count());
}

}  // namespace

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cerr << "usage: " << argv[0] << " <port> [hz]\n"
                  << "  e.g. " << argv[0] << " /dev/ttyACM0 100\n";
        return 1;
    }

    const std::string port = argv[1];
    const int hz = (argc >= 3) ? std::atoi(argv[2]) : 100;
    if (hz <= 0)
    {
        std::cerr << "hz must be > 0\n";
        return 1;
    }

    std::signal(SIGINT, handle_sigint);

    Eleven::serial_mcu link(port, 115200);
    std::cout << "sending " << hz << " Hz (frame id " << static_cast<unsigned>(kFrameId)
              << ", 20-byte payload) to " << port << ", Ctrl-C to stop\n";

    const auto period = std::chrono::microseconds(1000000 / hz);
    const auto start = std::chrono::steady_clock::now();
    auto next = start;
    auto last_report = start;

    uint32_t seq = 0;
    uint64_t rx_count = 0;
    pose_payload payload{};

    while (g_running)
    {
        payload.x = kTestX;
        payload.y = kTestY;
        payload.yaw = kTestYawDeg;
        payload.stamp_ms = now_ms();
        payload.seq = seq++;

        if (!link.serial_send(kFrameId, &payload, sizeof(payload)))
        {
            std::cerr << "send failed: payload too large or link not running\n";
        }

        Eleven::received_frame frame;
        while (link.serial_read(frame))
        {
            ++rx_count;
        }

        if (next - last_report >= std::chrono::seconds(1))
        {
            last_report = next;
            std::cout << "seq=" << payload.seq << " x=" << payload.x
                      << " y=" << payload.y << " yaw=" << payload.yaw
                      << " | tx_sent=" << link.tx_sent()
                      << " tx_dropped=" << link.tx_dropped()
                      << " rx=" << rx_count
                      << " open=" << link.isOpen() << '\n';
        }

        next += period;
        if (next < std::chrono::steady_clock::now())
        {
            next = std::chrono::steady_clock::now();    // 落后太多就重新对齐，不补发
        }
        std::this_thread::sleep_until(next);
    }

    std::cout << "stopped: tx_sent=" << link.tx_sent()
              << " tx_dropped=" << link.tx_dropped()
              << " rx=" << rx_count << '\n';
    return 0;
}
