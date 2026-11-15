# Eleven 串口通信协议

本文档描述 `include/eleven_serial_node/serial_mcu.hpp` 收发的二进制帧格式。库只负责组帧、校验、拆帧和收发，**帧 ID 的业务含义由应用层（节点）和 下位机共同约定**。

## 1. 物理层

| 项目 | 值 |
|---|---|
| 波特率 | 115200（可由构造函数 `baud` 参数覆盖） |
| 数据位 | 8 |
| 校验位 | 无（Parity::NONE） |
| 停止位 | 1 |
| 流控 | 无 |

字节序：多字节字段一律**小端**。`float` 为 IEEE-754 单精度，直接 `memcpy` 到线路上，因此要求双方都是小端平台且 `sizeof(float) == 4`。

## 2. 帧格式

```
偏移      字段        长度      说明
------------------------------------------------------------------------
0         HEAD0       1         0xFF
1         HEAD1       1         0xFE
2         ID          1         帧 ID（0x00~0xFF），由应用层定义
3         LEN         1         DATA 的字节数，0 ~ 64
4         DATA        LEN       有效载荷
4 + LEN   CRC_L       1         CRC16-Modbus 低字节
5 + LEN   CRC_H       1         CRC16-Modbus 高字节
6 + LEN   END0        1         0xFD
7 + LEN   END1        1         0xFB
------------------------------------------------------------------------
帧总长度 = LEN + 8，最大 72 字节
```

常量定义（`serial_mcu.hpp`）：

| 常量 | 值 |
|---|---|
| `FRAME_HEAD_0` | `0xFF` |
| `FRAME_HEAD_1` | `0xFE` |
| `FRAME_END_0` | `0xFD` |
| `FRAME_END_1` | `0xFB` |
| `MAX_PAYLOAD_LEN` | `64`（字节） |
| `MAX_DATA_LENGTH` | `16`（float 个数） |
| `FRAME_OVERHEAD` | `8`（字节） |

## 3. 校验

- 算法：**CRC16-Modbus**（多项式 0x8005 反射形式 0xA001，初值 `0xFFFF`，无最终异或）
- 覆盖范围：**ID + LEN + DATA**，即从偏移 2 开始、长度为 `LEN + 2` 字节
- 发送顺序：**低字节在前，高字节在后**

等价伪代码：

```text
crc = 0xFFFF
for byte in frame[2 : 4 + LEN]:     # ID, LEN, DATA
    crc = (crc >> 8) ^ table[(crc ^ byte) & 0xFF]
frame[4 + LEN] = crc & 0xFF
frame[5 + LEN] = crc >> 8
```

MCU 侧实现必须与此完全一致，否则会表现为“帧能收到但全部被丢弃”。

## 4. 载荷编码

`DATA` 是裸字节，库不做任何序列化：

- 发送 float 数组：`serial_send(id, float_array, count)`，写入 `count * 4` 字节
- 发送任意字节：`serial_send(id, void_ptr, byte_len)`，最大 64 字节
- 接收端按 4 字节一个单位读取：
  - `frame.as_float(i)`：第 `i` 个 float（偏移 `i * 4`）
  - `frame.as_u32(i)`：第 `i` 个 `uint32_t`（偏移 `i * 4`）
  - 越界时例如读取 `len` 之外的索引会返回 0，调用方应先检查 `frame.len`

## 5. 帧 ID

库不解释 ID，只原样收发。

## 6. 接收端的容错行为

接收线程把串口字节累积到内部缓冲区，然后循环拆帧：

| 情况 | 处理 |
|---|---|
| 帧头不匹配 | 丢弃 1 字节，重新找 `0xFF 0xFE` |
| `LEN > 64` | 丢弃 2 字节（帧头），重新同步 |
| 数据未收全 | 保留缓冲，等待下次 `receive()` |
| 帧尾不是 `0xFD 0xFB` | 丢弃 1 字节，重新同步 |
| CRC 不匹配 | 计数 `rx_crc_errors`，丢弃 2 字节，重新同步 |
| 缓冲超过 4096 字节 | 丢弃最旧的多余字节 |

`LEN` 字段损坏时会退化为逐字节重新同步，因此一帧损坏不会吞掉后续的好帧，但会付出重新扫描的代价。

统计接口：`rx_crc_errors()`、`rx_resync_bytes()`、`rx_dropped()`、`tx_dropped()`、`tx_sent()`。

## 7. 线程模型

构造时打开串口（**只尝试一次**），随后启动两个线程：

| 线程 | 职责 |
|---|---|
| RX 线程 | 阻塞 `receive()` → 拆帧 → 放入接收队列（容量 16，满则丢最旧） |
| TX 线程 | 从发送队列取帧（容量 4，满则丢最旧）→ `send()` |

- `serial_send()` 是非阻塞的：只入队，返回 `true` 不代表已经写到串口
- `serial_read()` 是非阻塞的：队列为空返回 `false`，需要调用方轮询（例如放在节点的定时器/主循环里）
- 串口未打开时，入队的帧会被 TX 线程直接丢弃并计入 `tx_dropped`
- 打开失败不会阻塞构造，RX 线程每 200ms 尝试重连一次
- 端口生命周期由 RX 线程独占：发送出错时 TX 线程只置断开标志并叫醒 RX，由 RX 关闭并重连
- **析构**：`serial_driver` 的 `receive()`/`send()` 没有超时，且 Linux 下 `close()` 唤不醒阻塞中的 `poll()`，所以析构用 `SIGUSR1` 打断阻塞的系统调用（重复唤醒以覆盖信号早于阻塞的窗口），join 两个线程后才关闭并释放端口
- 因此本类会在构造时安装**进程级 `SIGUSR1` 处理函数**（空实现，不设 `SA_RESTART`）；若你的程序自己要用 `SIGUSR1`，需要避开或改成本类内部的信号

日志直接输出到 `std::cout` / `std::cerr`，只在状态变化时打印（打开成功 / 首次打开失败 / 收发异常）。

### 链路测试程序

```bash
ros2 run eleven_serial_node serial_link_test /dev/ttyACM0 100
```

- 参数：`<port> [hz]`，hz 默认 100
- 帧 ID `0x02`，载荷 20 字节：`x=1.0f, y=2.0f, yaw=30.0f`（固定值）+ `uint32 时间戳(ms)` + `uint32 seq`（每帧 +1）
- 每发一帧打印一行 `[tx]`：`id / len / x / y / yaw / stamp / seq`（表示已入发送队列，不代表已写到线路上）
- 每收到一帧解析并打印一行 `[rx]`：同样打印 ID、长度和解析出的字段；本测试帧字段对不上时追加 `[MISMATCH]`，其他 ID/长度的帧按十六进制打印 `data`
- 每秒打印一次 `[stat]`：`tx_sent / tx_dropped / rx / rx_bad / crc_err / resync / rx_drop / open`
- Ctrl-C 退出
- 时间戳取 `system_clock` 毫秒，uint32 约 49.7 天回绕；只关心相对时间的话可改成程序启动计时
- 注意：每帧都打印时 100Hz 约 200 行/秒、200Hz 约 400 行/秒；若终端成为瓶颈导致发送掉拍，可降低 hz 或重定向到文件观察

## 8. 构建与依赖

- 头文件模式：所有实现都在 `serial_mcu.hpp`，不需要编译 `.so`
- 依赖：`serial_driver`（ROS2 transport_drivers）、`Threads`
- ROS2 工作区内：`<depend>serial_driver</depend>` + `find_package(serial_driver REQUIRED)`，并把 `${serial_driver_INCLUDE_DIRS}` 和 `${serial_driver_LIBRARIES}` 加到目标上
- 复制到其他 ROS2 包：把 `serial_mcu.hpp` 复制到该包 include 目录，同样加上 serial_driver 的 include 和库即可
- 注意：`serial_driver` 自身依赖 `rclcpp` / `rclcpp_lifecycle` / `std_msgs`，因此**非 ROS2 环境无法只靠复制头文件使用**；如需完全脱离 ROS，应把底层换成 asio/termios

## 9. 与 serial_node 旧协议的区别

`serial_node`（ROS1）是九期的通讯代码，使用的是另一套帧格式，两者**不能直接互通**：

| 项目 | serial_node 旧协议 | 本协议 |
|---|---|---|
| 帧头 | `0xFC 0xFB` | `0xFF 0xFE` |
| 帧尾 | `0xFD 0xFE` | `0xFD 0xFB` |
| 校验 | CRC16（CCITT 风格表），**只覆盖 DATA** | CRC16-Modbus，覆盖 **ID + LEN + DATA** |
| 最大载荷 | 36 个 float（144 字节） | 64 字节（16 个 float） |

对接下位机前务必确认固件使用的是哪一套。
