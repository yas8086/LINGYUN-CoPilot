// 灵云01号伴飞电脑 — SocketCAN 接口
// CAN 帧结构与收发抽象。核心为纯 C++(无 ROS 依赖),便于单元测试。
#ifndef AIRSHIP_CAN__CAN_INTERFACE_HPP_
#define AIRSHIP_CAN__CAN_INTERFACE_HPP_

#include <cstdint>
#include <mutex>
#include <string>

namespace airship_can
{

// CAN 数据帧(与 linux/can.h 的 canfd_frame 对应, 但独立于平台)
struct CanFrame
{
  uint32_t id;              // CAN ID (29 位扩展帧或 11 位标准帧)
  uint8_t len;              // 数据长度 (0-8)
  uint8_t data[8];          // 数据字节
  bool extended;            // 是否扩展帧 (29位)
};

// SocketCAN 接口抽象: 打开/关闭/发送/接收
class SocketCanInterface
{
public:
  // ifname: 网络接口名, 如 "can0"
  explicit SocketCanInterface(const std::string & ifname);
  ~SocketCanInterface();

  // 禁止拷贝
  SocketCanInterface(const SocketCanInterface &) = delete;
  SocketCanInterface & operator=(const SocketCanInterface &) = delete;

  // 支持移动(用于成员延迟初始化)
  SocketCanInterface(SocketCanInterface && other) noexcept;
  SocketCanInterface & operator=(SocketCanInterface && other) noexcept;

  // 打开 CAN 接口, 成功返回 true
  bool open();

  // 关闭接口
  void close();

  // 发送一帧, 成功返回 true
  // timeout_ms: 等待可写超时(默认 100ms)。CAN 接口异常(ERROR-PASSIVE/断线/队列满)时
  //             非阻塞轮询等待, 超时返回失败, 避免阻塞 socket 的 write 无限阻塞卡死调用方。
  bool send(const CanFrame & frame, int timeout_ms = 100);

  // 非阻塞接收一帧; 有数据返回 true 并填充 frame, 无数据返回 false
  // timeout_ms: 轮询等待超时
  bool receive(CanFrame & frame, int timeout_ms);

  // 接口是否已打开
  bool is_open() const;

  // 确保接口打开: 若尚未打开则尝试打开; 返回当前是否可用。
  // 用于实现 CAN 掉线自动重连(USB-CAN 插拔/接口重启后 fd 失效)。
  // 线程安全: 供多线程(如 DCDC 控制线程+接收线程)并发调用。
  bool ensure_open();

private:
  std::string ifname_;
  int fd_;
  // 保护 fd_ 的并发访问 (open/close/send/receive/ensure_open)
  mutable std::mutex mutex_;

  // 无锁内部实现 (调用方需持有 mutex_)
  bool open_unlocked();
  void close_unlocked();
  bool send_unlocked(const CanFrame & frame, int timeout_ms);
  bool receive_unlocked(CanFrame & frame, int timeout_ms);
};

}  // namespace airship_can

#endif  // AIRSHIP_CAN__CAN_INTERFACE_HPP_
