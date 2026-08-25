// 灵云01号伴飞电脑 — UDP 发送接口封装 (用于 L33 数传网口下传)
//
// 与 serial_interface 同为无 ROS 依赖的传输层封装, 便于单元测试。
// 用于把遥测 JSON 帧经 UDP 发往数传网口 (L33 网桥 MAC 层透传, 地面同网段可收)。
#ifndef AIRSHIP_LINK__UDP_SENDER_HPP_
#define AIRSHIP_LINK__UDP_SENDER_HPP_

#include <cstdint>
#include <string>

namespace airship_link
{

// UDP 发送接口 (UDP 无连接, sendto 不保证对方收到; 仅负责尽力发送)
class UdpSender
{
public:
  // host 支持 IP 或主机名; port 为目标端口
  UdpSender(const std::string & host, uint16_t port);
  ~UdpSender();

  UdpSender(const UdpSender &) = delete;
  UdpSender & operator=(const UdpSender &) = delete;

  UdpSender(UdpSender && other) noexcept;
  UdpSender & operator=(UdpSender && other) noexcept;

  // 解析目标地址并创建 socket, 成功返回 true
  bool open();

  // 关闭 socket
  void close();

  // 向目标发送字节, 成功返回 true (UDP sendto 返回发送字节数)
  bool send(const char * data, size_t len);

  // 发送字符串
  bool send(const std::string & data);

  bool is_open() const {return fd_ >= 0;}

private:
  std::string host_;
  uint16_t port_;
  int fd_;
  std::uint32_t addr_;  // 目标 IPv4 地址 (网络字节序)
};

}  // namespace airship_link

#endif  // AIRSHIP_LINK__UDP_SENDER_HPP_