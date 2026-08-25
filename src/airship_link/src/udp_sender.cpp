// 灵云01号伴飞电脑 — UDP 发送接口实现
#include "airship_link/udp_sender.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cstring>

namespace airship_link
{

UdpSender::UdpSender(const std::string & host, uint16_t port)
: host_(host), port_(port), fd_(-1), addr_(0)
{
}

UdpSender::~UdpSender()
{
  close();
}

UdpSender::UdpSender(UdpSender && other) noexcept
: host_(std::move(other.host_)),
  port_(other.port_),
  fd_(other.fd_),
  addr_(other.addr_)
{
  other.fd_ = -1;  // 转移所有权, 避免析构时误关
}

UdpSender & UdpSender::operator=(UdpSender && other) noexcept
{
  if (this != &other) {
    close();
    host_ = std::move(other.host_);
    port_ = other.port_;
    fd_ = other.fd_;
    addr_ = other.addr_;
    other.fd_ = -1;
  }
  return *this;
}

bool UdpSender::open()
{
  if (host_.empty()) {
    return false;
  }
  // 解析目标地址 (IP 或主机名)
  struct in_addr target {};
  if (inet_pton(AF_INET, host_.c_str(), &target) != 1) {
    return false;  // 非点分十进制 IP, 本实现暂仅支持 IP 字面量
  }
  addr_ = static_cast<std::uint32_t>(target.s_addr);

  if (fd_ >= 0) {
    return true;  // 已打开
  }
  fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd_ < 0) {
    return false;
  }
  // 允许向子网广播地址 (255.255.255.x / host 段全 1) 发送, 否则 sendto 广播会报 EACCES
  int broadcast = 1;
  if (::setsockopt(fd_, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) < 0) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }
  return true;
}

void UdpSender::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool UdpSender::send(const char * data, size_t len)
{
  if (fd_ < 0 || data == nullptr) {
    return false;
  }
  struct sockaddr_in dst {};
  dst.sin_family = AF_INET;
  dst.sin_port = htons(port_);
  dst.sin_addr.s_addr = addr_;
  const ssize_t ret = ::sendto(
    fd_, data, len, 0,
    reinterpret_cast<const struct sockaddr *>(&dst), sizeof(dst));
  return ret == static_cast<ssize_t>(len);
}

bool UdpSender::send(const std::string & data)
{
  return send(data.data(), data.size());
}

}  // namespace airship_link