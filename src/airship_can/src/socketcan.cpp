// 灵云01号伴飞电脑 — SocketCAN 接口实现
#include "airship_can/can_interface.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <utility>

namespace airship_can
{

SocketCanInterface::SocketCanInterface(const std::string & ifname)
: ifname_(ifname), fd_(-1)
{
}

SocketCanInterface::~SocketCanInterface()
{
  close();
}

SocketCanInterface::SocketCanInterface(SocketCanInterface && other) noexcept
: ifname_(std::move(other.ifname_)), fd_(other.fd_)
{
  other.fd_ = -1;
}

SocketCanInterface & SocketCanInterface::operator=(SocketCanInterface && other) noexcept
{
  if (this != &other) {
    close();
    ifname_ = std::move(other.ifname_);
    fd_ = other.fd_;
    other.fd_ = -1;
  }
  return *this;
}

bool SocketCanInterface::open()
{
  if (fd_ >= 0) {
    return true;
  }

  fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd_ < 0) {
    return false;
  }

  // 找到接口索引
  struct ifreq ifr;
  std::memset(&ifr, 0, sizeof(ifr));
  std::strncpy(ifr.ifr_name, ifname_.c_str(), IFNAMSIZ - 1);
  if (ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
    close();
    return false;
  }

  // 绑定接口
  struct sockaddr_can addr;
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    close();
    return false;
  }

  return true;
}

void SocketCanInterface::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SocketCanInterface::send(const CanFrame & frame)
{
  if (fd_ < 0) {
    return false;
  }

  struct can_frame cframe;
  std::memset(&cframe, 0, sizeof(cframe));
  cframe.can_id = frame.id;
  if (frame.extended) {
    cframe.can_id |= CAN_EFF_FLAG;
  }
  cframe.can_dlc = frame.len;
  std::memcpy(cframe.data, frame.data, frame.len);

  const ssize_t n = ::write(fd_, &cframe, sizeof(cframe));
  return n == static_cast<ssize_t>(sizeof(cframe));
}

bool SocketCanInterface::receive(CanFrame & frame, int timeout_ms)
{
  if (fd_ < 0) {
    return false;
  }

  // 轮询等待可读
  struct pollfd pfd;
  pfd.fd = fd_;
  pfd.events = POLLIN;
  const int ret = ::poll(&pfd, 1, timeout_ms);
  if (ret <= 0) {
    return false;
  }

  struct can_frame cframe;
  const ssize_t n = ::read(fd_, &cframe, sizeof(cframe));
  if (n != static_cast<ssize_t>(sizeof(cframe))) {
    return false;
  }

  frame.id = cframe.can_id & CAN_EFF_MASK;
  frame.extended = (cframe.can_id & CAN_EFF_FLAG) != 0;
  frame.len = cframe.can_dlc > 8 ? 8 : cframe.can_dlc;
  std::memcpy(frame.data, cframe.data, frame.len);
  return true;
}

}  // namespace airship_can
