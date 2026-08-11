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
{
  std::lock_guard<std::mutex> lock(other.mutex_);
  ifname_ = std::move(other.ifname_);
  fd_ = other.fd_;
  other.fd_ = -1;
}

SocketCanInterface & SocketCanInterface::operator=(SocketCanInterface && other) noexcept
{
  if (this != &other) {
    std::lock_guard<std::mutex> lock_this(mutex_);
    std::lock_guard<std::mutex> lock_other(other.mutex_);
    close_unlocked();
    ifname_ = std::move(other.ifname_);
    fd_ = other.fd_;
    other.fd_ = -1;
  }
  return *this;
}

bool SocketCanInterface::open()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return open_unlocked();
}

void SocketCanInterface::close()
{
  std::lock_guard<std::mutex> lock(mutex_);
  close_unlocked();
}

bool SocketCanInterface::is_open() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return fd_ >= 0;
}

bool SocketCanInterface::ensure_open()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ >= 0) {
    // 已打开: 校验接口是否仍存在(USB-CAN 被拔掉后 ifindex 失效)
    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, ifname_.c_str(), IFNAMSIZ - 1);
    if (ioctl(fd_, SIOCGIFINDEX, &ifr) == 0) {
      return true;  // 接口仍有效
    }
    // 接口已失效, 关闭后重新打开
    close_unlocked();
  }
  return open_unlocked();
}

bool SocketCanInterface::send(const CanFrame & frame)
{
  std::lock_guard<std::mutex> lock(mutex_);
  return send_unlocked(frame);
}

bool SocketCanInterface::receive(CanFrame & frame, int timeout_ms)
{
  // 接收线程 poll 阻塞期间不持有全局锁, 避免阻塞 send(MPPT 查询线程等)造成发送抖动。
  // 通过 fd 快照 + 锁内校验实现线程安全: poll 用快照 fd, 锁内 read 前核对 fd_ 未变化。
  int fd;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (fd_ < 0) {
      return false;
    }
    fd = fd_;
  }

  // 锁外等待可读
  struct pollfd pfd;
  pfd.fd = fd;
  pfd.events = POLLIN;
  const int ret = ::poll(&pfd, 1, timeout_ms);
  if (ret <= 0) {
    return false;
  }
  if ((pfd.revents & POLLIN) == 0) {
    return false;  // 非可读事件(如 POLLERR/POLLHUP), 避免误读
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ != fd) {
    return false;  // poll 期间 fd 已变化(关闭/重开), 放弃本轮, 交由调用方重试
  }
  return receive_unlocked(frame, 0);
}

// ===== 无锁内部实现 (调用方需持有 mutex_) =====
bool SocketCanInterface::open_unlocked()
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
    close_unlocked();
    return false;
  }

  // 绑定接口
  struct sockaddr_can addr;
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    close_unlocked();
    return false;
  }

  return true;
}

void SocketCanInterface::close_unlocked()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SocketCanInterface::send_unlocked(const CanFrame & frame)
{
  if (fd_ < 0) {
    return false;
  }

  // 钳制 DLC 到 CAN 合法最大值 8, 防止 len>8 时 memcpy 读写越界
  const uint8_t dlc = frame.len > 8u ? 8u : frame.len;

  struct can_frame cframe;
  std::memset(&cframe, 0, sizeof(cframe));
  cframe.can_id = frame.id;
  if (frame.extended) {
    cframe.can_id |= CAN_EFF_FLAG;
  }
  cframe.can_dlc = dlc;
  std::memcpy(cframe.data, frame.data, dlc);

  const ssize_t n = ::write(fd_, &cframe, sizeof(cframe));
  return n == static_cast<ssize_t>(sizeof(cframe));
}

bool SocketCanInterface::receive_unlocked(CanFrame & frame, int timeout_ms)
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
