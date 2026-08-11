// 灵云01号伴飞电脑 — 串口接口实现
#include "airship_link/serial_interface.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <utility>

namespace airship_link
{

SerialInterface::SerialInterface(const std::string & device)
: device_(device), fd_(-1)
{
}

SerialInterface::~SerialInterface()
{
  close();
}

SerialInterface::SerialInterface(SerialInterface && other) noexcept
: device_(std::move(other.device_)), fd_(other.fd_)
{
  other.fd_ = -1;
}

SerialInterface & SerialInterface::operator=(SerialInterface && other) noexcept
{
  if (this != &other) {
    close();
    device_ = std::move(other.device_);
    fd_ = other.fd_;
    other.fd_ = -1;
  }
  return *this;
}

bool SerialInterface::open(BaudRate baud)
{
  fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    return false;
  }

  struct termios tty;
  std::memset(&tty, 0, sizeof(tty));
  if (tcgetattr(fd_, &tty) != 0) {
    close();
    return false;
  }

  // 波特率
  speed_t speed = B9600;
  switch (baud) {
    case BaudRate::k115200: speed = B115200; break;
    case BaudRate::k230400: speed = B230400; break;
    case BaudRate::k460800: speed = B460800; break;
    case BaudRate::k921600: speed = B921600; break;
    default: speed = B9600; break;
  }
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  // 8N1, 无流控
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
  // CLOCAL: 忽略 modem 控制线(USB 串口无 DCD), 防止悬空时触发 hangup 导致 read 返回 0/EIO
  // CREAD: 使能接收, 部分系统不设则无法收数据
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_iflag &= ~(IXON | IXOFF | ICRNL | INLCR | IGNCR);
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_oflag &= ~OPOST;

  // 阻塞读超时
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 1;

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    close();
    return false;
  }

  return true;
}

void SerialInterface::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialInterface::write(const char * data, size_t len)
{
  if (fd_ < 0) {
    return false;
  }
  // 循环写: 处理部分写入、EINTR(信号打断)与 EAGAIN(非阻塞缓冲满), 确保整帧写完成
  size_t written = 0;
  while (written < len) {
    const ssize_t n = ::write(fd_, data + written, len - written);
    if (n < 0) {
      if (errno == EINTR) {
        continue;  // 被信号打断, 重试
      }
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        // 非阻塞模式下内核输出缓冲满: 等待可写后再重试, 避免整帧写失败/半帧
        struct pollfd pfd{};
        pfd.fd = fd_;
        pfd.events = POLLOUT;
        if (::poll(&pfd, 1, 1000) <= 0) {
          return false;  // 超时仍不可写
        }
        continue;
      }
      return false;
    }
    if (n == 0) {
      return false;  // 无进展, 避免死循环
    }
    written += static_cast<size_t>(n);
  }
  return true;
}

bool SerialInterface::write(const std::string & data)
{
  return write(data.data(), data.size());
}

int SerialInterface::read(char * data, size_t len, int timeout_ms)
{
  if (fd_ < 0) {
    return -1;
  }
  // 等待数据可读(带超时)
  struct pollfd pfd{};
  pfd.fd = fd_;
  pfd.events = POLLIN;
  const int pr = ::poll(&pfd, 1, timeout_ms);
  if (pr <= 0) {
    return -1;  // 超时或错误
  }
  const ssize_t n = ::read(fd_, data, len);
  if (n < 0) {
    return -1;
  }
  return static_cast<int>(n);
}

void SerialInterface::flush_rx()
{
  if (fd_ < 0) {
    return;
  }
  ::tcflush(fd_, TCIFLUSH);
}

}  // namespace airship_link
