// 灵云01号伴飞电脑 — 串口接口实现
#include "airship_link/serial_interface.hpp"

#include <fcntl.h>
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
  const ssize_t n = ::write(fd_, data, len);
  return n == static_cast<ssize_t>(len);
}

bool SerialInterface::write(const std::string & data)
{
  return write(data.data(), data.size());
}

}  // namespace airship_link
