// 灵云01号伴飞电脑 — 串口接口封装
// POSIX 串口读写, 无 ROS 依赖, 便于单元测试
#ifndef AIRSHIP_LINK__SERIAL_INTERFACE_HPP_
#define AIRSHIP_LINK__SERIAL_INTERFACE_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace airship_link
{

// 串口波特率枚举
enum class BaudRate
{
  k9600 = 9600,
  k115200 = 115200,
  k230400 = 230400,
  k460800 = 460800,
  k921600 = 921600,
};

// POSIX 串口接口
class SerialInterface
{
public:
  explicit SerialInterface(const std::string & device);
  ~SerialInterface();

  SerialInterface(const SerialInterface &) = delete;
  SerialInterface & operator=(const SerialInterface &) = delete;

  // 支持移动(用于成员延迟初始化)
  SerialInterface(SerialInterface && other) noexcept;
  SerialInterface & operator=(SerialInterface && other) noexcept;

  // 打开串口, 成功返回 true
  bool open(BaudRate baud);

  // 关闭串口
  void close();

  // 写入字节, 成功返回 true
  bool write(const char * data, size_t len);

  // 写入字符串
  bool write(const std::string & data);

  // 读取最多 len 字节到 data, 最多等待 timeout_ms 毫秒。
  // 返回实际读取字节数; 超时/错误返回 -1。(用于 Modbus 等请求-响应协议)
  int read(char * data, size_t len, int timeout_ms);

  // 清空接收缓冲(Modbus 请求前调用, 避免读到上次残留的应答)
  void flush_rx();

  bool is_open() const {return fd_ >= 0;}

private:
  std::string device_;
  int fd_;
};

}  // namespace airship_link

#endif  // AIRSHIP_LINK__SERIAL_INTERFACE_HPP_
