// 灵云01号伴飞电脑 — SerialInterface 串口接口单元测试 (基于 PTY 伪终端)
//
// 用 openpty 创建伪终端对: 从端送给 SerialInterface(open), 主端模拟对端设备。
// 覆盖: 打开/关闭/基本读写/read 超时/flush_rx/重复 open 不泄漏 fd/移动语义。
#include "airship_link/serial_interface.hpp"

#include <dirent.h>
#include <pty.h>
#include <unistd.h>

#include <gtest/gtest.h>

#include <cstddef>
#include <cstring>
#include <string>
#include <thread>

using airship_link::BaudRate;
using airship_link::SerialInterface;

namespace
{

// PTY 对: master 为主端, name 为从端路径(交给 SerialInterface open)
struct PtyPair
{
  int master = -1;
  std::string name;
  ~PtyPair() {if (master >= 0) {::close(master);}}
};

// 创建伪终端对, 失败返回 false(CI 容器无 tty 时上游测试可 SKIP)
bool make_pty(PtyPair & out)
{
  int slave = -1;
  if (openpty(&out.master, &slave, nullptr, nullptr, nullptr) != 0) {
    return false;
  }
  char * nm = ptsname(out.master);
  if (nm == nullptr) {
    ::close(slave);
    return false;
  }
  out.name = nm;
  ::close(slave);  // 从端由 SerialInterface 自行 open 路径
  return true;
}

// 统计 /proc/self/fd 目录项数(检测 fd 泄漏)
int count_open_fds()
{
  DIR * dp = opendir("/proc/self/fd");
  if (dp == nullptr) {
    return -1;
  }
  int n = 0;
  while (readdir(dp) != nullptr) {
    ++n;
  }
  closedir(dp);
  return n;
}

}  // namespace

// 打开/关闭基本生命周期
TEST(SerialInterfacePty, OpenAndClose)
{
  PtyPair p;
  ASSERT_TRUE(make_pty(p));
  SerialInterface s(p.name);
  EXPECT_FALSE(s.is_open());
  EXPECT_TRUE(s.open(BaudRate::k115200));
  EXPECT_TRUE(s.is_open());
  s.close();
  EXPECT_FALSE(s.is_open());
}

// 打开不存在的设备路径应失败
TEST(SerialInterfacePty, OpenInvalidPathFails)
{
  SerialInterface s("/dev/nonexistent_airship_test");
  EXPECT_FALSE(s.open(BaudRate::k9600));
  EXPECT_FALSE(s.is_open());
}

// 重复 open: 先释放旧 fd 再开新, 内部 net 不增长(防 fd 泄漏的回归锁)
TEST(SerialInterfacePty, ReopenNoFdLeak)
{
  PtyPair p;
  ASSERT_TRUE(make_pty(p));
  const int before = count_open_fds();
  {
    SerialInterface s(p.name);
    ASSERT_TRUE(s.open(BaudRate::k115200));
    for (int i = 0; i < 10; ++i) {
      EXPECT_TRUE(s.open(BaudRate::k115200));  // 每次 open 先 close 旧 fd 再开新
    }
    // 对象存活期间恰持有 1 个 fd(相比 before 基线)
  }
  const int after = count_open_fds();
  EXPECT_EQ(before, after) << "SerialInterface 反复 open/close 不应泄漏 fd";
}

// 串口 write -> PTY 主端读回内容一致 (整帧写回环)
TEST(SerialInterfacePty, WriteLoopback)
{
  PtyPair p;
  ASSERT_TRUE(make_pty(p));
  SerialInterface s(p.name);
  ASSERT_TRUE(s.open(BaudRate::k115200));

  const std::string payload = "0xAA\x55{\"t\":1234}\n";
  ASSERT_TRUE(s.write(payload));

  char buf[64];
  const ssize_t n = ::read(p.master, buf, sizeof(buf));
  ASSERT_EQ(n, static_cast<ssize_t>(payload.size()));
  EXPECT_EQ(std::string(buf, n), payload);
}

// PTY 主端写 -> 串口 read 读出 (读回环)
TEST(SerialInterfacePty, ReadLoopback)
{
  PtyPair p;
  ASSERT_TRUE(make_pty(p));
  SerialInterface s(p.name);
  ASSERT_TRUE(s.open(BaudRate::k115200));

  const std::string payload = "hello serial";
  ASSERT_EQ(::write(p.master, payload.data(), payload.size()),
    static_cast<ssize_t>(payload.size()));

  char buf[64];
  const int n = s.read(buf, sizeof(buf), 1000);
  ASSERT_EQ(n, static_cast<int>(payload.size()));
  EXPECT_EQ(std::string(buf, n), payload);
}

// 无数据时 read 需在超时后返回 -1(不阻塞)
TEST(SerialInterfacePty, ReadTimesOut)
{
  PtyPair p;
  ASSERT_TRUE(make_pty(p));
  SerialInterface s(p.name);
  ASSERT_TRUE(s.open(BaudRate::k115200));

  char buf[4];
  EXPECT_EQ(s.read(buf, sizeof(buf), 100), -1);
}

// flush_rx: TM主端写入残留数据, flush 后 read 超时(缓冲被清空)
TEST(SerialInterfacePty, FlushRxClearsInput)
{
  PtyPair p;
  ASSERT_TRUE(make_pty(p));
  SerialInterface s(p.name);
  ASSERT_TRUE(s.open(BaudRate::k115200));

  const char junk[] = "residual-stale-data";
  ASSERT_EQ(::write(p.master, junk, sizeof(junk) - 1), static_cast<ssize_t>(sizeof(junk) - 1));
  // 等待数据从主端到达从端输入缓冲后再 flush
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  s.flush_rx();
  char buf[8];
  EXPECT_EQ(s.read(buf, sizeof(buf), 100), -1) << "flush_rx 后不应读到残留数据";
}

// 移动构造与移动赋值: 所有权转移, moved-from 对象 is_open 为 false
TEST(SerialInterfacePty, MoveSemantics)
{
  PtyPair p1;
  PtyPair p2;
  ASSERT_TRUE(make_pty(p1));
  ASSERT_TRUE(make_pty(p2));
  SerialInterface a(p1.name);
  SerialInterface b(p2.name);
  ASSERT_TRUE(a.open(BaudRate::k115200));
  ASSERT_TRUE(b.open(BaudRate::k115200));

  SerialInterface a2(std::move(a));
  EXPECT_FALSE(a.is_open());
  EXPECT_TRUE(a2.is_open());

  a2 = std::move(b);
  EXPECT_FALSE(b.is_open());
  EXPECT_TRUE(a2.is_open());
}

// 重复 close 安全(幂等, 不崩溃)
TEST(SerialInterfacePty, CloseIdempotent)
{
  PtyPair p;
  ASSERT_TRUE(make_pty(p));
  SerialInterface s(p.name);
  ASSERT_TRUE(s.open(BaudRate::k115200));
  s.close();
  s.close();
  EXPECT_FALSE(s.is_open());
}
