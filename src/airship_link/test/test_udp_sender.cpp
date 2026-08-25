// 灵云01号伴飞电脑 — UdpSender 单元测试
#include "airship_link/udp_sender.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <gtest/gtest.h>

#include <cstring>
#include <string>

using airship_link::UdpSender;

// 辅助: 绑定一个本地 UDP 接收 socket, 返回 fd
static int bind_udp_recv(uint16_t port)
{
  int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) {
    return -1;
  }
  int opt = 1;
  setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
  sockaddr_in addr {};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = htons(port);
  if (::bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    ::close(fd);
    return -1;
  }
  return fd;
}

TEST(UdpSenderTest, ConstructDefaultClosed)
{
  UdpSender s("127.0.0.1", 14551);
  EXPECT_FALSE(s.is_open());
}

TEST(UdpSenderTest, OpenLocalhost)
{
  UdpSender s("127.0.0.1", 14551);
  EXPECT_TRUE(s.open());
  EXPECT_TRUE(s.is_open());
}

TEST(UdpSenderTest, SendReachesLocalRecv)
{
  // 用 0 端口交给内核分配可用端口较麻烦, 这里固定一个测试端口
  const uint16_t port = 14551;
  const int rfd = bind_udp_recv(port);
  ASSERT_GE(rfd, 0);

  char buf[512] = {};
  sockaddr_in cli {};
  socklen_t clilen = sizeof(cli);

  // 先设非阻塞, 便于 recvfrom 返回 0 表示未就绪; 这里用 SO_RCVTIMEO 更直观
  timeval tv {};
  tv.tv_sec = 1;
  setsockopt(rfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  UdpSender s("127.0.0.1", port);
  ASSERT_TRUE(s.open());

  const std::string payload = "0x55{\"bms\":{}}";
  EXPECT_TRUE(s.send(payload));

  const ssize_t n = ::recvfrom(rfd, buf, sizeof(buf) - 1, 0,
    reinterpret_cast<sockaddr *>(&cli), &clilen);
  ASSERT_GT(n, 0);
  buf[n] = '\0';
  EXPECT_EQ(std::string(buf), payload);

  ::close(rfd);
}

TEST(UdpSenderTest, MoveTransfersOwnership)
{
  UdpSender s("127.0.0.1", 14551);
  ASSERT_TRUE(s.open());
  UdpSender moved(std::move(s));
  EXPECT_TRUE(moved.is_open());
  // 原对象已转移, 关闭新对象资源被正确释放 (无崩溃即通过)
  moved.close();
  EXPECT_FALSE(moved.is_open());
}

TEST(UdpSenderTest, OpenWithEmptyHostFails)
{
  UdpSender s("", 14551);
  EXPECT_FALSE(s.open());
  EXPECT_FALSE(s.is_open());
}

TEST(UdpSenderTest, SendBeforeOpenFails)
{
  UdpSender s("127.0.0.1", 14551);
  EXPECT_FALSE(s.is_open());
  EXPECT_FALSE(s.send("data"));  // 未 open 时 send 应安全返回 false
}

TEST(UdpSenderTest, OpenWithBroadcastAddress)
{
  // 广播地址 open 应成功 (setsockopt SO_BROADCAST 生效, 不会 EACCES)
  UdpSender s("192.168.10.255", 20000);
  EXPECT_TRUE(s.open());
  EXPECT_TRUE(s.is_open());
}
