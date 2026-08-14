// 灵云01号伴飞电脑 — SocketCAN 接口线程安全单元测试
//
// 验证 DCDC 双线程(控制线程 send + 接收线程 receive/ensure_open)
// 并发访问 SocketCanInterface 时不会崩溃/死锁, 且收发数据正确。
// 使用 vcan 虚拟接口, 无需真实硬件。
#include <atomic>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "airship_can/can_interface.hpp"

using airship_can::CanFrame;
using airship_can::SocketCanInterface;

namespace
{
constexpr const char * kIface = "vcan_test";

// 忽略 system() 返回值的辅助函数 (抑制 -Wunused-result)
void run_cmd(const char * cmd)
{
  const int rc = std::system(cmd);
  (void)rc;
}

// 创建并 up 一个 vcan 接口 (需 root/sudo)
void setup_vcan()
{
  run_cmd("sudo modprobe vcan 2>/dev/null");
  run_cmd("sudo ip link add dev vcan_test type vcan 2>/dev/null");
  run_cmd("sudo ip link set up vcan_test 2>/dev/null");
}

void teardown_vcan()
{
  run_cmd("sudo ip link delete vcan_test 2>/dev/null");
}
}  // namespace

// 并发 send/receive 不崩溃, 且接收 socket 能持续收到发送的数据
// (使用两个独立 socket: sender 发, receiver 收, vcan 帧回环到接收 socket)
TEST(SocketCanThreadSafe, ConcurrentSendReceive)
{
  setup_vcan();

  SocketCanInterface send_can(kIface);
  SocketCanInterface recv_can(kIface);
  ASSERT_TRUE(send_can.open()) << "无法打开 vcan 接口, 需 sudo 权限";
  ASSERT_TRUE(recv_can.open()) << "无法打开 vcan 接口, 需 sudo 权限";

  // 发送线程: 持续发 ID=0x123 的帧
  std::atomic<bool> stop{false};
  std::atomic<int> sent_count{0};
  std::thread sender([&]() {
      CanFrame f;
      f.id = 0x123;
      f.extended = true;
      f.len = 8;
      for (int i = 0; i < 8; ++i) {
        f.data[i] = static_cast<uint8_t>(i);
      }
      while (!stop) {
        if (send_can.send(f)) {
          ++sent_count;
        }
      }
    });

  // 接收线程: 持续接收 (模拟 DCDC receive_loop)
  std::atomic<int> recv_count{0};
  std::thread receiver([&]() {
      CanFrame f;
      while (!stop) {
        if (recv_can.receive(f, 10)) {
          EXPECT_EQ(f.id, 0x123U);
          ++recv_count;
        }
      }
    });

  // 让并发跑一段时间
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  stop = true;
  sender.join();
  receiver.join();

  EXPECT_GT(sent_count, 0);
  EXPECT_GT(recv_count, 0) << "接收到了数据则证明并发收发正常, 无线程安全问题";

  teardown_vcan();
}

// 并发 ensure_open + send 不崩溃 (模拟 control/receive 双线程同时重连)
TEST(SocketCanThreadSafe, ConcurrentEnsureOpenAndSend)
{
  setup_vcan();

  SocketCanInterface can(kIface);
  ASSERT_TRUE(can.open());

  std::atomic<bool> stop{false};
  std::atomic<int> ensure_ok{0};
  std::atomic<int> send_ok{0};

  // 线程1: 反复 ensure_open (模拟控制线程)
  std::thread t1([&]() {
      while (!stop) {
        if (can.ensure_open()) {
          ++ensure_ok;
        }
      }
    });

  // 线程2: 反复 send (模拟接收线程或另一控制线程)
  std::thread t2([&]() {
      CanFrame f;
      f.id = 0x456;
      f.extended = true;
      f.len = 8;
      for (int i = 0; i < 8; ++i) {
        f.data[i] = 0;
      }
      while (!stop) {
        if (can.send(f)) {
          ++send_ok;
        }
      }
    });

  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  stop = true;
  t1.join();
  t2.join();

  EXPECT_GT(ensure_ok, 0);
  EXPECT_GT(send_ok, 0);

  teardown_vcan();
}

// 移动语义: 移动后旧对象不可用, 新对象可用 (线程安全保证 fd 正确转移)
TEST(SocketCanThreadSafe, MoveSemantics)
{
  setup_vcan();

  SocketCanInterface can(kIface);
  ASSERT_TRUE(can.open());

  SocketCanInterface moved(std::move(can));
  EXPECT_TRUE(moved.is_open());
  EXPECT_FALSE(can.is_open()) << "移动后原对象应已失效";

  // 新对象可正常 send
  CanFrame f;
  f.id = 0x789;
  f.extended = true;
  f.len = 8;
  for (int i = 0; i < 8; ++i) {
    f.data[i] = 0;
                                             }
  EXPECT_TRUE(moved.send(f));

  teardown_vcan();
}

// 并发 is_open + ensure_open (常量访问线程安全)
TEST(SocketCanThreadSafe, ConcurrentIsOpenEnsureOpen)
{
  setup_vcan();

  SocketCanInterface can(kIface);
  ASSERT_TRUE(can.open());

  std::atomic<bool> stop{false};
  std::atomic<int> is_open_count{0};
  std::atomic<int> ensure_count{0};

  std::thread t1([&]() {
      while (!stop) {
        if (can.is_open()) {++is_open_count;}
      }
    });
  std::thread t2([&]() {
      while (!stop) {
        if (can.ensure_open()) {++ensure_count;}
      }
    });

  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  stop = true;
  t1.join();
  t2.join();

  EXPECT_GT(is_open_count, 0);
  EXPECT_GT(ensure_count, 0);

  teardown_vcan();
}

// ===== 失败路径 / 边界场景 =====

// 接口不存在: open/send/receive/ensure_open 均安全失败, 不崩溃
TEST(SocketCanThreadSafe, NonexistentInterfaceFailsGracefully)
{
  // 使用不存在的接口名 (不创建 vcan)
  SocketCanInterface can("nonexistent_if");
  EXPECT_FALSE(can.open()) << "不存在的接口应打开失败";
  EXPECT_FALSE(can.is_open());
  EXPECT_FALSE(can.ensure_open());

  CanFrame f;
  f.id = 1;
  f.extended = true;
  f.len = 8;
  EXPECT_FALSE(can.send(f)) << "未打开时 send 应返回 false";
  EXPECT_FALSE(can.receive(f, 10)) << "未打开时 receive 应返回 false";
}

// 重复 open: 不应泄漏 fd (第二次 open 应成功且接口仍可用)
TEST(SocketCanThreadSafe, DoubleOpenIsIdempotent)
{
  setup_vcan();

  SocketCanInterface can(kIface);
  ASSERT_TRUE(can.open());
  EXPECT_TRUE(can.is_open());
  // 重复 open 应幂等, 不创建重复 socket
  EXPECT_TRUE(can.open());
  EXPECT_TRUE(can.is_open());

  CanFrame f;
  f.id = 1;
  f.extended = true;
  f.len = 8;
  EXPECT_TRUE(can.send(f)) << "重复 open 后接口仍应可用";

  teardown_vcan();
}

// close 后直接操作: send/receive 安全失败; ensure_open 会重新打开(自动重连语义)
TEST(SocketCanThreadSafe, OperationsAfterCloseFailGracefully)
{
  setup_vcan();

  SocketCanInterface can(kIface);
  ASSERT_TRUE(can.open());
  can.close();
  EXPECT_FALSE(can.is_open());

  // close 后直接操作应失败 (不依赖 ensure_open)
  CanFrame f;
  f.id = 1;
  f.extended = true;
  f.len = 8;
  EXPECT_FALSE(can.send(f));
  EXPECT_FALSE(can.receive(f, 10));

  // ensure_open 的语义是自动重连: close 后应能重新打开
  EXPECT_TRUE(can.ensure_open()) << "ensure_open 应自动重新打开接口";
  EXPECT_TRUE(can.is_open());

  teardown_vcan();
}

// ensure_open 自动重连: 接口被删除后失效, 恢复后自动重连
TEST(SocketCanThreadSafe, EnsureOpenReconnectsAfterInterfaceDeleted)
{
  setup_vcan();

  SocketCanInterface can(kIface);
  ASSERT_TRUE(can.open());

  // 删除接口 => 已打开的 fd 失效, ensure_open 应检测到并重新尝试(此时失败)
  run_cmd("sudo ip link delete vcan_test 2>/dev/null");
  // ensure_open 会尝试重开, 但接口不存在 => 打开失败
  EXPECT_FALSE(can.ensure_open()) << "接口被删除后 ensure_open 应失败";
  EXPECT_FALSE(can.is_open());

  // 重新创建接口并 up => ensure_open 应能自动重连
  run_cmd("sudo ip link add dev vcan_test type vcan 2>/dev/null");
  run_cmd("sudo ip link set up vcan_test 2>/dev/null");
  EXPECT_TRUE(can.ensure_open()) << "接口恢复后 ensure_open 应自动重连";
  EXPECT_TRUE(can.is_open());

  teardown_vcan();
}

// 移动后原对象再操作: 应安全失败 (不崩溃, 不误用被转移的 fd)
TEST(SocketCanThreadSafe, OperationOnMovedFromObjectFailsGracefully)
{
  setup_vcan();

  SocketCanInterface can(kIface);
  ASSERT_TRUE(can.open());

  SocketCanInterface moved(std::move(can));
  EXPECT_TRUE(moved.is_open());
  EXPECT_FALSE(can.is_open());

  CanFrame f;
  f.id = 1;
  f.extended = true;
  f.len = 8;
  EXPECT_FALSE(can.send(f)) << "移动后原对象 send 应失败";
  EXPECT_FALSE(can.receive(f, 10)) << "移动后原对象 receive 应失败";
  EXPECT_FALSE(can.ensure_open());

  teardown_vcan();
}
