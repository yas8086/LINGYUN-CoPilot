// 灵云01号伴飞电脑 — MQTT 客户端封装 (基于 libmosquitto)
//
// 用途: 4G 网络传输设备汇总数据到云。封装连接/发布/自动重连, 供 cloud_node 使用。
// 传输层封装, 与业务打包(airship_link::json_packer)分离。
#ifndef AIRSHIP_CLOUD__MQTT_CLIENT_HPP_
#define AIRSHIP_CLOUD__MQTT_CLIENT_HPP_

#include <atomic>
#include <string>

// 前置声明 libmosquitto 句柄类型
struct mosquitto;

namespace airship_cloud
{

// MQTT 客户端 (RAII, 不可拷贝)
class MqttClient
{
public:
  // host: broker 地址(IP或域名); port: 默认 1883
  // enable_tls: 是否启用 TLS (EMQX Cloud Serverless 必须 true, 端口 8883)
  // ca_cert: CA 证书路径, 空字符串则使用系统默认 CA 证书
  // tls_insecure: true=跳过证书主机名校验(仅调试), false=正常校验
  MqttClient(
    const std::string & host, int port,
    const std::string & client_id,
    const std::string & username = "",
    const std::string & password = "",
    bool enable_tls = false,
    const std::string & ca_cert = "",
    bool tls_insecure = false);
  ~MqttClient();

  MqttClient(const MqttClient &) = delete;
  MqttClient & operator=(const MqttClient &) = delete;

  // 异步连接并启动后台网络循环(自动重连), 返回是否成功启动
  bool connect();

  // 发布消息到指定 topic, 返回是否成功入队
  bool publish(const std::string & topic, const std::string & payload, int qos = 1);

  // 是否已连接 broker
  bool is_connected() const;

private:
  static void on_connect_cb(struct mosquitto * m, void * obj, int rc);
  static void on_disconnect_cb(struct mosquitto * m, void * obj, int rc);
  // 库级日志回调: 桥接 TLS 握手失败/认证失败等内部错误到 stderr(journald),
  // 否则现场只能看到"未连接"节流警告, 排障困难。
  static void log_cb(struct mosquitto * m, void * obj, int level, const char * str);

  struct mosquitto * mosq_;
  std::string host_;
  int port_;
  std::string client_id_;
  std::string username_;
  std::string password_;
  bool enable_tls_;
  std::string ca_cert_;
  bool tls_insecure_;
  // libmosquitto 库级 init/cleanup 是否已配对: 仅在 connect() 首次调用 init 后,
  // 析构才执行 cleanup(), 避免从未连接的对象析构时对未 init 的库做 cleanup。
  bool lib_inited_ = false;
  // connected_ 由 libmosquitto 后台 loop 线程写入, 主线程经 is_connected() 读取,
  // 必须用原子类型避免数据竞争。
  std::atomic<bool> connected_ = false;
};

}  // namespace airship_cloud

#endif  // AIRSHIP_CLOUD__MQTT_CLIENT_HPP_
