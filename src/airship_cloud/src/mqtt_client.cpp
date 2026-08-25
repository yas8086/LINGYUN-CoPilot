// 灵云01号伴飞电脑 — MQTT 客户端封装实现
#include "airship_cloud/mqtt_client.hpp"

#include <mosquitto.h>

#include <atomic>
#include <cstdio>
#include <cstring>

namespace airship_cloud
{

MqttClient::MqttClient(
  const std::string & host, int port,
  const std::string & client_id,
  const std::string & username,
  const std::string & password,
  bool enable_tls,
  const std::string & ca_cert,
  bool tls_insecure)
: mosq_(nullptr), host_(host), port_(port),
  client_id_(client_id), username_(username), password_(password),
  enable_tls_(enable_tls), ca_cert_(ca_cert), tls_insecure_(tls_insecure)
{
}

MqttClient::~MqttClient()
{
  if (!lib_inited_) {
    return;
  }
  if (mosq_ != nullptr) {
    mosquitto_disconnect(mosq_);
    mosquitto_loop_stop(mosq_, true);
    mosquitto_destroy(mosq_);
    mosq_ = nullptr;
  }
  mosquitto_lib_cleanup();
}

bool MqttClient::connect()
{
  // 库级 init 只需一次; 用标志保证与析构的 cleanup() 严格配对(可多次 connect 不重复 init)
  if (!lib_inited_) {
    mosquitto_lib_init();
    lib_inited_ = true;
  }
  // 幂等保护: 重复 connect 时先释放旧句柄, 避免句柄泄漏与旧后台 loop 线程残留
  // (当前 cloud_node 仅构造时调用一次不会触发, 但 API 不应埋雷)
  if (mosq_ != nullptr) {
    mosquitto_disconnect(mosq_);
    mosquitto_loop_stop(mosq_, true);
    mosquitto_destroy(mosq_);
    mosq_ = nullptr;
    connected_ = false;
  }
  mosq_ = mosquitto_new(client_id_.c_str(), true, this);
  if (mosq_ == nullptr) {
    return false;
  }

  if (!username_.empty()) {
    mosquitto_username_pw_set(mosq_, username_.c_str(), password_.c_str());
  }

  // TLS 配置(EMQX Cloud Serverless 必须走 8883 + TLS)
  if (enable_tls_) {
    const char * cafile = ca_cert_.empty() ? nullptr : ca_cert_.c_str();
    const char * capath = ca_cert_.empty() ? "/etc/ssl/certs" : nullptr;
    // cafile/capath 两者择一: 指定了 ca_cert 用 cafile, 否则用系统 CA 目录
    int rc = mosquitto_tls_set(mosq_, cafile, capath, nullptr, nullptr, nullptr);
    if (rc != MOSQ_ERR_SUCCESS) {
      mosquitto_destroy(mosq_);
      mosq_ = nullptr;
      return false;
    }
    // tls_insecure=true 时跳过证书 CN/SAN 主机名校验(仅限内网/调试)
    rc = mosquitto_tls_insecure_set(mosq_, tls_insecure_ ? true : false);
    if (rc != MOSQ_ERR_SUCCESS) {
      mosquitto_destroy(mosq_);
      mosq_ = nullptr;
      return false;
    }
    // TLS ALPN 协商使用 SSLv23 兼容模式(libmosquitto 默认即可)
    const int rc_opts = mosquitto_tls_opts_set(mosq_, 1 /* cert_reqs: 验证对端证书 */, nullptr, nullptr);
    if (rc_opts != MOSQ_ERR_SUCCESS) {
      mosquitto_destroy(mosq_);
      mosq_ = nullptr;
      return false;
    }
  }

  mosquitto_connect_callback_set(mosq_, &MqttClient::on_connect_cb);
  mosquitto_disconnect_callback_set(mosq_, &MqttClient::on_disconnect_cb);
  mosquitto_log_callback_set(mosq_, &MqttClient::log_cb);  // 库级日志走自定义回调(stderr), 见 log_cb

  // 异步连接 + 后台循环(loop 负责自动重连)
  if (mosquitto_connect_async(mosq_, host_.c_str(), port_, 60) != MOSQ_ERR_SUCCESS) {
    mosquitto_destroy(mosq_);
    mosq_ = nullptr;
    return false;
  }
  mosquitto_loop_start(mosq_);
  return true;
}

bool MqttClient::publish(const std::string & topic, const std::string & payload, int qos)
{
  if (mosq_ == nullptr) {
    return false;
  }
  const int rc = mosquitto_publish(
    mosq_, nullptr, topic.c_str(), payload.size(), payload.data(), qos, false);
  return rc == MOSQ_ERR_SUCCESS;
}

bool MqttClient::is_connected() const
{
  return connected_;
}

void MqttClient::on_connect_cb(struct mosquitto * m, void * obj, int rc)
{
  (void)m;
  auto * self = static_cast<MqttClient *>(obj);
  if (rc == 0) {
    self->connected_ = true;
  } else {
    self->connected_ = false;
  }
}

void MqttClient::on_disconnect_cb(struct mosquitto * m, void * obj, int rc)
{
  (void)m;
  (void)rc;
  auto * self = static_cast<MqttClient *>(obj);
  self->connected_ = false;
}

void MqttClient::log_cb(struct mosquitto * m, void * obj, int level, const char * str)
{
  (void)m;
  (void)obj;
  // 桥接 libmosquitto 库级日志(含 TLS 握手/证书/认证错误)到 stderr, 由 journald 收集
  if (str != nullptr) {
    std::fprintf(stderr, "[mqtt][level=%d] %s\n", level, str);
  }
}

}  // namespace airship_cloud
