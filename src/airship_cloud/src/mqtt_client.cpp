// 灵云01号伴飞电脑 — MQTT 客户端封装实现
#include "airship_cloud/mqtt_client.hpp"

#include <mosquitto.h>

#include <atomic>
#include <cstring>

namespace airship_cloud
{

MqttClient::MqttClient(
  const std::string & host, int port,
  const std::string & client_id,
  const std::string & username,
  const std::string & password)
: mosq_(nullptr), host_(host), port_(port),
  client_id_(client_id), username_(username), password_(password)
{
}

MqttClient::~MqttClient()
{
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
  mosquitto_lib_init();
  mosq_ = mosquitto_new(client_id_.c_str(), true, this);
  if (mosq_ == nullptr) {
    return false;
  }

  if (!username_.empty()) {
    mosquitto_username_pw_set(mosq_, username_.c_str(), password_.c_str());
  }

  mosquitto_connect_callback_set(mosq_, &MqttClient::on_connect_cb);
  mosquitto_disconnect_callback_set(mosq_, &MqttClient::on_disconnect_cb);

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

}  // namespace airship_cloud
