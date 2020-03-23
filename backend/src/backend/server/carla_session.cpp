/*
 * File: carla_session.cpp
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Thursday, 20th February 2020 2:52:50 pm
 */

#include "backend/server/carla_session.h"

using namespace carlaviz;
using namespace xviz;
using Json = nlohmann::json;

CarlaSession::CarlaSession(std::shared_ptr<websocketpp::connection<websocketpp::config::asio>> conn_ptr,
    std::weak_ptr<carlaviz::CarlaHandler> handler_weak_ptr, uint64_t interval_ms,
    const std::function<void(const std::unordered_map<std::string, bool>&)>& stream_settings_callback) :
    XVIZBaseSession(conn_ptr), handler_weak_ptr_(handler_weak_ptr), interval_ms_(interval_ms),
    stream_settings_callback_(stream_settings_callback) {
  conn_ptr_->set_message_handler(std::bind(&CarlaSession::OnMessage, this, std::placeholders::_1, 
    std::placeholders::_2));
}

void CarlaSession::OnConnect() {
  auto err_code = conn_ptr_->send(handler_weak_ptr_.lock()->GetMetadataWithMap());
  if (err_code) {
    CARLAVIZ_LOG_INFO("Cannot send metadata");
    is_error_ = true;
    return;
  }
  CARLAVIZ_LOG_INFO("Frontend connected");
  std::lock_guard lock_g(send_lock_);
  is_metadata_send_ = true;
}

void CarlaSession::Main() {
  if (is_error_) {
    return;
  }
  while (true) {
    send_lock_.lock();
    if (!is_metadata_send_) {
      is_metadata_send_ = true;
      auto err_c = conn_ptr_->send(handler_weak_ptr_.lock()->GetMetadata());
      if (err_c) {
        CARLAVIZ_LOG_INFO("Cannot send metadata without map");
        send_lock_.unlock();
        is_error_ = true;
        return;
      }
    }
    send_lock_.unlock();

    auto err_code = conn_ptr_->send(handler_weak_ptr_.lock()->GetUpdateData(), websocketpp::frame::opcode::BINARY);
    if (err_code) {
      if (err_code.message() != "invalid state") {
        CARLAVIZ_LOG_ERROR("Cannot send update data, %s", err_code.message().c_str());
      }
      is_error_ = true;
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms_));
  }
}

void CarlaSession::OnDisconnect() {
  is_error_ = true;
  CARLAVIZ_LOG_INFO("Frontend disconnected");
}

void CarlaSession::OnMessage(websocketpp::connection_hdl hdl, std::shared_ptr<websocketpp::config::core::message_type> msg_ptr) {
  try {
    Json setting_json = Json::parse(msg_ptr->get_payload());
    stream_settings_callback_(setting_json.get<std::unordered_map<std::string, bool>>());
  } catch (const std::exception& e) {
    CARLAVIZ_LOG_WARNING("When paring %s, get error: %s", msg_ptr->get_payload().c_str(),
      e.what());
  }
}

bool CarlaSession::SetSendStatus(bool new_status) {
  std::lock_guard lock_g(send_lock_);
  is_metadata_send_ = new_status;
  return is_error_;
}