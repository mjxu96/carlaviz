/*
 * File: carla_session.h
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Thursday, 20th February 2020 2:53:07 pm
 */


#ifndef CARLAVIZ_SERVER_CARLA_SESSION_H_
#define CARLAVIZ_SERVER_CARLA_SESSION_H_

#include "xviz/server/xviz_session.h"
#include "xviz/io/glb_writer.h"
#include "backend/proxy/carla_proxy.h"
#include "backend/proxy/drawing_proxy.h"

#include "carla_handler.h"

namespace carlaviz {

class CarlaSession : public xviz::XVIZBaseSession {
public:
  CarlaSession(std::shared_ptr<websocketpp::connection<websocketpp::config::asio>> conn_ptr,
    std::weak_ptr<CarlaHandler> handler_weak_ptr, uint64_t interval_ms,
    const std::function<void(const std::unordered_map<std::string, bool>&)>& stream_settings_callback);

  void OnConnect() override;
  void Main() override;
  void OnDisconnect() override;
  
  bool SetSendStatus(bool new_status);
private:

  void OnMessage(websocketpp::connection_hdl hdl, std::shared_ptr<websocketpp::config::core::message_type> msg_ptr);
  bool is_error_{false};
  uint64_t interval_ms_{100};

  std::function<void(const std::unordered_map<std::string, bool>&)> stream_settings_callback_{};
  std::weak_ptr<CarlaHandler> handler_weak_ptr_;
  std::mutex send_lock_{};
  bool is_metadata_send_{false};
};
  
} // namespace carlaviz


#endif