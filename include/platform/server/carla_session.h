/*
 * File: carla_session.h
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Thursday, 20th February 2020 2:53:07 pm
 */


#ifndef MELLOCOLATE_SERVER_CARLA_SESSION_H_
#define MELLOCOLATE_SERVER_CARLA_SESSION_H_

#include "server/xviz_session.h"
#include "io/glb_writer.h"
#include "platform/proxy/carla_proxy.h"
#include "platform/proxy/drawing_proxy.h"

namespace mellocolate {

class CarlaSession : public xviz::XVIZBaseSession {
public:
  CarlaSession(std::shared_ptr<websocketpp::connection<websocketpp::config::asio>> conn_ptr,
    std::shared_ptr<CarlaProxy> carla_proxy_ptr, std::shared_ptr<DrawingProxy> drawing_proxy_ptr, 
    uint64_t interval_ms);

  void OnConnect() override;
  void Main() override;
  void OnDisconnect() override;
private:
  bool is_error_{false};
  std::shared_ptr<CarlaProxy> carla_proxy_ptr_{nullptr};
  std::shared_ptr<DrawingProxy> drawing_proxy_ptr_{nullptr};
  uint64_t interval_ms_{50};
};
  
} // namespace mellocolate


#endif