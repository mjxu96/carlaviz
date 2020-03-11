/*
 * File: carla_handler.h
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Thursday, 20th February 2020 2:53:00 pm
 */

#ifndef MELLOCOLATE_SERVER_CARLA_HANDLER_H_
#define MELLOCOLATE_SERVER_CARLA_HANDLER_H_

#include "server/xviz_handler.h"
#include "platform/proxy/carla_proxy.h"
#include "platform/proxy/drawing_proxy.h"

namespace mellocolate {

class CarlaHandler : public xviz::XVIZBaseHandler, public std::enable_shared_from_this<CarlaHandler> {
public:
  CarlaHandler(const std::shared_ptr<CarlaProxy>& carla_proxy_ptr,
    const std::shared_ptr<DrawingProxy>& drawing_proxy_ptr, uint64_t interval_ms=100u);
  std::shared_ptr<xviz::XVIZBaseSession> GetSession(const std::unordered_map<std::string, std::string>& params,
  std::shared_ptr<websocketpp::connection<websocketpp::config::asio>> conn_ptr) override;

  std::string GetUpdateData();
  std::string GetMetaData();
private:
  void StartReadData();

  std::shared_ptr<CarlaProxy> carla_proxy_ptr_{nullptr};
  std::shared_ptr<DrawingProxy> drawing_proxy_ptr_{nullptr};
  uint64_t interval_ms_{100u};

  std::mutex metadata_lock_{};
  std::string metadata_str_{};

  std::mutex update_lock_{};
  std::string update_str_{};
};
  
} // namespace mellocolate



#endif