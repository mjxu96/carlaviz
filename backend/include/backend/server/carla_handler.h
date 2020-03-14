/*
 * File: carla_handler.h
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Thursday, 20th February 2020 2:53:00 pm
 */

#ifndef CARLAVIZ_SERVER_CARLA_HANDLER_H_
#define CARLAVIZ_SERVER_CARLA_HANDLER_H_

#include "xviz/server/xviz_handler.h"
#include "backend/proxy/carla_proxy.h"
#include "backend/proxy/drawing_proxy.h"

namespace carlaviz {

class CarlaHandler : public xviz::XVIZBaseHandler, public std::enable_shared_from_this<CarlaHandler> {
public:
  CarlaHandler(const std::shared_ptr<CarlaProxy>& carla_proxy_ptr,
    const std::shared_ptr<DrawingProxy>& drawing_proxy_ptr, uint64_t interval_ms=100u);
  std::shared_ptr<xviz::XVIZBaseSession> GetSession(const std::unordered_map<std::string, std::string>& params,
  std::shared_ptr<websocketpp::connection<websocketpp::config::asio>> conn_ptr) override;

  std::string GetUpdateData();
  std::string GetMetadata();
  std::string GetMetadataWithMap();
  void UpdateMetadata(const std::string& metadata_str);
  void SetStreamSettingsCallback(const std::function<void(const std::unordered_map<std::string, bool>&)>& stream_settings_callback);
private:
  void StartReadData();

  std::function<void(const std::unordered_map<std::string, bool>&)> stream_settings_callback_{};

  std::unordered_map<size_t, std::shared_ptr<xviz::XVIZBaseSession>> children_sessions_{};
  size_t cnt_ = 0u;

  std::shared_ptr<CarlaProxy> carla_proxy_ptr_{nullptr};
  std::shared_ptr<DrawingProxy> drawing_proxy_ptr_{nullptr};
  uint64_t interval_ms_{100u};

  std::mutex metadata_lock_{};
  std::string updated_metadata_with_map_{};
  std::string updated_metadata_without_map_{};
  std::string map_string_{};

  std::mutex update_lock_{};
  std::string update_str_{};
};
  
} // namespace carlaviz



#endif