/*
 * File: carla_handler.cpp
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Thursday, 20th February 2020 2:52:39 pm
 */

#include "backend/server/carla_session.h"
#include "backend/server/carla_handler.h"

using namespace carlaviz;
using namespace xviz;


CarlaHandler::CarlaHandler(const std::shared_ptr<CarlaProxy>& carla_proxy_ptr,
  const std::shared_ptr<DrawingProxy>& drawing_proxy_ptr, uint64_t interval_ms) :
    carla_proxy_ptr_(carla_proxy_ptr), drawing_proxy_ptr_(drawing_proxy_ptr), interval_ms_(interval_ms) {
  map_string_ = carla_proxy_ptr_->GetMapString();
  UpdateMetadata(carla_proxy_ptr_->GetMetadata());
  std::thread t(std::bind(
    &CarlaHandler::StartReadData, this
  ));
  t.detach();
}

std::shared_ptr<xviz::XVIZBaseSession> CarlaHandler::GetSession(const std::unordered_map<std::string, std::string>& params,
  std::shared_ptr<websocketpp::connection<websocketpp::config::asio>> conn_ptr) {
  auto session_ptr = std::make_shared<CarlaSession>(conn_ptr, weak_from_this(), interval_ms_, stream_settings_callback_);
  children_sessions_[cnt_] = session_ptr;
  cnt_++;
  return session_ptr;
}

void CarlaHandler::StartReadData() {
  while (true) {
    // auto start = std::chrono::high_resolution_clock::now();
    auto xviz_builder = carla_proxy_ptr_->GetUpdateData();
    drawing_proxy_ptr_->AddDrawings(xviz_builder);
    std::string output;
    xviz::XVIZGLBWriter writer;
    writer.WriteMessage(output, xviz_builder.GetMessage());

    auto data_end = std::chrono::high_resolution_clock::now();

    std::lock_guard lock_g(update_lock_);
    update_str_ = std::move(output);
    // auto end = std::chrono::high_resolution_clock::now();
    // CARLAVIZ_LOG_INFO("Data: %.3f   lock: %.3f", std::chrono::duration<double, std::milli>(data_end - start).count(),
      // std::chrono::duration<double, std::milli>(end - data_end).count());
  }
}

std::string CarlaHandler::GetMetadata() {
  std::lock_guard lock_g(metadata_lock_);
  return updated_metadata_without_map_;
}

std::string CarlaHandler::GetMetadataWithMap() {
  std::lock_guard lock_g(metadata_lock_);
  return updated_metadata_with_map_;
}

std::string CarlaHandler::GetUpdateData() {
  std::lock_guard lock_g(update_lock_);
  return update_str_;
}

void CarlaHandler::UpdateMetadata(const std::string& metadata_str) {
  metadata_lock_.lock();
  updated_metadata_without_map_ = metadata_str;
  updated_metadata_with_map_ = metadata_str;
  updated_metadata_with_map_.pop_back();
  updated_metadata_with_map_ += ",\"map\": " + map_string_;
  updated_metadata_with_map_ += "}";
  metadata_lock_.unlock();

  std::vector<size_t> to_delete_children;
  for (const auto& [id, session_ptr] : children_sessions_) {
    auto carla_session_ptr = std::dynamic_pointer_cast<CarlaSession>(session_ptr);
    if (carla_session_ptr == nullptr) {
      CARLAVIZ_LOG_ERROR("Child session should not be nullptr");
      continue;
    }
    if (carla_session_ptr->SetSendStatus(false)) {
      to_delete_children.push_back(id);
    }
  }

  for (const auto id : to_delete_children) {
    children_sessions_.erase(id);
  }
}
void CarlaHandler::SetStreamSettingsCallback(
  const std::function<void(const std::unordered_map<std::string, bool>&)>& stream_settings_callback) {
  stream_settings_callback_ = stream_settings_callback;
}