/*
 * File: carla_session.cpp
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Thursday, 20th February 2020 2:52:50 pm
 */

#include "platform/server/carla_session.h"

using namespace mellocolate;
using namespace xviz;

CarlaSession::CarlaSession(std::shared_ptr<websocketpp::connection<websocketpp::config::asio>> conn_ptr,
    std::shared_ptr<CarlaProxy> carla_proxy_ptr, std::shared_ptr<DrawingProxy> drawing_proxy_ptr, uint64_t interval_ms) : 
    XVIZBaseSession(conn_ptr), carla_proxy_ptr_(carla_proxy_ptr), drawing_proxy_ptr_(drawing_proxy_ptr), interval_ms_(interval_ms) {
}

void CarlaSession::OnConnect() {
  if (carla_proxy_ptr_ == nullptr) {
    LOG_ERROR("Carla proxy is nullptr");
    is_error_ = true;
    return;
  }
  std::string metadata = carla_proxy_ptr_->GetMetaData();
  auto err_code = conn_ptr_->send(std::move(metadata));
  if (err_code) {
    LOG_INFO("Cannot send metadata");
    is_error_ = true;
    return;
  }
  LOG_INFO("Frontend connected");
}

void CarlaSession::Main() {
  if (is_error_) {
    return;
  }
  if (drawing_proxy_ptr_ == nullptr) {
    LOG_ERROR("Drawing proxy is nullptr");
    return;
  }
  while (true) {
    // LOG_ERROR("Drawing once");
    auto data_start = std::chrono::high_resolution_clock::now();
    auto builder = carla_proxy_ptr_->GetUpdateData();
    auto data_end = std::chrono::high_resolution_clock::now();
    drawing_proxy_ptr_->AddDrawings(builder);
    auto drawing_end = std::chrono::high_resolution_clock::now();
    std::string output;
    XVIZGLBWriter writer;
    writer.WriteMessage(output, builder.GetMessage());
    auto writer_end = std::chrono::high_resolution_clock::now();
    auto err_code = conn_ptr_->send(std::move(output), websocketpp::frame::opcode::BINARY);
    if (err_code) {
      LOG_ERROR("Sending error, exit");
      return;
    }
    auto send_finish = std::chrono::high_resolution_clock::now();
    LOG_ERROR("Data spend %.3f, drawing spend: %.3f, writer spend: %.3f, send spend %.3f", 
      std::chrono::duration<double, std::milli>(data_end - data_start).count(),
      std::chrono::duration<double, std::milli>(drawing_end - data_end).count(),
      std::chrono::duration<double, std::milli>(writer_end - drawing_end).count(),
      std::chrono::duration<double, std::milli>(send_finish - writer_end).count());
    std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms_));
  }
}

void CarlaSession::OnDisconnect() {
  LOG_INFO("Frontend disconnected");
}