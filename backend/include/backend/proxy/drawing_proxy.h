/*
 * File: drawing_proxy.h
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Monday, 7th October 2019 5:54:33 pm
 */

#ifndef CARLAVIZ_DRAWING_PROXY_H_
#define CARLAVIZ_DRAWING_PROXY_H_

#include "backend/utils/def.h"
#include "backend/utils/json.hpp"
#include "backend/utils/macrologger.h"

#include "xviz/builder/xviz_builder.h"

#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/unordered_map.hpp>

#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

namespace carlaviz {

enum class DrawingType {
  LINE = 0u,
  POINT = 1u,
  TEXT = 2u,
};

struct polyline {
  std::string color{"#00FF00"};
  double width{2.5};
  std::vector<double> points{};
};

struct point {
  std::vector<double> points{};
};

struct text {
  std::string color{"#FFFFFF"};
  double size{13.0};
  std::string message{};
  std::vector<double> position{};
};

class DrawingProxy {
 public:
  DrawingProxy(uint16_t listen_port);
  void AddDrawings(xviz::XVIZBuilder& xviz);
  void StartListen();

 private:
  uint16_t listen_port_{8089u};
  void Accept();
  void AddClient(
      boost::asio::basic_stream_socket<boost::asio::ip::tcp>& socket);

  void Decode(const std::string& str, uint32_t id);
  void CleanUpDrawing(uint32_t id);

  std::mutex polyline_update_lock_{};
  std::unordered_map<uint32_t, std::vector<polyline>> polylines_{};

  std::mutex point_update_lock_{};
  std::unordered_map<uint32_t, std::vector<point>> points_{};

  std::mutex text_update_lock_{};
  std::unordered_map<uint32_t, std::vector<text>> texts_{};

  std::mutex add_client_lock_{};
  uint32_t client_max_id_{0u};
};

}  // namespace carlaviz

#endif