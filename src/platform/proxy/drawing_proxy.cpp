/*
 * File: drawing_proxy.cpp
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Monday, 7th October 2019 5:53:57 pm
 */

#include "platform/proxy/drawing_proxy.h"

using namespace mellocolate;
using tcp = boost::asio::ip::tcp;
namespace websocket = boost::beast::websocket;

using Json = nlohmann::json;

DrawingProxy::DrawingProxy(uint16_t listen_port) : listen_port_(listen_port) {}

void DrawingProxy::StartListen() {
  auto t = std::thread(&DrawingProxy::Accept, this);
  t.detach();
}

XVIZPrimitiveBuider DrawingProxy::GetPolyLines() {
  XVIZPrimitiveBuider polyline_builder("/planning/trajectory");
  polyline_update_lock_.lock();
  for (const auto& polyline_pair : polylines_) {
    auto polyline = polyline_pair.second;
    polyline_builder.AddPolyLine(XVIZPrimitivePolyLineBuilder(polyline.points)
                                        .AddColor(polyline.color)
                                        .AddWidth(polyline.width));
  }
  polyline_update_lock_.unlock();
  return polyline_builder;
}

void DrawingProxy::Accept() {
  LOG_INFO("Waiting for a drawing client to connect. Listening to port %u....",
           listen_port_);
  try {
    boost::asio::io_context ioc{1};
    tcp::acceptor acceptor{ioc, tcp::endpoint(tcp::v4(), listen_port_)};
    for (;;) {
      tcp::socket socket{ioc};
      acceptor.accept(socket);
      std::thread t = std::thread{
          std::bind(&DrawingProxy::AddClient, this, std::move(socket))};
      t.detach();
    }
  } catch (const std::exception& e) {
    LOG_ERROR("%s", e.what());
  }
}

void DrawingProxy::AddClient(
    boost::asio::basic_stream_socket<boost::asio::ip::tcp>& socket) {
  add_client_lock_.lock();
  int id = client_max_id_;
  client_max_id_++;
  add_client_lock_.unlock();
  auto client_ptr =
      boost::make_shared<websocket::stream<tcp::socket>>(std::move(socket));
  try {
    client_ptr->accept();
    LOG_INFO("Drawing client %u connected", id);

    for (;;) {
      boost::beast::multi_buffer buffer;
      boost::beast::error_code ec;
      size_t size = client_ptr->read(buffer, ec);
      if (ec.value() != 0) {
        LOG_INFO("Drawing client %u connection closed", id);
        polyline_update_lock_.lock();
        polylines_.erase(id);
        polyline_update_lock_.unlock();
        return;
      }
      std::ostringstream os;
      os << boost::beast::buffers(buffer.data());

      auto polyline = DecodeToPoints(os.str());
      polyline_update_lock_.lock();
      polylines_[id] = polyline;
      polyline_update_lock_.unlock();
    }

  } catch (boost::system::system_error const& se) {
    polyline_update_lock_.lock();
    polylines_.erase(id);
    polyline_update_lock_.unlock();
    if (se.code() != websocket::error::closed) {
      LOG_ERROR("Drawing client read error %s", se.what());
    } else {
      LOG_INFO("Drawing client %u connection closed", id);
    }
  }
}

polyline DrawingProxy::DecodeToPoints(const std::string& str) {
  polyline polyline;
  Json decoded_json;
  try {
    decoded_json = Json::parse(str);
  } catch (std::exception const& e) {
    LOG_ERROR(
        "Receive bad json data from drawing client, ignore this data message, "
        "the message is %s",
        str.c_str());
    return polyline;
  }

  for (auto itr = decoded_json.begin(); itr != decoded_json.end(); itr++) {
    std::string key = itr.key();
    if (key == "vertices") {
      if (itr.value().is_array()) {
        for (const auto& point : itr.value()) {
          if (point.is_array()) {
            auto point_vertices = point.get<std::vector<double>>();
            if (point_vertices.size() == 3) {
              polyline.points.emplace_back(point_vertices[0], -point_vertices[1],
                                  point_vertices[2]);
            } else {
              LOG_ERROR("Point should have 3 coordinates");
              break;
            }
          } else {
            LOG_ERROR(
                "Point in vertices entry in drawing message is not array");
            break;
          }
        }
      } else {
        LOG_ERROR("Vertices entry in drawing message is not array");
        break;
      }
    }
    if (key == "color") {
      polyline.color = itr.value().get<std::string>();
    }
    if (key == "width") {
      polyline.width = itr.value().get<double>();
    }
  }
  return polyline;
}