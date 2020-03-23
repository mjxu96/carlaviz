/*
 * File: drawing_proxy.cpp
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Monday, 7th October 2019 5:53:57 pm
 */

#include "backend/proxy/drawing_proxy.h"

using namespace carlaviz;
using namespace xviz;
using tcp = boost::asio::ip::tcp;
namespace websocket = boost::beast::websocket;

using Json = nlohmann::json;

DrawingProxy::DrawingProxy(uint16_t listen_port) : listen_port_(listen_port) {}

void DrawingProxy::StartListen() {
  auto t = std::thread(&DrawingProxy::Accept, this);
  t.detach();
}

void DrawingProxy::AddDrawings(xviz::XVIZBuilder& xviz) {
  XVIZPrimitiveBuilder& polyline_primitive_builder = xviz.Primitive("/drawing/polylines");
  // XVIZBuilder polyline_builder("/planning/trajectory");
  polyline_update_lock_.lock();
  for (const auto& polylines_pair : polylines_) {
    for (const auto& polyline : polylines_[polylines_pair.first]) {
      std::string style = std::string("{\"stroke_color\":\"") + polyline.color + std::string("\", \"stroke_width\":") + std::to_string(polyline.width) + std::string("}");
      polyline_primitive_builder.Polyline(polyline.points)
        .Style(style);
      // polyline_primitive_builder.Polyline()
      // polyline_builder.AddPolyLine(XVIZPrimitivePolyLineBuilder(polyline.points)
      //                                   .AddColor(polyline.color)
      //                                   .AddWidth(polyline.width));
    }
  }
  polyline_update_lock_.unlock();

  XVIZPrimitiveBuilder& point_primitive_builder = xviz.Primitive("/drawing/points");
  point_update_lock_.lock();
  for (const auto& points_pair : points_) {
    for (const auto& point : points_[points_pair.first]) {
      point_primitive_builder.Points(point.points);
    }
  }
  point_update_lock_.unlock();

  XVIZPrimitiveBuilder& text_primitive_builder = xviz.Primitive("/drawing/texts");
  text_update_lock_.lock();
  for (const auto& text_pair : texts_) {
    for (const auto& text : texts_[text_pair.first]) {
      std::string style = std::string("{\"fill_color\":\"") + text.color + std::string("\", \"text_size\":") + std::to_string(text.size) + std::string("}");
      text_primitive_builder.Text(text.message)
        .Position(text.position).Style(style);
    }
  }
  text_update_lock_.unlock();
  // return polyline_builder;
}

void DrawingProxy::Accept() {
  CARLAVIZ_LOG_INFO("Waiting for a drawing client to connect. Listening to port %u....",
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
    CARLAVIZ_LOG_ERROR("%s", e.what());
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
    CARLAVIZ_LOG_INFO("Drawing client %u connected", id);

    for (;;) {
      boost::beast::multi_buffer buffer;
      boost::beast::error_code ec;
      size_t size = client_ptr->read(buffer, ec);
      if (ec.value() != 0) {
        CARLAVIZ_LOG_INFO("Drawing client %u connection closed", id);
        CleanUpDrawing(id);
        return;
      }
      // std::ostringstream os;
      // os << boost::beast::buffers(buffer.data());
      std::string str = boost::beast::buffers_to_string(buffer.data());

      // auto polyline = DecodeToPolylines(os.str());
      // polyline_update_lock_.lock();
      // polylines_[id] = polyline;
      // polyline_update_lock_.unlock();
      Decode(str, id);
    }

  } catch (boost::system::system_error const& se) {
    CleanUpDrawing(id);
    if (se.code() != websocket::error::closed) {
      CARLAVIZ_LOG_ERROR("Drawing client read error %s", se.what());
    } else {
      CARLAVIZ_LOG_INFO("Drawing client %u connection closed", id);
    }
  }
}

void DrawingProxy::Decode(const std::string& str, uint32_t id) {
  Json decoded_json;
  try {
    decoded_json = Json::parse(str);
  } catch (std::exception const& e) {
    CARLAVIZ_LOG_ERROR(
        "Receive bad json data from drawing client, ignore this data message, "
        "the message is %s",
        str.c_str());
    return;
  }

  std::string type_str = decoded_json["type"];
  DrawingType type = DrawingType::LINE;
  if (type_str == "point") {
    type = DrawingType::POINT;
  } else if (type_str == "text") {
    type = DrawingType::TEXT;
  }

  if (type == DrawingType::LINE) {
    std::vector<polyline> polylines;
    std::string color = "#FF0000";
    double width = 1.0;
    for (auto itr = decoded_json.begin(); itr != decoded_json.end(); itr++) {
      std::string key = itr.key();
      if (key == "vertices") {
        if (itr.value().is_array()) {
          for (const auto& line : itr.value()) {
            if (line.is_array()) {
              polyline polyline;
              for (const auto& point : line) {
                if (point.is_array()) {
                  auto point_vertices = point.get<std::vector<double>>();
                  if (point_vertices.size() == 3) {
                    polyline.points.push_back(point_vertices[0]);
                    polyline.points.push_back(-point_vertices[1]);
                    polyline.points.push_back(point_vertices[2]);
                  } else {
                    CARLAVIZ_LOG_ERROR("Point should have 3 coordinates");
                    break;
                  }
                } else {
                  CARLAVIZ_LOG_ERROR(
                      "Point in vertices entry in drawing message is not array");
                  break;
                }
              }
              polylines.push_back(std::move(polyline));
            } else {
              CARLAVIZ_LOG_ERROR("One client should send multiple lines");
            }
            
          }
        } else {
          CARLAVIZ_LOG_ERROR("Vertices entry in drawing message is not array");
          break;
        }
      }
      if (key == "color") {
        color = itr.value().get<std::string>();
      }
      if (key == "width") {
        width = itr.value().get<double>();
      }
    }
    for (auto& polyline : polylines) {
      polyline.color = color;
      polyline.width = width;
    }
    polyline_update_lock_.lock();
    polylines_[id] = std::move(polylines);
    polyline_update_lock_.unlock();

  } else if (type == DrawingType::POINT) {

    std::vector<point> points;
    for (auto itr = decoded_json.begin(); itr != decoded_json.end(); itr++) {
      std::string key = itr.key();
      if (key == "points") {
        if (itr.value().is_array()) {
          for (const auto& line : itr.value()) {
            if (line.is_array()) {
              point p;
              for (const auto& point : line) {
                if (point.is_array()) {
                  auto point_vertices = point.get<std::vector<double>>();
                  if (point_vertices.size() == 3) {
                    p.points.push_back(point_vertices[0]);
                    p.points.push_back(-point_vertices[1]);
                    p.points.push_back(point_vertices[2]);
                  } else {
                    CARLAVIZ_LOG_ERROR("Point should have 3 coordinates");
                    break;
                  }
                } else {
                  CARLAVIZ_LOG_ERROR(
                      "Point in vertices entry in drawing message is not array");
                  break;
                }
              }
              points.push_back(std::move(p));
            } else {
              CARLAVIZ_LOG_ERROR("One client should send multiple lines");
            }
            
          }
        } else {
          CARLAVIZ_LOG_ERROR("Vertices entry in drawing message is not array");
          break;
        }
      }
    }
    point_update_lock_.lock();
    points_[id] = std::move(points);
    point_update_lock_.unlock();

  } else if (type == DrawingType::TEXT) {
    std::vector<text> texts;
    std::string color = "#FFFFFF";
    double size = 13.0;
    for (auto itr = decoded_json.begin(); itr != decoded_json.end(); itr++) {
      std::string key = itr.key();
      if (key == "text") {
        if (itr.value().is_array()) {
          for (const auto& tt : itr.value()) {
            text t;
            t.message = tt["message"].get<std::string>();
            auto pos = tt["position"].get<std::vector<double>>();
            if (pos.size() != 3) {
              CARLAVIZ_LOG_ERROR("Position should have 3 coordinates");
              break;
            } else {
              pos[1] = -pos[1];
              t.position = std::move(pos);
            }
            texts.push_back(std::move(t));
          }
        } else {
          CARLAVIZ_LOG_ERROR("Input texts should be array");
        }
      }

      if (key == "color") {
        color = itr.value().get<std::string>();
      }

      if (key == "size") {
        size = itr.value().get<double>();
      }
    }

    for (auto& text :texts) {
      text.color = color;
      text.size = size;
    }

    text_update_lock_.lock();
    texts_[id] = std::move(texts);
    text_update_lock_.unlock();
  }
}

void DrawingProxy::CleanUpDrawing(uint32_t id) {
    polyline_update_lock_.lock();
    polylines_.erase(id);
    polyline_update_lock_.unlock();

    point_update_lock_.lock();
    points_.erase(id);
    point_update_lock_.unlock();

    text_update_lock_.lock();
    texts_.erase(id);
    text_update_lock_.unlock();
}
