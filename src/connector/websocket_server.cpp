#include "connector/websocket_server.h"

using namespace std::chrono_literals;
using namespace std::string_literals;

namespace rothberg {

std::string ReadGeoJsonFromFile(std::string file_name) {
  std::ifstream t(file_name);
  std::stringstream buffer;
  buffer << t.rdbuf();
  return buffer.str();
}


void WebsocketServer::Init(boost::shared_ptr<rothberg::utils::Package> package_ptr, 
  boost::shared_ptr<std::mutex> package_mutex,
  std::string host, uint16_t port) {
  package_ptr_ = std::move(package_ptr);
  package_mutex_ = std::move(package_mutex);
  host_ = std::move(host);
  port_ = port;
}

void WebsocketServer::Run() {
  try {
    if (host_.size() == 0) {
      std::cerr << "[Websocket Server Error] Please indicate ip address" << std::endl;
      return;
    }

    std::cout << host_ << std::endl;
    const auto address =  boost::asio::ip::make_address(host_);
    const auto port = port_;

    boost::asio::io_context ioc{1};
    boost::asio::ip::tcp::acceptor accetor{ioc, {address, port}};

    std::cout << "[Websocket Server Log] Start listening to " << address << ":" << port << std::endl;

    while (true) {
      boost::asio::ip::tcp::socket socket{ioc};
      accetor.accept(socket);
      std::cout << "[Websocket Server Log] Client connected" << std::endl;
      std::thread t = std::thread{std::bind(
        &WebsocketServer::DoSession, this,
        std::move(socket))};
      t.detach();
      threads_.push_back(std::move(t));
    }

  } catch (const std::exception& e) {
    std::cerr << "[Websocket Server Error] Error: " << e.what() << std::endl;
    return;
  }
}

void WebsocketServer::DoSession(boost::asio::basic_stream_socket<boost::asio::ip::tcp>& socket) {

  try {
    boost::beast::websocket::stream<boost::asio::ip::tcp::socket> ws{std::move(socket)};

    ws.accept();
    boost::beast::multi_buffer init_buffer;
    std::string init_meda_data = GetInitMetaDataJson();
    boost::beast::ostream(init_buffer) << init_meda_data;
    ws.write(init_buffer.data());
    for(;;) {
      boost::beast::multi_buffer buffer;
   
      std::string update_data = GetLiveDataJson();
      boost::beast::ostream(buffer) << update_data;
      ws.write(buffer.data());
      std::this_thread::sleep_for(40ms);
    }
  } catch(boost::system::system_error const& se) {
    if(se.code() != boost::beast::websocket::error::closed) {
      std::cerr << "Error: " << se.code().message() << std::endl;
    } else {
      std::cout << "Session closed" << std::endl;
    }
  } catch(std::exception const& e) {
    std::cerr << "Error: " << e.what() << std::endl;
  }

}

std::string WebsocketServer::GetInitMetaDataJson() {
  std::string json_str = "{\"type\": \"xviz/metadata\", \"data\": { \"version\": \"2.0.0\", \"streams\": { \"/vehicle_pose\": { \"category\": \"pose\" }, \"/object/shape\": { \"category\": \"primitive\", \"coordinate\": \"IDENTITY\", \"stream_style\": { \"fill_color\": \"#fb0\", \"height\": 1.5, \"extruded\": true }, \"primitive_type\": \"polygon\" } } } }";
  nlohmann::json json = nlohmann::json::parse(json_str);
  json["data"]["map"] = ReadGeoJsonFromFile("map.geojson");
  return json.dump();
  //return nlohmann::json::parse(json_str).dump();

}

std::string WebsocketServer::GetLiveDataJson() {
  std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
  double now_time = now.time_since_epoch().count() / 1e9;
  std::string now_time_str = std::to_string(now_time);//ss.str();
  /*
  std::string json_str = std::string("{ \"type\": \"xviz/state_update\", \"data\":{ \"update_type\": \"snapshot\", \"updates\": [ { \"timestamp\": ") + 
    now_time_str + std::string(", \"poses\": { \"/vehicle_pose\": { \"timestamp\":") +  now_time_str + std::string(", \"map_origin\": { \"longitude\": ") +  
    std::to_string(tmp_pos_x) + std::string(",\"latitude\": ") + std::to_string(tmp_pos_y) + std::string(", \"altitude\": 0 }, \"orientation\": [ 0, 0, 0 ] } }, \"primitives\": { \"/object/shape\": { \"polygons\": [ { \"vertices\": [ [ 0, 0, 0 ], [ -0.00001, 0.00001, 0 ], [ -0.00002, 0.00001, 0 ] ], \"base\": { \"object_id\": \"object-1\" } } ] } } } ] } }");
  nlohmann::json json = nlohmann::json::parse(json_str);
  tmp_pos_x += 0.00001/25.0;
  */
  nlohmann::json json;
  json["type"] = "xviz/state_update";
  json["data"]["update_type"] = "snapshot";
  json["data"]["updates"][0]["timestamp"] = now_time_str;
  json["data"]["updates"][0]["poses"]["/vehicle_pose"]["timestamp"] = now_time_str;
  json["data"]["updates"][0]["poses"]["/vehicle_pose"]["map_origin"]["longitude"] = 0;
  json["data"]["updates"][0]["poses"]["/vehicle_pose"]["map_origin"]["latitude"] = 0;
  json["data"]["updates"][0]["poses"]["/vehicle_pose"]["map_origin"]["altitude"] = 0;
  json["data"]["updates"][0]["poses"]["/vehicle_pose"]["orientation"][0] = 0;
  json["data"]["updates"][0]["poses"]["/vehicle_pose"]["orientation"][1] = 0;
  json["data"]["updates"][0]["poses"]["/vehicle_pose"]["orientation"][2] = 0;
  package_mutex_->lock();
  auto actor_list = package_ptr_->GetActorListPtr();
  int i = 0;
  double of = 2.0;
  std::vector<std::pair<double, double>> offset = {{-of, 0.0}, {0.0, of}, {of, 0.0}, {0.0, -of}};
  for (const auto& actor : *actor_list) {
    if (actor->GetTypeId().substr(0, 2) != "ve") {
      continue;
    }
    double x = actor->GetLocation().x;
    double y = actor->GetLocation().y;
    for (int j = 0; j < offset.size(); j++) {
      json["data"]["updates"][0]["primitives"]["/object/shape"]["polygons"][i]["vertices"][j][0] = x + offset[j].first;
      json["data"]["updates"][0]["primitives"]["/object/shape"]["polygons"][i]["vertices"][j][1] = y + offset[j].second;
      json["data"]["updates"][0]["primitives"]["/object/shape"]["polygons"][i]["vertices"][j][2] = 0;
    }
    json["data"]["updates"][0]["primitives"]["/object/shape"]["polygons"][i]["base"]["object_id"] = actor->GetTypeId() + std::to_string(actor->GetId());
    i++;
  }
  package_mutex_->unlock();
  //json["data"]["updates"][0]["primitives"]["/object/shape"]["polygons"][0]
  return json.dump();
}

} // namespace rothberg
