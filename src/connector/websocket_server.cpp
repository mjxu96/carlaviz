#include "connector/websocket_server.h"

namespace rothberg {


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
      std::cerr << "Please indicate ip address" << std::endl;
      return;
    }

    std::cout << host_ << std::endl;
    const auto address =  boost::asio::ip::make_address(host_);
    const auto port = port_;

    boost::asio::io_context ioc{1};
    boost::asio::ip::tcp::acceptor accetor{ioc, {address, port}};

    std::cout << "Listening to " << address << ":" << port << std::endl;

    while (true) {
      boost::asio::ip::tcp::socket socket{ioc};
      accetor.accept(socket);
      std::cout << "Client connected" << std::endl;
      std::thread t = std::thread{std::bind(
        &WebsocketServer::DoSession, this,
        std::move(socket))};
      t.detach();
      threads_.push_back(std::move(t));
    }

  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return;
  }
}

void WebsocketServer::DoSession(boost::asio::basic_stream_socket<boost::asio::ip::tcp>& socket) {

  try {
    boost::beast::websocket::stream<boost::asio::ip::tcp::socket> ws{std::move(socket)};

    ws.accept();
    for(;;) {
      boost::beast::multi_buffer buffer;

      ws.read(buffer);
      std::cout << boost::beast::buffers(buffer.data()) << std::endl;

      ws.text(ws.got_text());
      ws.write(buffer.data());
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

nlohmann::json WebsocketServer::GetInitMetaDataJson() {
  std::string json_str = "{ \"type\": \"xviz/metadata\",\"data\": { \"version\": \"2.0.0\", \"streams\": { \"/vehicle_pose\": { \"category\": \"pose\" }, \"/object/tracking_point\": { \"category\": \"primitive\", \"coordinate\": \"VEHICLE_RELATIVE\", \"stream_style\": { \"fill_color\": \"#fb0\" }, \"primitive_type\": \"circle\" }, \"/object/shape\": { \"category\": \"primitive\", \"coordinate\": \"VEHICLE_RELATIVE\", \"stream_style\": { \"fill_color\": \"#fb0\", \"height\": 1.5, \"extruded\": true }, \"primitive_type\": \"polygon\" } }, \"log_info\": { \"start_time\": 1000, \"end_time\": 1005 } } }";
  return nlohmann::json::parse(json_str);

}

nlohmann::json WebsocketServer::GetLiveDataJson() {

}

} // namespace rothberg
