#ifndef WEBSOCKET_SERVER_H_
#define WEBSOCKET_SERVER_H_

#include "connector/utils/package.h"
#include "connector/utils/json.hpp"

#include <boost/shared_ptr.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/ip/tcp.hpp>

#include <cstdlib>
#include <functional>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <thread>
#include <vector>
#include <mutex>
#include <chrono>

namespace rothberg {


class WebsocketServer {
public:
  WebsocketServer() = default;
  void Init(boost::shared_ptr<rothberg::utils::Package> package_ptr, 
    boost::shared_ptr<std::mutex> package_mutex, 
    std::string host="127.0.0.1", uint16_t port=8081u);
  void Run();

private:
  void DoSession(boost::asio::basic_stream_socket<boost::asio::ip::tcp>& socket);

  //nlohmann::json GetInitMetaDataJson();
  //nlohmann::json GetLiveDataJson();
  std::string GetInitMetaDataJson();
  std::string GetLiveDataJson();

  std::string host_{"127.0.0.1"};
  uint16_t port_{8081u};
  boost::shared_ptr<rothberg::utils::Package> package_ptr_{nullptr};
  boost::shared_ptr<std::mutex> package_mutex_{nullptr};
  std::vector<std::thread> threads_{};

  // TODO remove tmp
  double tmp_pos_x{0};
  double tmp_pos_y{0};
};

} // namespace rothberg
#endif