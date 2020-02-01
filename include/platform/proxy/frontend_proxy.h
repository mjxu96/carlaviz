/*
 * File: frontend_proxy.h
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Monday, 7th October 2019 3:22:53 pm
 */

#ifndef MELLOCOLATE_FRONTEND_PROXY_H_
#define MELLOCOLATE_FRONTEND_PROXY_H_

#include "platform/utils/macrologger.h"

#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/unordered_map.hpp>

#include <mutex>
#include <thread>
#include <unordered_set>

namespace mellocolate {

class FrontendClient {
 public:
  FrontendClient(boost::shared_ptr<
                 boost::beast::websocket::stream<boost::asio::ip::tcp::socket>>
                     frontend_client_ptr);
  void Write(const std::string& data);
  void Write(const boost::beast::multi_buffer& data);
  void ChangeMetadataSendStatus(bool new_status);
  bool IsMetadataSend() const;
  bool SetBinary(bool is_binary);

 private:
  bool is_metadata_sent_ = false;
  boost::shared_ptr<
      boost::beast::websocket::stream<boost::asio::ip::tcp::socket>>
      frontend_client_ptr_ = nullptr;
};

class FrontendProxy {
 public:
  FrontendProxy() = default;
  FrontendProxy(uint16_t frontend_listen_port);
  void UpdateMetadata(std::string updated_metadata);
  void StartListen();
  void SendToAllClients(const std::string& message);

 private:
  void Accept();
  void AddClient(boost::asio::ip::tcp::socket socket);
  void SendMetadata(const boost::shared_ptr<FrontendClient>& client);

  uint16_t frontend_listen_port_ = 8081u;
  std::mutex update_metadata_lock_{};
  std::string updated_metadata_{""};

  std::mutex add_client_lock_{};
  uint32_t frontend_max_id_ = 0u;
  boost::unordered_map<uint32_t, boost::shared_ptr<FrontendClient>>
      frontend_clients_{};
};

}  // namespace mellocolate

#endif