/*
 * File: frontend_proxy.cpp
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Monday, 7th October 2019 3:22:45 pm
 */

#include "platform/proxy/frontend_proxy.h"

using namespace mellocolate;
using tcp = boost::asio::ip::tcp;
namespace websocket = boost::beast::websocket;

FrontendClient::FrontendClient(
    boost::shared_ptr<
        boost::beast::websocket::stream<boost::asio::ip::tcp::socket>>
        frontend_client_ptr)
    : frontend_client_ptr_(std::move(frontend_client_ptr)) {}

void FrontendClient::Write(const std::string& data) {
  boost::beast::multi_buffer buffer;
  boost::beast::ostream(buffer) << data;
  frontend_client_ptr_->write(buffer.data());
}

void FrontendClient::Write(const boost::beast::multi_buffer& data) {
  frontend_client_ptr_->write(data.data());
}

void FrontendClient::ChangeMetadataSendStatus(bool new_status) {
  is_metadata_sent_ = new_status;
}
bool FrontendClient::IsMetadataSend() const { return is_metadata_sent_; }

FrontendProxy::FrontendProxy(uint16_t frontend_listen_port)
    : frontend_listen_port_(frontend_listen_port) {}

void FrontendProxy::UpdateMetadata(std::string updated_metadata) {
  update_metadata_lock_.lock();
  updated_metadata_ = std::move(updated_metadata);
  update_metadata_lock_.unlock();

  std::lock_guard<std::mutex> lock_guard(add_client_lock_);
  for (const auto& client_pair : frontend_clients_) {
    auto client_ptr = client_pair.second;
    client_ptr->ChangeMetadataSendStatus(false);
  }
}

void FrontendProxy::StartListen() {
  auto t = std::thread(&FrontendProxy::Accept, this);
  t.detach();
}

void FrontendProxy::SendToAllClients(const std::string& message) {
  boost::beast::multi_buffer buffer;
  boost::beast::ostream(buffer) << message;

  std::unordered_set<uint32_t> to_delete_ids;

  std::lock_guard<std::mutex> lock_guard(add_client_lock_);
  for (const auto& client_pair : frontend_clients_) {
    auto client_ptr = client_pair.second;
    try {
      if (!client_ptr->IsMetadataSend()) {
        SendMetadata(client_ptr);
      }
      client_ptr->Write(buffer);
    } catch (boost::system::system_error const& se) {
      to_delete_ids.insert(client_pair.first);
      if (se.code() != websocket::error::closed &&
          std::strcmp(se.what(), "Broken pipe") != 0 &&
          std::strcmp(se.what(), "Connection reset by peer")) {
        LOG_ERROR("ERROR WHEN SENDDING UPDATE %s", se.what());
      } else {
        LOG_INFO("Frontend connection closed");
      }
    }
  }
  for (const auto& to_delete_id : to_delete_ids) {
    frontend_clients_.erase(to_delete_id);
  }
}

void FrontendProxy::Accept() {
  LOG_INFO("Waiting for a frontend to connect. Listening to port %u....",
           frontend_listen_port_);
  try {
    boost::asio::io_context ioc{1};
    // TODO change ip here
    tcp::acceptor acceptor{ioc,
                           tcp::endpoint(tcp::v4(), frontend_listen_port_)};
    for (;;) {
      tcp::socket socket{ioc};
      acceptor.accept(socket);
      AddClient(std::move(socket));
    }
  } catch (const std::exception& e) {
    LOG_ERROR("%s", e.what());
  }
}

void FrontendProxy::AddClient(boost::asio::ip::tcp::socket socket) {
  auto client_ptr =
      boost::make_shared<websocket::stream<tcp::socket>>(std::move(socket));
  try {
    client_ptr->accept();
    LOG_INFO("Frontend connected");

    auto frontend_client = boost::make_shared<FrontendClient>(client_ptr);

    SendMetadata(frontend_client);

    std::lock_guard<std::mutex> lock_guard(add_client_lock_);
    frontend_clients_.insert({frontend_max_id_, frontend_client});
    frontend_max_id_++;
  } catch (boost::system::system_error const& se) {
    if (se.code() != websocket::error::closed) {
      throw se;
    } else {
      LOG_INFO("Frontend connection closed");
    }
  }
}

void FrontendProxy::SendMetadata(
    const boost::shared_ptr<FrontendClient>& client) {
  boost::beast::multi_buffer buffer;
  update_metadata_lock_.lock();
  if (updated_metadata_.size() == 0) {
    update_metadata_lock_.unlock();
    return;
  }
  boost::beast::ostream(buffer) << updated_metadata_;
  update_metadata_lock_.unlock();
  client->Write(buffer);
  client->ChangeMetadataSendStatus(true);
}