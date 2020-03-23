/*
 * File: frontend_proxy.cpp
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Monday, 7th October 2019 3:22:45 pm
 */

#include "backend/proxy/frontend_proxy.h"

using namespace carlaviz;
using tcp = boost::asio::ip::tcp;
namespace websocket = boost::beast::websocket;
using Json = nlohmann::json;

FrontendClient::FrontendClient(
    boost::shared_ptr<
        boost::beast::websocket::stream<boost::asio::ip::tcp::socket>>
        frontend_client_ptr,
        const std::function<void(const std::unordered_map<std::string, bool>&)>& stream_settings_callback)
    : frontend_client_ptr_(std::move(frontend_client_ptr)),
      stream_settings_callback_(stream_settings_callback) {
  std::thread t(&FrontendClient::StartRead, this);
  t.detach();
}

void FrontendClient::StartRead() {
  try {
    while (true) {
      boost::beast::multi_buffer buffer;
      frontend_client_ptr_->read(buffer);
      auto read_data = boost::beast::buffers_to_string(buffer.data());
      try {
        Json setting_json = Json::parse(read_data);
        stream_settings_callback_(setting_json.get<std::unordered_map<std::string, bool>>());
      } catch (const std::exception& e) {
        CARLAVIZ_LOG_WARNING("When paring %s, get error: %s", read_data.c_str(),
          e.what());
      }
    }
  } catch (const std::exception& e) {
    if (std::strcmp(e.what(), "Operation canceled") != 0 &&
        std::strcmp(e.what(), "Broken pipe") != 0 &&
        std::strcmp(e.what(), "Connection reset by peer") != 0) {
        CARLAVIZ_LOG_ERROR("ERROR WHEN READING SETTING UPDATE %s", e.what());
    }
  }
}

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
void FrontendClient::ChangeMapStringSendStatus(bool new_status) {
  is_map_string_sent_ = new_status;
}
bool FrontendClient::IsMetadataSend() const { return is_metadata_sent_; }
bool FrontendClient::IsMapSend() const {
  return is_map_string_sent_;
}

bool FrontendClient::SetBinary(bool is_binary) {
  frontend_client_ptr_->binary(true);
  return is_binary;
}

FrontendProxy::FrontendProxy(uint16_t frontend_listen_port)
    : frontend_listen_port_(frontend_listen_port) {}

void FrontendProxy::UpdateMetadata(const std::string& updated_metadata) {
  update_metadata_lock_.lock();
  updated_metadata_without_map_ = updated_metadata;
  updated_metadata_with_map_ = updated_metadata;
  updated_metadata_with_map_.pop_back();
  updated_metadata_with_map_ += ",\"map\": " + map_string_;
  updated_metadata_with_map_ += "}";
  update_metadata_lock_.unlock();

  std::lock_guard<std::mutex> lock_guard(add_client_lock_);
  for (const auto& client_pair : frontend_clients_) {
    auto client_ptr = client_pair.second;
    client_ptr->ChangeMetadataSendStatus(false);
  }
}

void FrontendProxy::SetMapString(const std::string& map_string) {
  map_string_ = map_string;
}

void FrontendProxy::StartListen() {
  auto t = std::thread(&FrontendProxy::Accept, this);
  t.detach();
}

void FrontendProxy::SetStreamSettingsCallback(
    const std::function<void(const std::unordered_map<std::string, bool>&)>& stream_settings_callback) {
  stream_settings_callback_ = stream_settings_callback;
}

void FrontendProxy::SendToAllClients(std::string&& message) {
  boost::beast::multi_buffer buffer;
  boost::beast::ostream(buffer) << std::move(message);

  std::unordered_set<uint32_t> to_delete_ids;
  // CARLAVIZ_LOG_INFO("string size %ld, sent size %ld", message.size(), buffer.size());

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
          std::strcmp(se.what(), "Operation canceled") != 0 &&
          std::strcmp(se.what(), "Connection reset by peer")) {
        CARLAVIZ_LOG_ERROR("ERROR WHEN SENDDING UPDATE %s", se.what());
      } else {
        CARLAVIZ_LOG_INFO("Frontend connection closed");
      }
    }
  }
  for (const auto& to_delete_id : to_delete_ids) {
    frontend_clients_.erase(to_delete_id);
  }
}

void FrontendProxy::Accept() {
  CARLAVIZ_LOG_INFO("Waiting for a frontend to connect. Listening to port %u....",
           frontend_listen_port_);
  try {
    boost::asio::io_context ioc{1};
    tcp::acceptor acceptor{ioc,
                           tcp::endpoint(tcp::v4(), frontend_listen_port_)};
    for (;;) {
      tcp::socket socket{ioc};
      acceptor.accept(socket);
      AddClient(std::move(socket));
    }
  } catch (const std::exception& e) {
    CARLAVIZ_LOG_ERROR("%s", e.what());
  }
}

void FrontendProxy::AddClient(boost::asio::ip::tcp::socket socket) {
  auto client_ptr =
      boost::make_shared<websocket::stream<tcp::socket>>(std::move(socket));
  try {
    client_ptr->accept();
    CARLAVIZ_LOG_INFO("Frontend connected");

    auto frontend_client = boost::make_shared<FrontendClient>(client_ptr, 
      stream_settings_callback_);

    frontend_client->SetBinary(true);
    SendMetadata(frontend_client);

    std::lock_guard<std::mutex> lock_guard(add_client_lock_);
    frontend_clients_.insert({frontend_max_id_, frontend_client});
    frontend_max_id_++;
  } catch (boost::system::system_error const& se) {
    if (se.code() != websocket::error::closed) {
      throw se;
    } else {
      CARLAVIZ_LOG_INFO("Frontend connection closed");
    }
  }
}

void FrontendProxy::SendMetadata(
    const boost::shared_ptr<FrontendClient>& client) {
  boost::beast::multi_buffer buffer;
  update_metadata_lock_.lock();
  if (updated_metadata_with_map_.size() == 0 ||
      updated_metadata_without_map_.size() == 0) {
    update_metadata_lock_.unlock();
    return;
  }
  if (client->IsMapSend()) {
    boost::beast::ostream(buffer) << updated_metadata_without_map_;
  } else {
    boost::beast::ostream(buffer) << updated_metadata_with_map_;
  }
  update_metadata_lock_.unlock();
  client->Write(buffer);
  client->ChangeMetadataSendStatus(true);
  client->ChangeMapStringSendStatus(true);
}