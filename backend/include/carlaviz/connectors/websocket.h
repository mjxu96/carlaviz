/*
 * Project: carlaviz
 * Description: Carla Visulization in Browser
 * Author: Minjun Xu (mjxu96@outlook.com)
 * -----
 * MIT License
 * Copyright (c) 2023 Minjun Xu
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

#pragma once

#include "base.h"

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

#include <atomic>
#include <functional>
#include <memory>
#include <thread>
#include <unordered_set>
#include <vector>

namespace carlaviz::connectors {

class WebsocketServer : public ConnectorBase<WebsocketServer> {
 public:
  using BaseType = ConnectorBase<WebsocketServer>;
  using BaseType::BaseType;

  void UpdateMetadata(std::string&& metadata) {
    logging::LogInfo("Metadata is updated, sending to all connected clients");
    this->lock_.lock();
    metadata_ = std::move(metadata);
    this->lock_.unlock();
    BroadcastToAllClients(metadata_);
  }

  void UpdateSnapshot(const std::string& snapshot) {
    logging::LogDebug("Snapshot is updated, sending to all connected clients");
    BroadcastToAllClients(snapshot);
  }

  void BroadcastToAllClients(const std::string& data) {
    this->lock_.lock();
    for (auto conn_itr = clients_.begin(); conn_itr != clients_.end();) {
      if (!SendData(*conn_itr, data)) {
        conn_itr = clients_.erase(conn_itr);
        logging::LogInfo("Removed connection {} from connected clients list",
                         (*conn_itr)->get_remote_endpoint());
      } else {
        conn_itr++;
      }
    }
    this->lock_.unlock();
  }

  bool SendData(
      std::shared_ptr<websocketpp::connection<websocketpp::config::asio>> conn,
      const std::string& data) {
    std::error_code err = conn->send(data.data(), data.size(),
                                     websocketpp::frame::opcode::binary);
    if (err) {
      logging::LogInfo("Remote connection from {} is disconnected",
                       conn->get_remote_endpoint());
      return false;
    }
    return true;
  }

  void Start() {
    is_stopped_.store(false);
    logging::LogInfo("CarlaViz will host websocket server on {}:{}",
                     this->option_.host, this->option_.port);

    server_.init_asio();
    server_.clear_access_channels(websocketpp::log::alevel::all);
    server_.clear_error_channels(websocketpp::log::elevel::all);
    server_.set_reuse_addr(true);
    server_.set_open_handler([&, this](websocketpp::connection_hdl hdl) {
      auto conn = this->server_.get_con_from_hdl(hdl);
      logging::LogInfo("CarlaViz receives websocket connection from {}",
                       conn->get_remote_endpoint());
      this->lock_.lock();
      if (this->SendData(conn, this->metadata_)) {
        this->clients_.insert(conn);
      }
      this->lock_.unlock();
    });

    server_.set_close_handler([this](websocketpp::connection_hdl hdl) {
      auto conn = this->server_.get_con_from_hdl(hdl);
      this->lock_.lock();
      clients_.erase(conn);
      this->lock_.unlock();
      logging::LogInfo("Remote connection from {} is disconnected",
                       conn->get_remote_endpoint());
    });

    server_.listen(asio::ip::tcp::endpoint{
        asio::ip::address::from_string(this->option_.host),
        this->option_.port});
    this->server_.start_accept();
    main_thread_ = std::make_unique<std::thread>([this] {
      logging::LogInfo("CarlaViz websocket server starts.");
      this->server_.run();
    });
  }

  void Join() {
    is_stopped_.store(true);
    server_.stop();
    main_thread_->join();

    this->lock_.lock();
    clients_.clear();
    this->lock_.unlock();
  }

 private:
  websocketpp::server<websocketpp::config::asio> server_;
  std::atomic<bool> is_stopped_{false};
  std::unique_ptr<std::thread> main_thread_{nullptr};
  std::unordered_set<
      std::shared_ptr<websocketpp::connection<websocketpp::config::asio>>>
      clients_;

  // data
  std::string metadata_;
};

}  // namespace carlaviz::connectors
