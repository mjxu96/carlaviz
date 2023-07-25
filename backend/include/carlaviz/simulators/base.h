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

#include <carlaviz/utils/definitions.h>
#include <carlaviz/utils/logging.h>
#include <carlaviz/utils/map.h>
#include <carlaviz/utils/options.h>
#include <carlaviz/utils/sensor.h>
#include <carlaviz/utils/spinlock.h>

#include <cstdint>
#include <functional>
#include <iostream>
#include <mutex>
#include <thread>
#include <unordered_set>

namespace carlaviz::simulators {

enum class ConnectionState { DISCONNECTED, CONNECTING, CONNECTED };

template <typename DerivedSimulator, typename TranslationType>
class SimulatorBase {
 public:
  SimulatorBase(const utils::SimulatorOption& option,
                TranslationType& translation)
      : option_(option), translation_(translation) {
    Init();
  }

  void Init() {
    translation_.RegisterSensorSubscriptionStateUpdateCallback(
        std::bind(&SimulatorBase::OnSensorSubscriptionStateUpdate, this,
                  std::placeholders::_1, std::placeholders::_2));
    static_cast<DerivedSimulator*>(this)->InitImpl();
  }

  void Connect() {
    try {
      static_cast<DerivedSimulator*>(this)->ConnectImpl();
      // process once the simulator is connected
      translation_.Process();
      connection_state_ = ConnectionState::CONNECTED;
    } catch (const std::exception& e) {
      logging::LogError("Connection to simulator failed for {}", e.what());
    }
  }

  void Disconnect() {
    try {
      static_cast<DerivedSimulator*>(this)->DisconnectImpl();
      logging::LogInfo("Disconnected from simulator");
    } catch (const std::exception& e) {
      // ignore disconnection error
      logging::LogError("Disconnecting from simulator fails for {}", e.what());
    }
    ClearWhenDisconnected();
    connection_state_ = ConnectionState::DISCONNECTED;
    translation_.Reset();
  }

  void LoopOnce() {
    try {
      static_cast<DerivedSimulator*>(this)->ProcessImpl();
      translation_.Process();
      std::this_thread::sleep_for(std::chrono::milliseconds(
          option_.sleep_between_updates_milliseconds));
    } catch (const std::exception& e) {
      logging::LogError("Connection to simulator failed for {}", e.what());
      Disconnect();
    }
  }

  void Process() {
    switch (connection_state_) {
      case ConnectionState::DISCONNECTED:
        if (retried_times_) {
          std::this_thread::sleep_for(
              std::chrono::seconds(option_.retry_interval_seconds));
        }
        if (retried_times_ <= option_.retry_times_after_disconnection) {
          // do connect
          Connect();
          retried_times_++;
        } else {
          logging::LogFatal(
              "Max retry times {} passed when connecting to simulator, "
              "aborting...",
              option_.retry_times_after_disconnection);
        }
        break;
      case ConnectionState::CONNECTING:
        // do nothing for now
        break;
      case ConnectionState::CONNECTED:
        LoopOnce();
        break;
      default:
        break;
    }
    ClearInEachLoop();
  }

  void OnSensorSubscriptionStateUpdate(
      uint64_t sensor_id, utils::SensorSubscriptionState new_state) {}

  void StartLoop() {
    while (!stopped_) {
      Process();
    }
  }

 protected:
  // types
  using FrameType = std::uint64_t;
  using SensorIteratorType = std::unordered_map<uint64_t, bool>::iterator;
  using SensorIDType = sensor::SensorIDType;
  using LockType = utils::Spinlock;

  utils::SimulatorOption option_;
  TranslationType& translation_;

  bool stopped_ = false;
  ConnectionState connection_state_{ConnectionState::DISCONNECTED};
  uint32_t retried_times_ = 0u;

  map::Map map_;

  std::atomic<FrameType> current_frame_ = 0;

  // sensor lock
  LockType sensor_data_lock_;
  // variables for sensors
  std::unordered_map<SensorIDType, bool> sensor_status_;
  std::unordered_set<SensorIDType> sensors_in_this_frame_;
  // varialbes for sensor data
  std::unordered_map<SensorIDType, sensor::SensorData> sensor_data_;

  // commonly used functions
  // general
  void ClearInEachLoop() {
    {
      std::lock_guard<LockType> guard(sensor_data_lock_);
      sensors_in_this_frame_.clear();
    }
  }

  void ClearWhenDisconnected() {
    current_frame_ = 0;
    {
      std::lock_guard<LockType> guard(sensor_data_lock_);
      sensor_status_.clear();
      sensors_in_this_frame_.clear();
      sensor_data_.clear();
    }
  }

  // sensor related
  bool AddSensor(SensorIDType sensor_id, std::string_view description,
                 bool force = false) {
    std::lock_guard<LockType> guard(sensor_data_lock_);
    sensors_in_this_frame_.insert(sensor_id);
    auto sensor_itr = sensor_status_.find(sensor_id);
    if (sensor_itr != sensor_status_.end() && !force) {
      // we already subscribed to this sensor
      return false;
    }
    logging::LogInfo("Add sensor, id={}, description={} {}", sensor_id,
                     description, (force ? "forcefully" : ""));
    sensor_status_[sensor_id] = true;
    return true;
  }

  void ClearRemovedSensors() {
    std::lock_guard<LockType> guard(sensor_data_lock_);
    // check if any sensors are removed
    // if so, remove them from subscribers
    for (auto sensor_itr_in_last_frame = sensor_status_.begin();
         sensor_itr_in_last_frame != sensor_status_.end();) {
      if (sensors_in_this_frame_.find(sensor_itr_in_last_frame->first) ==
          sensors_in_this_frame_.end()) {
        logging::LogInfo(
            "Remove sensor, id={}, status before removed={}",
            sensor_itr_in_last_frame->first,
            (sensor_itr_in_last_frame->second ? "enabled" : "disabled"));
        // call to derived class to remove the sensor
        static_cast<DerivedSimulator*>(this)->RemoveSensorImpl(
            sensor_itr_in_last_frame->first);
        sensor_data_.erase(sensor_itr_in_last_frame->first);
        this->translation_.RemoveSensor(sensor_itr_in_last_frame->first);
        sensor_itr_in_last_frame =
            sensor_status_.erase(sensor_itr_in_last_frame);
      } else {
        sensor_itr_in_last_frame++;
      }
    }
  }

  // handler to update the sensor data to translation layer
  void ProcessSensorData() {
    std::lock_guard<LockType> guard(this->sensor_data_lock_);
    for (const auto& [sensor_id, sensor_data] : sensor_data_) {
      auto sensor_status_itr = sensor_status_.find(sensor_id);
      if (sensor_status_itr == sensor_status_.end() ||
          !sensor_status_itr->second) {
        continue;
      }
      std::visit(
          [sid = sensor_id, this](const auto& data) {
            logging::LogDebug(
                "Send sensor data to translation layer, id={} type={}", sid,
                sensor::SensorTypeToString(data.info.type));
            using DataType = std::decay_t<decltype(data)>;
            if constexpr (std::is_same_v<DataType, sensor::Image>) {
              this->translation_.UpdateImage(data);
            } else if constexpr (std::is_same_v<DataType,
                                                sensor::LidarMeasurement>) {
              this->translation_.UpdateLidarMeasurement(data);
            } else {
              logging::LogError("unknown sensor data type {}, id={}",
                                sensor::SensorTypeToString(data.info.type),
                                sid);
            }
          },
          sensor_data);
    }
    return;
  }
};

}  // namespace carlaviz::simulators
