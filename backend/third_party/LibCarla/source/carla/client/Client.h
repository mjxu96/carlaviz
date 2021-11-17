// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/client/detail/Simulator.h"
#include "carla/client/World.h"
#include "carla/PythonUtil.h"
#include "carla/trafficmanager/TrafficManager.h"

namespace carla {
namespace client {

  using namespace carla::traffic_manager;

  class Client {
  public:

    /// Construct a carla client.
    ///
    /// @param host IP address of the host machine running the simulator.
    /// @param port TCP port to connect with the simulator.
    /// @param worker_threads number of asynchronous threads to use, or 0 to use
    ///        all available hardware concurrency.
    explicit Client(
        const std::string &host,
        uint16_t port,
        size_t worker_threads = 0u);

    /// Set a timeout for networking operations. If set, any networking
    /// operation taking longer than @a timeout throws rpc::timeout.
    void SetTimeout(time_duration timeout) {
      _simulator->SetNetworkingTimeout(timeout);
    }

    time_duration GetTimeout() {
      return _simulator->GetNetworkingTimeout();
    }

    /// Return the version string of this client API.
    std::string GetClientVersion() const {
      return _simulator->GetClientVersion();
    }

    /// Return the version string of the simulator we are connected to.
    std::string GetServerVersion() const {
      return _simulator->GetServerVersion();
    }

    std::vector<std::string> GetAvailableMaps() const {
      return _simulator->GetAvailableMaps();
    }

    bool SetFilesBaseFolder(const std::string &path) {
      return _simulator->SetFilesBaseFolder(path);
    }

    std::vector<std::string> GetRequiredFiles(const std::string &folder = "", const bool download = true) const {
      return _simulator->GetRequiredFiles(folder, download);
    }

    void RequestFile(const std::string &name) const {
      _simulator->RequestFile(name);
    }

    World ReloadWorld(bool reset_settings = true) const {
      return World{_simulator->ReloadEpisode(reset_settings)};
    }

    World LoadWorld(
        std::string map_name,
        bool reset_settings = true,
        rpc::MapLayer map_layers = rpc::MapLayer::All) const {
      return World{_simulator->LoadEpisode(std::move(map_name), reset_settings, map_layers)};
    }

    World GenerateOpenDriveWorld(
        std::string opendrive,
        const rpc::OpendriveGenerationParameters & params,
        bool reset_settings = true) const {
      return World{_simulator->LoadOpenDriveEpisode(
          std::move(opendrive), params, reset_settings)};
    }

    /// Return an instance of the world currently active in the simulator.
    World GetWorld() const {
      return World{_simulator->GetCurrentEpisode()};
    }

    /// Return an instance of the TrafficManager currently active in the simulator.
    TrafficManager GetInstanceTM(uint16_t port = TM_DEFAULT_PORT) const {
      return TrafficManager(_simulator->GetCurrentEpisode(), port);
    }

    /// Return an instance of the Episode currently active in the simulator.
    carla::client::detail::EpisodeProxy GetCurrentEpisode() const {
      return _simulator->GetCurrentEpisode();
    }

    std::string StartRecorder(std::string name, bool additional_data = false) {
      return _simulator->StartRecorder(name, additional_data);
    }

    void StopRecorder(void) {
      _simulator->StopRecorder();
    }

    std::string ShowRecorderFileInfo(std::string name, bool show_all) {
      return _simulator->ShowRecorderFileInfo(name, show_all);
    }

    std::string ShowRecorderCollisions(std::string name, char type1, char type2) {
      return _simulator->ShowRecorderCollisions(name, type1, type2);
    }

    std::string ShowRecorderActorsBlocked(std::string name, double min_time, double min_distance) {
      return _simulator->ShowRecorderActorsBlocked(name, min_time, min_distance);
    }

    std::string ReplayFile(std::string name, double start, double duration,
        uint32_t follow_id, bool replay_sensors) {
      return _simulator->ReplayFile(name, start, duration, follow_id, replay_sensors);
    }

    void StopReplayer(bool keep_actors) {
      _simulator->StopReplayer(keep_actors);
    }

    void SetReplayerTimeFactor(double time_factor) {
      _simulator->SetReplayerTimeFactor(time_factor);
    }

    void SetReplayerIgnoreHero(bool ignore_hero) {
      _simulator->SetReplayerIgnoreHero(ignore_hero);
    }

    void ApplyBatch(
        std::vector<rpc::Command> commands,
        bool do_tick_cue = false) const {
      _simulator->ApplyBatch(std::move(commands), do_tick_cue);
    }

    std::vector<rpc::CommandResponse> ApplyBatchSync(
        std::vector<rpc::Command> commands,
        bool do_tick_cue = false) const {
      auto responses = _simulator->ApplyBatchSync(std::move(commands), false);
      if (do_tick_cue)
        _simulator->Tick(_simulator->GetNetworkingTimeout());

      return responses;
    }

  private:

    std::shared_ptr<detail::Simulator> _simulator;
  };

  inline Client::Client(
      const std::string &host,
      uint16_t port,
      size_t worker_threads)
    : _simulator(
        new detail::Simulator(host, port, worker_threads),
        PythonUtil::ReleaseGILDeleter()) {}

} // namespace client
} // namespace carla
