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

#include <carlaviz/backend.h>
#include <carlaviz/utils/logging.h>

#include <gflags/gflags.h>

#include <iostream>
#include <limits>

// simulator side configuration
DEFINE_string(simulator_host, "localhost", "The host of simulator");
DEFINE_uint32(simulator_port, 2000u, "The port of simulator");
DEFINE_uint32(simulator_timeout_seconds, 10u, "Simulator connect timeout");
DEFINE_uint32(simulator_retry_times_after_disconnection, 3u,
              "Simulator connect retry times");
DEFINE_uint32(simulator_retry_interval_seconds, 1u,
              "Simulator connect retry interval");
DEFINE_uint32(simulator_sleep_between_updates_milliseconds, 0u,
              "Whether to sleep for some time between two simulator updates");
DEFINE_string(simulator_ego_vehicle_name, "ego", "The ego vehicle name");
DEFINE_uint32(simulator_sensor_max_lag_frame, 30u,
              "Simulator sensor max lag frame");

DEFINE_bool(translation_allow_static_objects, true,
            "Whether to show static objects");

DEFINE_string(connector_host, "0.0.0.0", "The host for server to listen on");
DEFINE_uint32(connector_port, 8081, "The port for server to listen on");
DEFINE_uint32(connector_update_interval_milliseconds, 500,
              "The interval to update");

DEFINE_string(log_filename, "", "Log filename");
DEFINE_string(log_level, "info", "Log level");

using namespace carlaviz;

void LogFlags() {
  std::vector<gflags::CommandLineFlagInfo> flags;
  gflags::GetAllFlags(&flags);
  logging::LogInfo("All parameters:");
  for (const auto& flag : flags) {
    logging::LogInfo("\t{}: {}", flag.name,
                     flag.is_default ? flag.default_value : flag.current_value);
  }
}

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  logging::LoggingOption logging_option{
      .filename = FLAGS_log_filename,
      .level = logging::LogLevelStringToLogLevel(FLAGS_log_level)};

  logging::SetUpGlobalLogging(logging_option);

  LogFlags();

  utils::ConnectorOption connector_option{
      .host = FLAGS_connector_host,
      .port = (uint16_t)FLAGS_connector_port,
      .update_interval_milliseconds =
          FLAGS_connector_update_interval_milliseconds};

  Connector connector(connector_option);

  connector.Start();

  utils::TranslationOption translation_option{
      .allow_static_objects = FLAGS_translation_allow_static_objects};

  Translation translation(translation_option, connector);

  utils::SimulatorOption option{
      .host = FLAGS_simulator_host,
      .port = (uint16_t)FLAGS_simulator_port,
      .timeout_seconds = FLAGS_simulator_timeout_seconds,
      .retry_times_after_disconnection =
          FLAGS_simulator_retry_times_after_disconnection,
      .retry_interval_seconds = FLAGS_simulator_retry_interval_seconds,
      .sleep_between_updates_milliseconds =
          FLAGS_simulator_sleep_between_updates_milliseconds,
      .ego_vehicle_name = FLAGS_simulator_ego_vehicle_name,
      .sensor_max_lag_frame = FLAGS_simulator_sensor_max_lag_frame};

  Simulator simulator(option, translation);

  simulator.StartLoop();
  return 0;
}
