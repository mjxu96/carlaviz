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

#include <carlaviz/utils/logging.h>

#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_sinks.h>

#include <iostream>
#include <memory>
#include <stdexcept>

namespace carlaviz::logging {

LogLevel LogLevelStringToLogLevel(const std::string& string) {
  std::string lower_string = string;
  for (char& c : lower_string) {
    if (c >= 'A' && c <= 'Z') {
      c = c - ('A' - 'a');
    }
  }
  return static_cast<LogLevel>(spdlog::level::from_str(lower_string));
}

void SetUpGlobalLogging(const LoggingOption& option) {
  auto console_sink = std::make_shared<spdlog::sinks::stdout_sink_mt>();
  auto logger = std::make_shared<spdlog::logger>(std::string());
  logger->sinks().push_back(console_sink);
  if (option.filename.size() > 0) {
    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(
        option.filename, false);
    logger->sinks().push_back(file_sink);
  }
  spdlog::set_default_logger(logger);
  spdlog::set_level(static_cast<spdlog::level::level_enum>(option.level));
}
}  // namespace carlaviz::logging
