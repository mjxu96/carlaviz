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

#include "definitions.h"

#include <spdlog/spdlog.h>

#include <stdexcept>
#include <string>

namespace carlaviz::logging {

enum class LogLevel : int {
  DEBUG = spdlog::level::debug,
  INFO = spdlog::level::info,
  WARN = spdlog::level::warn,
  ERROR = spdlog::level::err,
  FATAL = spdlog::level::critical
};

LogLevel LogLevelStringToLogLevel(const std::string &);

struct LoggingOption {
  std::string filename = "";
  LogLevel level = LogLevel::INFO;
};

void SetUpGlobalLogging(const LoggingOption &option);

template <typename... Args>
void LogDebug(spdlog::format_string_t<Args...> fmt, Args &&...args) {
  spdlog::debug(fmt, std::forward<Args>(args)...);
  spdlog::default_logger()->flush();
}

template <typename... Args>
void LogInfo(spdlog::format_string_t<Args...> fmt, Args &&...args) {
  spdlog::info(fmt, std::forward<Args>(args)...);
  spdlog::default_logger()->flush();
}

template <typename... Args>
void LogWarn(spdlog::format_string_t<Args...> fmt, Args &&...args) {
  spdlog::warn(fmt, std::forward<Args>(args)...);
  spdlog::default_logger()->flush();
}

template <typename... Args>
void LogError(spdlog::format_string_t<Args...> fmt, Args &&...args) {
  spdlog::error(fmt, std::forward<Args>(args)...);
  spdlog::default_logger()->flush();
}

template <typename... Args>
void LogFatal(spdlog::format_string_t<Args...> fmt, Args &&...args) {
  spdlog::critical(fmt, std::forward<Args>(args)...);
  spdlog::default_logger()->flush();
  // TODO update fmt::format to std::format
  throw std::runtime_error(fmt::format(fmt, std::forward<Args>(args)...));
}

}  // namespace carlaviz::logging
