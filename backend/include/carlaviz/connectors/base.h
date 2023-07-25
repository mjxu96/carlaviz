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
#include <carlaviz/utils/spinlock.h>

#include <functional>
#include <string>

namespace carlaviz::connectors {

using FrontendParameterUpdateCallbackType =
    std::function<void(const std::string&)>;

// Connectors connects frontend and it should run
// in a dedicated thread.
template <typename DerivedServer>
class ConnectorBase {
 public:
  ConnectorBase(utils::ConnectorOption option) : option_(option) {}

  void RegisterFrontendParameterUpdateCallback(
      const FrontendParameterUpdateCallbackType& callback) {
    frontend_param_update_callback_ = callback;
  }

  void UpdateMetadata(std::string&& metadata) {
    return static_cast<DerivedServer*>(this)->UpdateMetadata(
        std::move(metadata));
  }

  void Reset() {}

  void Start() { return static_cast<DerivedServer*>(this)->Start(); }

  void Join() { return static_cast<DerivedServer*>(this)->Join(); }

 protected:
  FrontendParameterUpdateCallbackType frontend_param_update_callback_;
  utils::ConnectorOption option_;
  utils::Spinlock lock_;
};

}  // namespace carlaviz::connectors
