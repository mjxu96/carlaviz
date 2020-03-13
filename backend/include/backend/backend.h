/*
 * File: backend.h
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Monday, 7th October 2019 3:19:54 pm
 */

#ifndef CARLAVIZ_BACKEND_H_
#define CARLAVIZ_BACKEND_H_

#include "backend/proxy/carla_proxy.h"
#include "backend/proxy/drawing_proxy.h"
#include "server/carla_handler.h"
#include "server/carla_session.h"
#include "server/xviz_server.h"
#include "backend/proxy/frontend_proxy.h"


#include "io/glb_writer.h"

namespace carlaviz {

class Backend {
 public:
  Backend() = default;
  void Init();
  void Run();
  void Clear();
  void SetIsExperimental(bool is_experimental);
  void SetTimeInterval(uint32_t time_interval);

 private:
  std::shared_ptr<DrawingProxy> drawing_proxy_{nullptr};
  std::shared_ptr<xviz::XVIZServer> server_{nullptr};
  std::shared_ptr<FrontendProxy> frontend_proxy_{nullptr};
  std::shared_ptr<CarlaProxy> carla_proxy_{nullptr};
  bool is_experimental_{false};
  uint32_t time_interval_{100u};
};

}  // namespace carlaviz

#endif