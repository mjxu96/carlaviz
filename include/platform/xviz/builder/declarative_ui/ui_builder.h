/*
 * File: ui_builder.h
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Wednesday, 8th January 2020 1:15:13 am
 */

#ifndef XVIZ_DECLARATIVE_UI_BUILDER_H_
#define XVIZ_DECLARATIVE_UI_BUILDER_H_

#include "proto/declarativeui.pb.h"
#include "builder/declarative_ui/base_ui_builder.h"

#include <vector>

namespace xviz {
  


class XVIZUIBuilder {
public:

  XVIZUIBuilder();

  XVIZUIBuilder& Child(const std::shared_ptr<XVIZBaseUIBuilder>& child);
  std::vector<xviz::UIPanel> GetUI();

private:
  std::vector<std::shared_ptr<XVIZBaseUIBuilder>> children_{};
};

} // namespace xviz
#endif