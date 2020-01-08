/*
 * File: ui_builder.cpp
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Wednesday, 8th January 2020 1:37:33 am
 */

#include "builder/declarative_ui/ui_builder.h"

using namespace xviz;

XVIZUIBuilder::XVIZUIBuilder() {
}

XVIZUIBuilder& XVIZUIBuilder::Child(const std::shared_ptr<XVIZBaseUIBuilder>& child) {
  children_.push_back(child);
  return *this;
}

std::vector<UIPanel> XVIZUIBuilder::GetUI() {
  std::vector<UIPanel> uis;
  for (auto& child : children_) {
    uis.push_back(child->GetUI());
  }
  return uis;
}