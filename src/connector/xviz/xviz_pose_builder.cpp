
#include "connector/xviz/xviz_pose_builder.h"

using namespace mellocolate;
using Json = nlohmann::json;

XVIZPoseBuilder::XVIZPoseBuilder(std::string name) :
  name_(std::move(name)) {}

XVIZPoseBuilder& XVIZPoseBuilder::AddTimestamp(double timestamp) {
  timestamp_ = timestamp;
  return *this;
}

XVIZPoseBuilder& XVIZPoseBuilder::AddMapOrigin(point_3d_t map_origin) {
  map_origin_ = std::move(map_origin);
  return *this;
}

XVIZPoseBuilder& XVIZPoseBuilder::AddPosition(point_3d_t position) {
  position_ = std::move(position);
  return *this;
}

XVIZPoseBuilder& XVIZPoseBuilder::AddOrientation(point_3d_t orientation) {
  orientation_ = std::move(orientation);
  return *this;
}

std::string XVIZPoseBuilder::GetName() const {
  return name_;
}

Json XVIZPoseBuilder::GetData() const {
  Json json;
  if (timestamp_ != boost::none) {
    json["timestamp"] = timestamp_.value();
  }
  if (map_origin_ != boost::none) {
    json["map_origin"]["longitude"] = map_origin_.value().get<0>();
    json["map_origin"]["latitude"] = map_origin_.value().get<1>();
    json["map_origin"]["altitude"] = map_origin_.value().get<2>();
  }
  if (position_ != boost::none) {
    json["position"][0] = position_.value().get<0>();
    json["position"][1] = position_.value().get<1>();
    json["position"][2] = position_.value().get<2>();
  }
  if (orientation_ != boost::none) {
    json["orientation"][0] = orientation_.value().get<0>();
    json["orientation"][1] = orientation_.value().get<1>();
    json["orientation"][2] = orientation_.value().get<2>();
  }
  return json;
}