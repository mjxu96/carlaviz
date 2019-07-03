
#include "connector/xviz/xviz_builder.h"

using namespace mellocolate;
using Json = nlohmann::json;

XVIZBuilder& XVIZBuilder::AddTimestamp(double timestamp) {
  timestamp_ = timestamp;
  return *this;
}

XVIZBuilder& XVIZBuilder::AddPose(XVIZPoseBuilder pose) {
  poses_.push_back(std::move(pose));
  return *this;
}

XVIZBuilder& XVIZBuilder::AddPrimitive(XVIZPrimitiveBuider primitive) {
  primitives_.push_back(std::move(primitive));
  return *this;
}

XVIZBuilder& XVIZBuilder::AddTimeSeries(XVIZTimeSeriesBuider time_series) {
  time_series_.push_back(std::move(time_series));
  return *this;
}

std::string XVIZBuilder::GetData() const {
  Json json;
  json["type"] = "xviz/state_update";
  json["data"]["update_type"] = "snapshot";
  if (timestamp_ != boost::none) {
    json["data"]["updates"][0]["timestamp"] = timestamp_.value();
  }

  for (const auto& pose : poses_) {
    if (pose.GetData() != nullptr) {
      json["data"]["updates"][0]["poses"][pose.GetName()] = pose.GetData();
    }
  }

  for (const auto& primitive : primitives_) {
    if (primitive.GetData() != nullptr) {
      json["data"]["updates"][0]["primitives"][primitive.GetName()] = primitive.GetData();
    }
  }

  size_t i = 0;
  for (const auto& time_series : time_series_) {
    if (time_series.GetData() != nullptr) {
      json["data"]["updates"][0]["time_series"][i] = time_series.GetData();
      i++;
    }
  }

  return json.dump();
}
