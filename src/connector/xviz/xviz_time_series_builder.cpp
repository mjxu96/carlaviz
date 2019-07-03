
#include "connector/xviz/xviz_time_series_builder.h"

using namespace mellocolate;
using Json = nlohmann::json;

XVIZTimeSeriesBuider::XVIZTimeSeriesBuider(std::string name) :
  name_(std::move(name)) {}

XVIZTimeSeriesBuider& XVIZTimeSeriesBuider::AddTimestamp(double timestamp) {
  timestamp_ = timestamp;
  return *this;
}

XVIZTimeSeriesBuider& XVIZTimeSeriesBuider::AddValue(double value) {
  value_number_ = value;
  return *this;
}

XVIZTimeSeriesBuider& XVIZTimeSeriesBuider::AddValue(std::string value) {
  value_string_ = std::move(value);
  return *this;
}

XVIZTimeSeriesBuider& XVIZTimeSeriesBuider::AddValue(bool value) {
  value_bool_ = value;
  return *this;
}
/*
std::string XVIZTimeSeriesBuider::GetName() const {
  return name_;
}
*/

Json XVIZTimeSeriesBuider::GetData() const {
  Json json;
  json["streams"][0] = name_;
  if (timestamp_ != boost::none) {
    json["timestamp"] = timestamp_.value();
  }
  if (value_bool_ != boost::none) {
    json["values"]["bools"] = (value_bool_.value() ? "true" : "false");
  }
  if (value_number_ != boost::none) {
    json["values"]["doubles"] = value_number_.value();
  }
  if (value_string_ != boost::none) {
    json["values"]["strings"] = value_string_.value();
  }

  return json;
}