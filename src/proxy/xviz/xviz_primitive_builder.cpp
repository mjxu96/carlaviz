/*
 * File: xviz_primitive_builder.cpp
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Sunday, 7th July 2019 8:52:29 pm
 */

#include "proxy/xviz/xviz_primitive_builder.h"

using namespace mellocolate;
using Json = nlohmann::json;

XVIZPrimitivePolygonBuilder::XVIZPrimitivePolygonBuilder(
    std::vector<point_3d_t> vertices)
    : vertices_(std::move(vertices)) {}

XVIZPrimitivePolygonBuilder& XVIZPrimitivePolygonBuilder::AddId(
    std::string id) {
  id_ = std::move(id);
  return *this;
}

Json XVIZPrimitivePolygonBuilder::GetData() const {
  Json json;
  size_t i = 0u;
  for (const auto& point : vertices_) {
    json["vertices"][i][0] = point.get<0>();
    json["vertices"][i][1] = point.get<1>();
    json["vertices"][i][2] = point.get<2>();
    i++;
  }
  if (id_ != boost::none) {
    json["base"]["object_id"] = id_.value();
  }
  return json;
}

XVIZPrimitiveCircleBuilder::XVIZPrimitiveCircleBuilder(point_3d_t center,
                                                       double radius)
    : center_(std::move(center)), radius_(radius) {}

XVIZPrimitiveCircleBuilder& XVIZPrimitiveCircleBuilder::AddId(std::string id) {
  id_ = std::move(id);
  return *this;
}

Json XVIZPrimitiveCircleBuilder::GetData() const {
  Json json;
  json["center"][0] = center_.get<0>();
  json["center"][1] = center_.get<1>();
  json["center"][2] = center_.get<2>();
  json["radius"] = radius_;
  if (id_ != boost::none) {
    json["base"]["object_id"] = id_.value();
  }
  return json;
}

XVIZPrimitivePointBuilder::XVIZPrimitivePointBuilder(
    std::vector<point_3d_t> points)
    : points_(std::move(points)) {}

XVIZPrimitivePointBuilder& XVIZPrimitivePointBuilder::AddId(std::string id) {
  id_ = std::move(id);
  return *this;
}

Json XVIZPrimitivePointBuilder::GetData() const {
  Json json;
  size_t i = 0u;
  for (const auto& point : points_) {
    json["points"][i][0] = point.get<0>();
    json["points"][i][1] = point.get<1>();
    json["points"][i][2] = point.get<2>();
    i++;
  }
  if (id_ != boost::none) {
    json["base"]["object_id"] = id_.value();
  }
  return json;
}

XVIZPrimitiveImageBuilder::XVIZPrimitiveImageBuilder(std::string encoded_str)
    : encoded_str_(std::move(encoded_str)) {}

Json XVIZPrimitiveImageBuilder::GetData() const {
  //   {
  //     "position": [9, 15, 3],
  //     "data":
  //     "/9j/2wBDAAMCAgICAgMCAgIDAwMDBAYEBAQEBAgGBgUGCQgKCgkICQkKDA8MCgsOCwkJDRENDg8QEBEQCgwSExIQEw8QEBD/yQALCAABAAEBAREA/8wABgAQEAX/2gAIAQEAAD8A0s8g/9k=",
  //     "width_px": 1280,
  //     "height_px": 720
  // }
  Json json;
  json["data"] = encoded_str_;
  // json["position"] = {9, 15, 3};
  json["width_px"] = 352;
  json["height_px"] = 356;
  return json;
}

XVIZPrimitiveBuider::XVIZPrimitiveBuider(std::string name)
    : name_(std::move(name)) {}

XVIZPrimitiveBuider& XVIZPrimitiveBuider::AddPolygon(
    XVIZPrimitivePolygonBuilder polygon) {
  polygons_.push_back(std::move(polygon));
  return *this;
}

XVIZPrimitiveBuider& XVIZPrimitiveBuider::AddCircle(
    XVIZPrimitiveCircleBuilder circle) {
  circles_.push_back(std::move(circle));
  return *this;
}

XVIZPrimitiveBuider& XVIZPrimitiveBuider::AddPoints(
    XVIZPrimitivePointBuilder points) {
  points_.push_back(std::move(points));
  return *this;
}

XVIZPrimitiveBuider& XVIZPrimitiveBuider::AddImages(
    XVIZPrimitiveImageBuilder image) {
  images_.push_back(std::move(image));
  return *this;
}

std::string XVIZPrimitiveBuider::GetName() const { return name_; }

Json XVIZPrimitiveBuider::GetData() const {
  Json json;

  size_t i = 0u;
  for (const auto& circle : circles_) {
    json["circles"][i] = circle.GetData();
    i++;
  }

  i = 0u;
  for (const auto& polygon : polygons_) {
    json["polygons"][i] = polygon.GetData();
    i++;
  }

  i = 0u;
  for (const auto& point : points_) {
    json["points"][i] = point.GetData();
    i++;
  }

  i = 0u;
  for (const auto& image : images_) {
    json["images"][i] = image.GetData();
  }

  return json;
}
