/*
 * File: xviz_primitive_builder.h
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Sunday, 7th July 2019 8:52:29 pm
 */

#ifndef MELLOCOLATE_XVIZ_PRIMITIVE_BUILDER_H_
#define MELLOCOLATE_XVIZ_PRIMITIVE_BUILDER_H_

#include "platform/utils/json.hpp"
#include "platform/utils/def.h"
#include "platform/utils/utils.h"

#include <boost/optional.hpp>

#include <vector>
#include <string>

namespace mellocolate {

class XVIZPrimitivePolygonBuilder {
public:
  XVIZPrimitivePolygonBuilder(std::vector<point_3d_t> vertices);
  XVIZPrimitivePolygonBuilder& AddId(std::string id);

  nlohmann::json GetData() const;

private:
  std::vector<point_3d_t> vertices_;
  boost::optional<std::string> id_ = boost::none;
};

class XVIZPrimitiveCircleBuilder {
public:
  XVIZPrimitiveCircleBuilder(point_3d_t center, double radius);
  XVIZPrimitiveCircleBuilder& AddId(std::string id);

  nlohmann::json GetData() const;
private:
  point_3d_t center_;
  double radius_;
  boost::optional<std::string> id_ = boost::none;
};

class XVIZPrimitivePointBuilder {
public:
  XVIZPrimitivePointBuilder(std::vector<point_3d_t> points);
  XVIZPrimitivePointBuilder& AddId(std::string id);

  nlohmann::json GetData() const;
private:
  std::vector<point_3d_t> points_;
  boost::optional<std::string> id_ = boost::none;
};

class XVIZPrimitiveImageBuilder {
public:
  XVIZPrimitiveImageBuilder(utils::Image encoded_image);
  nlohmann::json GetData() const;
private:
  utils::Image image_;
};

class XVIZPrimitiveBuider {
public:
  XVIZPrimitiveBuider(std::string name);
  XVIZPrimitiveBuider& AddPolygon(XVIZPrimitivePolygonBuilder polygon);
  XVIZPrimitiveBuider& AddCircle(XVIZPrimitiveCircleBuilder circle);
  XVIZPrimitiveBuider& AddPoints(XVIZPrimitivePointBuilder points);
  XVIZPrimitiveBuider& AddImages(XVIZPrimitiveImageBuilder image);

  std::string GetName() const;
  nlohmann::json GetData() const;
private:
  std::string name_;
  std::vector<XVIZPrimitivePolygonBuilder> polygons_;
  std::vector<XVIZPrimitiveCircleBuilder> circles_;
  std::vector<XVIZPrimitivePointBuilder> points_;
  std::vector<XVIZPrimitiveImageBuilder> images_;
};
  
} // namespace mellocolate

#endif