
#ifndef MELLOCOLATE_XVIZ_PRIMITIVE_BUILDER_H_
#define MELLOCOLATE_XVIZ_PRIMITIVE_BUILDER_H_

#include "connector/utils/json.hpp"
#include "connector/utils/def.h"

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

class XVIZPrimitiveBuider {
public:
  XVIZPrimitiveBuider(std::string name);
  XVIZPrimitiveBuider& AddPolygon(XVIZPrimitivePolygonBuilder polygon);
  XVIZPrimitiveBuider& AddCircle(XVIZPrimitiveCircleBuilder circle);

  std::string GetName() const;
  nlohmann::json GetData() const;
private:
  std::string name_;
  std::vector<XVIZPrimitivePolygonBuilder> polygons_;
  std::vector<XVIZPrimitiveCircleBuilder> circles_;
};
  
} // namespace mellocolate

#endif