
#ifndef MELLOCOLATE_XVIZ_POSE_BUILDER_H_
#define MELLOCOLATE_XVIZ_POSE_BUILDER_H_

#include "connector/utils/json.hpp"
#include "connector/utils/def.h"

#include <boost/geometry.hpp>
#include <boost/optional.hpp>

#include <string>


namespace mellocolate {


class XVIZPoseBuilder {
public:
  XVIZPoseBuilder(std::string name);
  XVIZPoseBuilder& AddTimestamp(double timestamp);
  XVIZPoseBuilder& AddMapOrigin(point_3d_t map_origin);
  XVIZPoseBuilder& AddPosition(point_3d_t position);
  XVIZPoseBuilder& AddOrientation(point_3d_t orientation);

  std::string GetName() const;
  nlohmann::json GetData() const;

private:
  std::string name_{""};
  boost::optional<double> timestamp_ = boost::none;
  boost::optional<point_3d_t> map_origin_ = boost::none;
  boost::optional<point_3d_t> position_ = boost::none;
  boost::optional<point_3d_t> orientation_ = boost::none;
};

} // namespace mellocolate


#endif