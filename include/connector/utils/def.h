#ifndef MELLOCOLATE_CONNECTOR_DEF_H_
#define MELLOCOLATE_CONNECTOR_DEF_H_

#include <boost/geometry.hpp>

namespace mellocolate {

using point_3d_t = boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian>;

} // namespace mellocolate


#endif