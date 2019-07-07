/*
 * File: def.h
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Saturday, 6th July 2019 1:52:47 pm
 */

#ifndef MELLOCOLATE_PROXY_DEF_H_
#define MELLOCOLATE_PROXY_DEF_H_

#include "connector/utils/macrologger.h"

#include <boost/geometry.hpp>
#include <boost/shared_ptr.hpp>

namespace mellocolate {

using point_3d_t = boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian>;
using point_2d_t = boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian>;


} // namespace mellocolate


#endif