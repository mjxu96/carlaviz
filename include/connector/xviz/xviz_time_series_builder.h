
#ifndef MELLOCOLATE_XVIZ_TIME_SERIES_BUILDER_H_
#define MELLOCOLATE_XVIZ_TIME_SERIES_BUILDER_H_

#include "connector/utils/def.h"
#include "connector/utils/json.hpp"

#include <boost/optional.hpp>

#include <string>

namespace mellocolate {

class XVIZTimeSeriesBuider {
public:
  XVIZTimeSeriesBuider(std::string name);
  XVIZTimeSeriesBuider& AddTimestamp(double timestamp);
  XVIZTimeSeriesBuider& AddValue(double value);
  XVIZTimeSeriesBuider& AddValue(std::string value);
  XVIZTimeSeriesBuider& AddValue(bool value);

  //std::string GetName() const;
  nlohmann::json GetData() const;

private:
  std::string name_{""};
  boost::optional<double> timestamp_ = boost::none;
  boost::optional<bool> value_bool_ = boost::none;
  boost::optional<double> value_number_ = boost::none;
  boost::optional<std::string> value_string_ = boost::none;
};
  
} // namespace mellocolate



#endif