#ifndef MELLOCOLATE_XVIZ_METADATA_BUILDER_H_
#define MELLOCOLATE_XVIZ_METADATA_BUILDER_H_

#include "connector/xviz/utils/metadata_stream.h"
#include "connector/utils/json.hpp"

#include <vector>

namespace mellocolate {

class XVIZMetaDataBuilder {
public:
  XVIZMetaDataBuilder() = default;
  XVIZMetaDataBuilder& AddStream(metadata::Stream stream);
  XVIZMetaDataBuilder& SetVersion(std::string version);
  XVIZMetaDataBuilder& SetTime(double start_time, double end_time);
  XVIZMetaDataBuilder& SetMap(std::string map);
  std::string GetMetaData();

private:
  std::vector<metadata::Stream> streams_;
  std::string version_ = "2.0.0";
  std::optional<double> start_time_;
  std::optional<double> end_time_;
  std::optional<std::string> map_;
};

} // namespace mellocolate



#endif