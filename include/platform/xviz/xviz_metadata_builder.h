/*
 * File: xviz_metadata_builder.h
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Sunday, 7th July 2019 8:52:29 pm
 */

#ifndef MELLOCOLATE_XVIZ_METADATA_BUILDER_H_
#define MELLOCOLATE_XVIZ_METADATA_BUILDER_H_

#include "platform/xviz/utils/metadata_stream.h"
#include "platform/xviz/utils/ui_config.h"
#include "platform/utils/json.hpp"

#include <vector>

namespace mellocolate {

class XVIZMetaDataBuilder {
public:
  XVIZMetaDataBuilder() = default;
  XVIZMetaDataBuilder& AddStream(metadata::Stream stream);
  XVIZMetaDataBuilder& AddUIConfig(metadata::UIConfig ui_config);
  XVIZMetaDataBuilder& SetVersion(std::string version);
  XVIZMetaDataBuilder& SetTime(double start_time, double end_time);
  XVIZMetaDataBuilder& SetMap(std::string map);
  std::string GetMetaData();

private:
  std::vector<metadata::Stream> streams_;
  boost::optional<metadata::UIConfig> ui_config_ = boost::none;
  std::string version_ = "2.0.0";
  boost::optional<double> start_time_ = boost::none;
  boost::optional<double> end_time_ = boost::none;
  boost::optional<std::string> map_ = boost::none;
};

} // namespace mellocolate



#endif