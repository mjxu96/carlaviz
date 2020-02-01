/*
 * File: metadata.h
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Tuesday, 24th December 2019 8:06:16 pm
 */

#ifndef XVIZ_METADATA_BUILDER_H_
#define XVIZ_METADATA_BUILDER_H_

#include "declarative_ui/ui_builder.h"

#include "utils/macrologger.h"
#include "utils/definitions.h"
#include "utils/utils.h"

#include "message.h"
#include "proto/session.pb.h"

#include <memory>

namespace xviz {

class XVIZMetadataBuilder {
public:
  XVIZMetadataBuilder();
  std::shared_ptr<Metadata> GetData();
  XVIZMessage GetMessage();

  XVIZMetadataBuilder& Stream(const std::string& stream_id);
  // XVIZMetadataBuilder& Stream(std::string&& stream_id);

  XVIZMetadataBuilder& StartTime(double time);
  XVIZMetadataBuilder& EndTime(double time);

  XVIZMetadataBuilder& UI(const std::unordered_map<std::string, XVIZUIBuilder>& ui_builder);
  XVIZMetadataBuilder& UI(std::unordered_map<std::string, XVIZUIBuilder>&& ui_builder);
  XVIZMetadataBuilder& UI(const std::shared_ptr<std::unordered_map<std::string, XVIZUIBuilder>>& ui_builder_ptr);

  XVIZMetadataBuilder& Source(const std::string& source);
  XVIZMetadataBuilder& Source(std::string&& source);
  XVIZMetadataBuilder& Source(const char* source);
  XVIZMetadataBuilder& Unit(const std::string& unit);
  XVIZMetadataBuilder& Unit(std::string&& unit);
  XVIZMetadataBuilder& Unit(const char* unit);

  XVIZMetadataBuilder& Category(Category category);
  XVIZMetadataBuilder& Type(Primitive primitive_type);
  XVIZMetadataBuilder& Type(ScalarType scalar_type);

  XVIZMetadataBuilder& Coordinate(CoordinateType coordinate_type);

  XVIZMetadataBuilder& TransformMatrix(const std::vector<double>& matrix);

  XVIZMetadataBuilder& StreamStyle(const std::string& style_str);
  XVIZMetadataBuilder& StyleClass();

  // TODO to imple
  XVIZMetadataBuilder& LogInfo();

private:
  void Flush();
  void Reset();

  std::shared_ptr<Metadata> data_{nullptr};
  std::shared_ptr<std::unordered_map<std::string, XVIZUIBuilder>> ui_{nullptr};
  std::string stream_id_{};
  StreamMetadata temp_stream_{};
  uint32_t type_ = -1;
  // TODO TMP TYPE
};
  
} // namespace xviz



#endif