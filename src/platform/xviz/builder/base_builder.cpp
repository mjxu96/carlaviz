/*
 * File: base_builder.cpp
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Saturday, 7th December 2019 2:44:22 pm
 */

#include "builder/base_builder.h"

using namespace xviz;

XVIZBaseBuilder::XVIZBaseBuilder(Category category, std::shared_ptr<xviz::Metadata> metadata) {
  category_ = category;
  metadata_ = metadata;
}

void XVIZBaseBuilder::Validate() {
  ValidateMatchMetadata();
}

void XVIZBaseBuilder::ValidateMatchMetadata() {
  if (metadata_ == nullptr) {
    LOG_WARNING("Metadata is missing.");
  } else if (metadata_->streams().find(stream_id_) == metadata_->streams().end()) {
    LOG_WARNING("%s is not defined in metadata.", stream_id_.c_str());
  } else {
    auto cat = (*(metadata_->streams().find(stream_id_))).second.category();
    if (cat != category_) {
      LOG_WARNING("Stream %s category %s does not match metadata definition %s", stream_id_.c_str(), 
        StreamMetadata::Category_Name(category_).c_str(), StreamMetadata::Category_Name(cat).c_str());
    }
  }
}

// std::shared_ptr<XVIZBaseBuilder> XVIZBaseBuilder::Stream(std::string stream_id) {
//   if (stream_id_.size() != 0) {
//     this->Flush();
//   }
//   stream_id_ = std::move(stream_id);
//   return shared_from_this();
// }