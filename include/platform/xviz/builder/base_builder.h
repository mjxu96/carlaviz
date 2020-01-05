/*
 * File: base_builder.h
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Saturday, 7th December 2019 2:22:21 pm
 */

#ifndef XVIZ_BASE_BUILDER_H_
#define XVIZ_BASE_BUILDER_H_

#include "utils/definitions.h"
#include "utils/macrologger.h"

#include "proto/session.pb.h"

#include <string>
#include <memory>

namespace xviz {


class XVIZBaseBuilder {
public:
  XVIZBaseBuilder(Category category, const std::shared_ptr<xviz::Metadata>& metadata);
  // std::shared_ptr<XVIZBaseBuilder> Stream(std::string stream_id);

protected:
  std::string stream_id_{};
  Category category_{};
  std::shared_ptr<xviz::Metadata> metadata_{nullptr};

  void Validate();
  void ValidateMatchMetadata();
  virtual void Flush() = 0;

};

}

#endif