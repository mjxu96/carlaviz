/*
 * File: base_builder.cpp
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Saturday, 7th December 2019 2:44:22 pm
 */

#include "builder/base_builder.h"

using namespace xviz;

std::unordered_map<Primitive, std::unordered_set<std::string>> 
  XVIZBaseBuilder::primitive_style_map_ = 
  {
    {
      Primitive::StreamMetadata_PrimitiveType_CIRCLE, {
        "opacity",
        "stroked",
        "filled",
        "stroke_color",
        "fill_color",
        "radius",
        "radius_min_pixels",
        "radius_max_pixels",
        "stroke_width",
        "stroke_width_min_pixels",
        "stroke_width_max_pixels"
      }
    },
    {
      Primitive::StreamMetadata_PrimitiveType_POINT, {
        "opacity",
        "fill_color",
        "radius_pixels",
        "point_color_mode",
        "point_color_domain"
      }
    },
    {
      Primitive::StreamMetadata_PrimitiveType_POLYGON, {
        "stroke_color",
        "fill_color",
        "stroke_width",
        "stroke_width_min_pixels",
        "stroke_width_max_pixels",
        "height",
        "opacity",
        "stroked",
        "filled",
        "extruded"
      }
    },
    {
      Primitive::StreamMetadata_PrimitiveType_POLYLINE, {
        "opacity",
        "stroke_color",
        "stroke_width",
        "stroke_width_min_pixels",
        "stroke_width_max_pixels"
      }
    },
    {
      Primitive::StreamMetadata_PrimitiveType_TEXT, {
        "opacity",
        "font_family",
        "font_weight",
        "text_size",
        "text_rotation",
        "text_anchor",
        "text_baseline",
        "fill_color"
      }
    },
    {
      Primitive::StreamMetadata_PrimitiveType_STADIUM, {
        "opacity",
        "fill_color",
        "radius",
        "radius_min_pixels",
        "radius_max_pixels"
      }
    }
  };

XVIZBaseBuilder::XVIZBaseBuilder(Category category, const std::shared_ptr<xviz::Metadata>& metadata) {
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