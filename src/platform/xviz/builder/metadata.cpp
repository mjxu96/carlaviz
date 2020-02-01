/*
 * File: metadata.cpp
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Tuesday, 24th December 2019 8:15:24 pm
 */


#include "builder/metadata.h"

using namespace xviz;


XVIZMetadataBuilder::XVIZMetadataBuilder() {
  data_ = std::make_shared<Metadata>();
  // TODO UI BUILDER
  data_->set_version("2.0.0");
  Reset();
}

std::shared_ptr<Metadata> XVIZMetadataBuilder::GetData() {
  Flush();

  if (ui_ != nullptr) {
    auto ui_config_ptr = data_->mutable_ui_config();

    for (auto& [panel_key, builder] : *ui_) {
      auto ui_panel_info = UIPanelInfo();
      ui_panel_info.set_name(panel_key);
      ui_panel_info.set_type("panel");
      auto uis = builder.GetUI();
      for (auto& ui : uis) {
        auto new_ui = ui_panel_info.add_children();
        *new_ui = std::move(ui);
      }
      // *(ui_panel_info.mutable_config()) = std::move(cfg);
      (*ui_config_ptr)[panel_key] = std::move(ui_panel_info);
    }
  }

  return data_;
}

XVIZMessage XVIZMetadataBuilder::GetMessage() {
  return XVIZMessage(GetData());
}

XVIZMetadataBuilder& XVIZMetadataBuilder::Stream(const std::string& stream_id) {
  if (!stream_id_.empty()) {
    Flush();
  }
  stream_id_ = stream_id;
  return *this;
}

XVIZMetadataBuilder& XVIZMetadataBuilder::StartTime(double time) {
  data_->mutable_log_info()->set_start_time(time);
  return *this;
}

XVIZMetadataBuilder& XVIZMetadataBuilder::EndTime(double time) {
  data_->mutable_log_info()->set_end_time(time);
  return *this;
}

XVIZMetadataBuilder& XVIZMetadataBuilder::UI(const std::unordered_map<std::string, XVIZUIBuilder>& ui_builder) {
  return UI(std::make_shared<std::unordered_map<std::string, XVIZUIBuilder>>(ui_builder));
}

XVIZMetadataBuilder& XVIZMetadataBuilder::UI(std::unordered_map<std::string, XVIZUIBuilder>&& ui_builder) {
  return UI(std::make_shared<std::unordered_map<std::string, XVIZUIBuilder>>(std::move(ui_builder)));
}

XVIZMetadataBuilder& XVIZMetadataBuilder::UI(const std::shared_ptr<std::unordered_map<std::string, XVIZUIBuilder>>& ui_builder_ptr) {
  ui_ = ui_builder_ptr;
  return *this;
}

XVIZMetadataBuilder& XVIZMetadataBuilder::Source(const std::string& source) {
  temp_stream_.set_source(source);
  return *this;
}

XVIZMetadataBuilder& XVIZMetadataBuilder::Source(std::string&& source) {
  temp_stream_.set_source(std::move(source));
  return *this;
}

XVIZMetadataBuilder& XVIZMetadataBuilder::Source(const char* source) {
  temp_stream_.set_source(source);
  return *this;
}

XVIZMetadataBuilder& XVIZMetadataBuilder::Unit(const std::string& unit) {
  temp_stream_.set_units(unit);
  return *this;
}

XVIZMetadataBuilder& XVIZMetadataBuilder::Unit(std::string&& unit) {
  temp_stream_.set_units(std::move(unit));
  return *this;
}

XVIZMetadataBuilder& XVIZMetadataBuilder::Unit(const char* unit) {
  temp_stream_.set_units(unit);
  return *this;
}

XVIZMetadataBuilder& XVIZMetadataBuilder::Category(xviz::Category category) {
  temp_stream_.set_category(category);
  return *this;
}

XVIZMetadataBuilder& XVIZMetadataBuilder::Coordinate(CoordinateType coordinate_type) {
  temp_stream_.set_coordinate(coordinate_type);
  return *this;
}

XVIZMetadataBuilder& XVIZMetadataBuilder::Type(Primitive primitive_type) {
  type_ = (uint32_t)primitive_type;
  return *this;
}

XVIZMetadataBuilder& XVIZMetadataBuilder::Type(ScalarType scalar_type) {
  type_ = (uint32_t)scalar_type;
  return *this;
}

XVIZMetadataBuilder& XVIZMetadataBuilder::TransformMatrix(const std::vector<double>& matrix) {
  for (auto v : matrix) {
    auto v_ptr = temp_stream_.mutable_transform()->Add();
    *v_ptr = v;
  }
  return *this;
}

XVIZMetadataBuilder& XVIZMetadataBuilder::StreamStyle(const std::string& style_str) {
  auto stream_style = temp_stream_.mutable_stream_style();
  stream_style->MergeFrom(*JsonStringToStyleStream(style_str));
  return *this;
}

// TODO 
XVIZMetadataBuilder& XVIZMetadataBuilder::StyleClass() {
  LOG_ERROR("NOT IMPLEMENT STYLECLASS");
  throw std::runtime_error("not implement STYLECLASS");
  return *this;
}

// TODO 
XVIZMetadataBuilder& XVIZMetadataBuilder::LogInfo() {
  LOG_ERROR("NOT IMPLEMENT LOGINFO");
  throw std::runtime_error("not implement LOGINFO");
  return *this;
}

void XVIZMetadataBuilder::Reset() {
  stream_id_ = "";
  temp_stream_ = StreamMetadata();
  // TODO is this value correct?
  type_ = 0u;
}

void XVIZMetadataBuilder::Flush() {
  if (stream_id_.empty()) {
    Reset();
    return;
  }  

  auto category_int = (uint32_t)(temp_stream_.category());
  if (category_int == 1u || category_int == 5u) {
    temp_stream_.set_primitive_type((Primitive)(type_));
  } else if (category_int == 2u || category_int == 3u) {
    temp_stream_.set_scalar_type((ScalarType)type_);
  }

  auto streams = data_->mutable_streams();
  (*streams)[stream_id_].MergeFrom(temp_stream_);

  Reset();
}