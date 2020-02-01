/*
 * File: utils.cpp
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Tuesday, 24th December 2019 8:42:41 pm
 */

#include "utils/utils.h"

using Json = nlohmann::json;

std::shared_ptr<xviz::StyleObjectValue> xviz::JsonObjectToStyleObject(const nlohmann::json& json) {
  auto style_object = std::make_shared<xviz::StyleObjectValue>();
  if (json.find("fill_color") != json.end()) {
    auto fill_color = json.value("fill_color", "#FFFFFF");
    style_object->set_fill_color(fill_color.c_str());
  }

  if (json.find("stroke_color") != json.end()) {
    auto stroke_color = json.value("stroke_color", "#FFFFFF");
    style_object->set_stroke_color(stroke_color);
  }

  if (json.find("stroke_width") != json.end()) {
    auto stroke_width = json.value("stroke_width", 1.0);
    style_object->set_stroke_width(stroke_width);
  }

  if (json.find("radius") != json.end()) {
    auto radius = json.value("radius", 1.0);
    style_object->set_radius(radius);
  }

  if (json.find("text_size") != json.end()) {
    auto text_size = json.value("text_size", 1.0);
    style_object->set_text_size(text_size);
  }

  if (json.find("text_rotation") != json.end()) {
    auto text_rotation = json.value("text_rotation", 0.0);
    style_object->set_text_rotation(text_rotation);
  }

  if (json.find("text_anchor") != json.end()) {
    LOG_ERROR("TEXT ANCHOR STYLE NOT IMPLEMENTED");
    throw std::runtime_error("not implement");
  }

  if (json.find("text_baseline") != json.end()) {
    LOG_ERROR("TEXT BASELINE STYLE NOT IMPLEMENTED");
    throw std::runtime_error("not implement");
  }

  if (json.find("height") != json.end()) {
    auto height = json.value("height", 1.0);
    style_object->set_height(height);
  }
  return style_object;
}

std::shared_ptr<xviz::StyleObjectValue> xviz::JsonObjectToStyleObject(nlohmann::json&& json) {
  auto style_object = std::make_shared<xviz::StyleObjectValue>();
  if (json.find("fill_color") != json.end()) {
    auto fill_color = json.value("fill_color", "#FFFFFF");
    style_object->set_fill_color(fill_color.c_str());
  }

  if (json.find("stroke_color") != json.end()) {
    auto stroke_color = json.value("stroke_color", "#FFFFFF");
    style_object->set_stroke_color(stroke_color);
  }

  if (json.find("stroke_width") != json.end()) {
    auto stroke_width = json.value("stroke_width", 1.0);
    style_object->set_stroke_width(stroke_width);
  }

  if (json.find("radius") != json.end()) {
    auto radius = json.value("radius", 1.0);
    style_object->set_radius(radius);
  }

  if (json.find("text_size") != json.end()) {
    auto text_size = json.value("text_size", 1.0);
    style_object->set_text_size(text_size);
  }

  if (json.find("text_rotation") != json.end()) {
    auto text_rotation = json.value("text_rotation", 0.0);
    style_object->set_text_rotation(text_rotation);
  }

  if (json.find("text_anchor") != json.end()) {
    LOG_ERROR("TEXT ANCHOR STYLE NOT IMPLEMENTED");
    throw std::runtime_error("not implement");
  }

  if (json.find("text_baseline") != json.end()) {
    LOG_ERROR("TEXT BASELINE STYLE NOT IMPLEMENTED");
    throw std::runtime_error("not implement");
  }

  if (json.find("height") != json.end()) {
    auto height = json.value("height", 1.0);
    style_object->set_height(height);
  }
  return style_object;
}

std::shared_ptr<xviz::StyleObjectValue> xviz::JsonStringToStyleObject(const std::string& json_str) {
  return JsonObjectToStyleObject(Json::parse(json_str));
}

std::shared_ptr<xviz::StyleObjectValue> xviz::JsonStringToStyleObject(std::string&& json_str) {
  return JsonObjectToStyleObject(Json::parse(std::move(json_str)));
}

std::shared_ptr<xviz::StyleStreamValue> xviz::JsonObjectToStyleStream(const nlohmann::json& json) {
  auto style_stream = std::make_shared<xviz::StyleStreamValue>();
  if (json.find("fill_color") != json.end()) {
    auto fill_color = json.value("fill_color", "#FFFFFF");
    style_stream->set_fill_color(fill_color.c_str());
  }

  if (json.find("stroke_color") != json.end()) {
    auto stroke_color = json.value("stroke_color", "#FFFFFF");
    style_stream->set_stroke_color(stroke_color);
  }

  if (json.find("stroke_width") != json.end()) {
    auto stroke_width = json.value("stroke_width", 1.0);
    style_stream->set_stroke_width(stroke_width);
  }

  if (json.find("radius") != json.end()) {
    auto radius = json.value("radius", 1.0);
    style_stream->set_radius(radius);
  }

  if (json.find("text_size") != json.end()) {
    auto text_size = json.value("text_size", 1.0);
    style_stream->set_text_size(text_size);
  }

  if (json.find("text_rotation") != json.end()) {
    auto text_rotation = json.value("text_rotation", 0.0);
    style_stream->set_text_rotation(text_rotation);
  }

  if (json.find("text_anchor") != json.end()) {
    LOG_ERROR("TEXT ANCHOR STYLE NOT IMPLEMENTED");
    throw std::runtime_error("not implement");
  }

  if (json.find("text_baseline") != json.end()) {
    LOG_ERROR("TEXT BASELINE STYLE NOT IMPLEMENTED");
    throw std::runtime_error("not implement");
  }

  if (json.find("height") != json.end()) {
    auto height = json.value("height", 1.0);
    style_stream->set_height(height);
  }

  if (json.find("radius_min_pixels") != json.end()) {
    auto raidus_min_pixels = json.value("radius_min_pixels", 1u);
    style_stream->set_radius_min_pixels(raidus_min_pixels);
  }

  if (json.find("radius_max_pixels") != json.end()) {
    auto raidus_max_pixels = json.value("radius_max_pixels", 1u);
    style_stream->set_radius_max_pixels(raidus_max_pixels);
  }

  if (json.find("stroke_width_min_pixels") != json.end()) {
    auto stroke_width_min_pixels = json.value("stroke_width_min_pixels", 1u);
    style_stream->set_stroke_width_min_pixels(stroke_width_min_pixels);
  }

  if (json.find("stroke_width_max_pixels") != json.end()) {
    auto stroke_width_max_pixels = json.value("stroke_width_max_pixels", 1u);
    style_stream->set_stroke_width_max_pixels(stroke_width_max_pixels);
  }

  if (json.find("opacity") != json.end()) {
    auto opacity = json.value("opacity", 0.0);
    style_stream->set_opacity(opacity);
  }

  if (json.find("stroked") != json.end()) {
    auto stroked = json.value("stroked", false);
    style_stream->set_stroked(stroked);
  }

  if (json.find("filled") != json.end()) {
    auto filled = json.value("filled", false);
    style_stream->set_filled(filled);
  }

  if (json.find("extruded") != json.end()) {
    auto extruded = json.value("extruded", false);
    style_stream->set_extruded(extruded);
  }

  if (json.find("radius_pixels") != json.end()) {
    auto radius_pixels = json.value("radius_pixels", 1u);
    style_stream->set_radius_pixels(radius_pixels);
  }

  if (json.find("font_weight") != json.end()) {
    auto font_weight = json.value("font_weight", 1u);
    style_stream->set_font_weight(font_weight);
  }

  if (json.find("font_family") != json.end()) {
    LOG_ERROR("FONT FAMILY STYLE NOT IMPLEMENTED");
    throw std::runtime_error("not implemented");
  }

  if (json.find("point_cloud_mode") != json.end()) {
    auto point_cloud_mode = json.value("point_cloud_mode", "distance_to_vehicle");
    style_stream->set_point_cloud_mode(point_cloud_mode);
  }
  return style_stream;
}

std::shared_ptr<xviz::StyleStreamValue> xviz::JsonObjectToStyleStream(nlohmann::json&& json) {
  auto style_stream = std::make_shared<xviz::StyleStreamValue>();
  if (json.find("fill_color") != json.end()) {
    auto fill_color = json.value("fill_color", "#FFFFFF");
    style_stream->set_fill_color(fill_color.c_str());
  }

  if (json.find("stroke_color") != json.end()) {
    auto stroke_color = json.value("stroke_color", "#FFFFFF");
    style_stream->set_stroke_color(stroke_color);
  }

  if (json.find("stroke_width") != json.end()) {
    auto stroke_width = json.value("stroke_width", 1.0);
    style_stream->set_stroke_width(stroke_width);
  }

  if (json.find("radius") != json.end()) {
    auto radius = json.value("radius", 1.0);
    style_stream->set_radius(radius);
  }

  if (json.find("text_size") != json.end()) {
    auto text_size = json.value("text_size", 1.0);
    style_stream->set_text_size(text_size);
  }

  if (json.find("text_rotation") != json.end()) {
    auto text_rotation = json.value("text_rotation", 0.0);
    style_stream->set_text_rotation(text_rotation);
  }

  if (json.find("text_anchor") != json.end()) {
    LOG_ERROR("TEXT ANCHOR STYLE NOT IMPLEMENTED");
    throw std::runtime_error("not implement");
  }

  if (json.find("text_baseline") != json.end()) {
    LOG_ERROR("TEXT BASELINE STYLE NOT IMPLEMENTED");
    throw std::runtime_error("not implement");
  }

  if (json.find("height") != json.end()) {
    auto height = json.value("height", 1.0);
    style_stream->set_height(height);
  }

  if (json.find("radius_min_pixels") != json.end()) {
    auto raidus_min_pixels = json.value("radius_min_pixels", 1u);
    style_stream->set_radius_min_pixels(raidus_min_pixels);
  }

  if (json.find("radius_max_pixels") != json.end()) {
    auto raidus_max_pixels = json.value("radius_max_pixels", 1u);
    style_stream->set_radius_max_pixels(raidus_max_pixels);
  }

  if (json.find("stroke_width_min_pixels") != json.end()) {
    auto stroke_width_min_pixels = json.value("stroke_width_min_pixels", 1u);
    style_stream->set_stroke_width_min_pixels(stroke_width_min_pixels);
  }

  if (json.find("stroke_width_max_pixels") != json.end()) {
    auto stroke_width_max_pixels = json.value("stroke_width_max_pixels", 1u);
    style_stream->set_stroke_width_max_pixels(stroke_width_max_pixels);
  }

  if (json.find("opacity") != json.end()) {
    auto opacity = json.value("opacity", 0.0);
    style_stream->set_opacity(opacity);
  }

  if (json.find("stroked") != json.end()) {
    auto stroked = json.value("stroked", false);
    style_stream->set_stroked(stroked);
  }

  if (json.find("filled") != json.end()) {
    auto filled = json.value("filled", false);
    style_stream->set_filled(filled);
  }

  if (json.find("extruded") != json.end()) {
    auto extruded = json.value("extruded", false);
    style_stream->set_extruded(extruded);
  }

  if (json.find("radius_pixels") != json.end()) {
    auto radius_pixels = json.value("radius_pixels", 1u);
    style_stream->set_radius_pixels(radius_pixels);
  }

  if (json.find("font_weight") != json.end()) {
    auto font_weight = json.value("font_weight", 1u);
    style_stream->set_font_weight(font_weight);
  }

  if (json.find("font_family") != json.end()) {
    LOG_ERROR("FONT FAMILY STYLE NOT IMPLEMENTED");
    throw std::runtime_error("not implemented");
  }

  if (json.find("point_cloud_mode") != json.end()) {
    auto point_cloud_mode = json.value("point_cloud_mode", "distance_to_vehicle");
    style_stream->set_point_cloud_mode(point_cloud_mode);
  }
  return style_stream;
}

std::shared_ptr<xviz::StyleStreamValue> xviz::JsonStringToStyleStream(const std::string& json_str) {
  return JsonObjectToStyleStream(Json::parse(json_str));
}

std::shared_ptr<xviz::StyleStreamValue> xviz::JsonStringToStyleStream(std::string&& json_str) {
  return JsonObjectToStyleStream(Json::parse(std::move(json_str)));
}

