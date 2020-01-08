/*
 * File: primitive.cpp
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Tuesday, 17th December 2019 10:00:38 pm
 */

#include "builder/primitive.h"

using namespace xviz;
using Json = nlohmann::json;

template<typename T>
void AddVertices(T& vertice_to_add, const std::shared_ptr<std::vector<double>>& vertices) {
  if (vertices == nullptr) {
    LOG_ERROR("Vertice pointer is NULL");
    return;
  }
  for (const auto& v : *vertices) {
    vertice_to_add.add_vertices(v);
    // vertice_to_add.add_vertices(point.y);
    // vertice_to_add.add_vertices(point.z);
  }
}

template<typename T>
void AddBase(T* ptr_to_add, const std::pair<bool, xviz::PrimitiveBase>& base_to_add) {
  if (base_to_add.first) {
    ptr_to_add->mutable_base()->MergeFrom(base_to_add.second);
  }
}

XVIZPrimitiveBuilder::XVIZPrimitiveBuilder(const std::shared_ptr<Metadata>& metadata) :
  XVIZBaseBuilder(xviz::StreamMetadata::PRIMITIVE, metadata) {
  primitives_ = std::make_shared<std::unordered_map<std::string, PrimitiveState>>();
}

XVIZPrimitiveBuilder& XVIZPrimitiveBuilder::Stream(const std::string& stream_id) {
  if (stream_id_.size() > 0) {
    this->Flush();
  }
  stream_id_ = stream_id;
  return *this;
}

// Polygon
XVIZPrimitiveBuilder& XVIZPrimitiveBuilder::Polygon(const std::vector<double>& vertices) {
  auto vertices_ptr = std::make_shared<std::vector<double>>(vertices);
  return Polygon(vertices_ptr);
}

XVIZPrimitiveBuilder& XVIZPrimitiveBuilder::Polygon(std::vector<double>&& vertices) {
  auto vertices_ptr = std::make_shared<std::vector<double>>(std::move(vertices));
  return Polygon(vertices_ptr);
}

XVIZPrimitiveBuilder& XVIZPrimitiveBuilder::Polygon(const std::shared_ptr<std::vector<double>>& vertices_ptr) {
  if (type_ != nullptr) {
    Flush();
  }
  vertices_ = std::shared_ptr<std::vector<double>>(vertices_ptr);
  type_ = std::make_shared<Primitive>();
  *type_ = Primitive::StreamMetadata_PrimitiveType_POLYGON;
  return *this;
}

// Polyline
XVIZPrimitiveBuilder& XVIZPrimitiveBuilder::Polyline(const std::vector<double>& vertices) {
  auto vertices_ptr = std::make_shared<std::vector<double>>(vertices);
  return Polyline(vertices_ptr);
}

XVIZPrimitiveBuilder& XVIZPrimitiveBuilder::Polyline(std::vector<double>&& vertices) {
  auto vertices_ptr = std::make_shared<std::vector<double>>(std::move(vertices));
  return Polyline(vertices_ptr);
}

XVIZPrimitiveBuilder& XVIZPrimitiveBuilder::Polyline(const std::shared_ptr<std::vector<double>>& vertices_ptr) {
  if (type_ != nullptr) {
    Flush();
  }
  vertices_ = std::shared_ptr<std::vector<double>>(vertices_ptr);
  type_ = std::make_shared<Primitive>();
  *type_ = Primitive::StreamMetadata_PrimitiveType_POLYLINE;
  return *this;
}

// Points
XVIZPrimitiveBuilder& XVIZPrimitiveBuilder::Points(const std::vector<double>& vertices) {
  auto vertices_ptr = std::make_shared<std::vector<double>>(vertices);
  return Points(vertices_ptr);
}

XVIZPrimitiveBuilder& XVIZPrimitiveBuilder::Points(std::vector<double>&& vertices) {
  auto vertices_ptr = std::make_shared<std::vector<double>>(std::move(vertices));
  return Points(vertices_ptr);
}

XVIZPrimitiveBuilder& XVIZPrimitiveBuilder::Points(const std::shared_ptr<std::vector<double>>& vertices_ptr) {
  if (type_ != nullptr) {
    Flush();
  }
  vertices_ = std::shared_ptr<std::vector<double>>(vertices_ptr);
  type_ = std::make_shared<Primitive>();
  *type_ = Primitive::StreamMetadata_PrimitiveType_POINT;
  return *this;
}

// Position
XVIZPrimitiveBuilder& XVIZPrimitiveBuilder::Position(const std::vector<double>& vertices) {
  return Position(std::make_shared<std::vector<double>>(vertices));
}

XVIZPrimitiveBuilder& XVIZPrimitiveBuilder::Position(std::vector<double>&& vertices) {
  return Position(std::make_shared<std::vector<double>>(std::move(vertices)));
}

XVIZPrimitiveBuilder& XVIZPrimitiveBuilder::Position(const std::shared_ptr<std::vector<double>>& vertices_ptr) {
  if (vertices_ptr->size() != 3u) {
    LOG_ERROR("A position must be of the form [x, y, z]");
  }
  vertices_ = vertices_ptr;
  return *this;
}

// Circle
XVIZPrimitiveBuilder& XVIZPrimitiveBuilder::Circle(const std::vector<double>& vertices, double radius) {
  return Circle(std::make_shared<std::vector<double>>(vertices), std::make_shared<double>(radius));
}

XVIZPrimitiveBuilder& XVIZPrimitiveBuilder::Circle(std::vector<double>&& vertices, double radius) {
  return Circle(std::make_shared<std::vector<double>>(std::move(vertices)), std::make_shared<double>(radius));
}

XVIZPrimitiveBuilder& XVIZPrimitiveBuilder::Circle(const std::shared_ptr<std::vector<double>>& vertices_ptr, const std::shared_ptr<double>& radius) {
  if (type_ != nullptr) {
    Flush();
  }

  Position(vertices_ptr);
  radius_ = radius;
  type_ = std::make_shared<Primitive>();
  *type_ = Primitive::StreamMetadata_PrimitiveType_CIRCLE;
  return *this;
}

// Image
XVIZPrimitiveBuilder& XVIZPrimitiveBuilder::Dimensions(uint32_t width_pixel, uint32_t height_pixel) {
  if (image_ == nullptr) {
    LOG_ERROR("An image must be set before call Dimensions()");
    return *this;
  }

  image_->set_width_px(width_pixel);
  image_->set_height_px(height_pixel);
  return *this;
}

XVIZPrimitiveBuilder& XVIZPrimitiveBuilder::Image(const std::string& encoded_data_str) {
  if (type_ != nullptr) {
    Flush();
  }
  type_ = std::make_shared<Primitive>();
  *type_ = Primitive::StreamMetadata_PrimitiveType_IMAGE;
  image_ = std::make_shared<xviz::Image>();
  image_->set_data(encoded_data_str);
  return *this;
}

XVIZPrimitiveBuilder& XVIZPrimitiveBuilder::Image(std::string&& encoded_data_str) {
  if (type_ != nullptr) {
    Flush();
  }
  type_ = std::make_shared<Primitive>();
  *type_ = Primitive::StreamMetadata_PrimitiveType_IMAGE;
  image_ = std::make_shared<xviz::Image>();
  image_->set_data(std::move(encoded_data_str));
  return *this;
}

// Style
XVIZPrimitiveBuilder& XVIZPrimitiveBuilder::Style(const std::string& style_json_str) {
  return Style(JsonStringToStyleObject(style_json_str));
}

XVIZPrimitiveBuilder& XVIZPrimitiveBuilder::Style(std::string&& style_json_str) {
  return Style(JsonStringToStyleObject(std::move(style_json_str)));
}

XVIZPrimitiveBuilder& XVIZPrimitiveBuilder::Style(const Json& style_json) {
  return Style(JsonObjectToStyleObject(style_json));
}

XVIZPrimitiveBuilder& XVIZPrimitiveBuilder::Style(Json&& style_json) {
  return Style(JsonObjectToStyleObject(std::move(style_json)));
}

XVIZPrimitiveBuilder& XVIZPrimitiveBuilder::Style(const std::shared_ptr<StyleObjectValue>& style_object) {
  ValidatePrerequisite();
  style_ = style_object;
  return *this;
}

std::shared_ptr<std::unordered_map<std::string, PrimitiveState>> XVIZPrimitiveBuilder::GetData() {
  if (type_ != nullptr) {
    Flush();
  }
  if (primitives_->empty()) {
    return nullptr;
  }
  return primitives_;
}

void XVIZPrimitiveBuilder::Flush() {
  Validate();
  FlushPrimitives();
}

void XVIZPrimitiveBuilder::Validate() {
  XVIZBaseBuilder::Validate();
  // SUPER :: VALIDATE
  // TODO imple this function
}

void XVIZPrimitiveBuilder::ValidatePrerequisite() {
  if (type_ == nullptr) {
    LOG_ERROR("Start from a primitive first, e.g Polygon(), Image(), etc.");
  }
}

void XVIZPrimitiveBuilder::FlushPrimitives() {
  if (primitives_->find(stream_id_) == primitives_->end()) {
    (*primitives_)[stream_id_] = PrimitiveState();
  }
  auto stream_ptr = &(*primitives_)[stream_id_];
  auto base_pair = FlushPrimitiveBase();
  // auto has_base = base_pair.first;
  // auto base = base_pair.second;

  if (type_ == nullptr) {
    Reset();
    return;
  }
  switch (*type_) {
    case Primitive::StreamMetadata_PrimitiveType_POLYGON:
      {
        auto polygon_ptr = stream_ptr->add_polygons();
        AddVertices<xviz::Polygon>(*polygon_ptr, vertices_);
        AddBase<xviz::Polygon>(polygon_ptr, base_pair);
        // if (has_base) {
        //   auto cur_base_ptr = polygon_ptr->mutable_base();
        //   cur_base_ptr->MergeFrom(base);
        // }
        break;
      }

    case Primitive::StreamMetadata_PrimitiveType_POLYLINE:
      {
        auto polyline_ptr = stream_ptr->add_polylines();
        AddVertices<xviz::Polyline>(*polyline_ptr, vertices_);
        AddBase<xviz::Polyline>(polyline_ptr, base_pair);
        // if (has_base) {
        //   auto cur_base_ptr = polyline_ptr->mutable_base();
        //   cur_base_ptr->MergeFrom(base);
        // }
        break;
      }
    
    case Primitive::StreamMetadata_PrimitiveType_POINT:
      {
        if (vertices_ == nullptr) {
          LOG_ERROR("Vertice pointer is NULL");
          break;
        }
        auto point_ptr = stream_ptr->add_points();
        for (auto v : *vertices_) {
          point_ptr->add_points(v);
        }
        if (colors_ != nullptr) {
          auto colors_ptr = point_ptr->mutable_colors();
          *colors_ptr = std::move(*colors_);
        }
        AddBase<xviz::Point>(point_ptr, base_pair);
        // if (has_base) {
        //   auto cur_base_ptr = point_ptr->mutable_base();
        //   cur_base_ptr->MergeFrom(base);
        // }
        break;
      }

    case Primitive::StreamMetadata_PrimitiveType_IMAGE:
      {
        if (vertices_ != nullptr && vertices_->size() > 2) {
          image_->add_position((*vertices_)[0]);
          image_->add_position((*vertices_)[1]);
          image_->add_position((*vertices_)[2]);
        }
        auto image_ptr = stream_ptr->add_images();
        *image_ptr = std::move(*image_);
        AddBase<xviz::Image>(image_ptr, base_pair);
        // if (has_base) {
        //   auto cur_base_ptr = image_ptr->mutable_base();
        //   cur_base_ptr->MergeFrom(base);
        // }
        break;
      }

    case Primitive::StreamMetadata_PrimitiveType_CIRCLE:
      {
        auto circle_ptr = stream_ptr->add_circles();
        if (vertices_ == nullptr || vertices_->size() != 3) {
          LOG_ERROR("Circle's center must be the form of [x, y, z]");
          break;
        }
        for (auto v : *vertices_) {
          circle_ptr->add_center(v);
        }
        circle_ptr->set_radius(*radius_);
        AddBase<xviz::Circle>(circle_ptr, base_pair);
        // if (has_base) {
        //   auto cur_base_ptr = circle_ptr->mutable_base();
        //   cur_base_ptr->MergeFrom(base);
        // }
        break;
      }
    
    // TEXT, STADIUM,
    default:
      LOG_ERROR("No this type exists %d", *type_);
      return;
  }


  Reset();
}

std::pair<bool, PrimitiveBase> XVIZPrimitiveBuilder::FlushPrimitiveBase() {
  bool has_base = false;
  PrimitiveBase base;

  if (id_ != nullptr) {
    has_base = true;
    auto id_ptr = base.mutable_object_id();
    *id_ptr = *id_;
  }

  if (style_ != nullptr) {
    has_base = true;
    auto style_ptr = base.mutable_style();
    // TODO style?
    style_ptr->MergeFrom(*style_);
    // *style_ptr = *style_;
  }

  if (classes_ != nullptr) {
    has_base = true;
    for (const auto& c : *classes_) {
      auto new_class_ptr = base.add_classes();
      *new_class_ptr = c;
    }
  }


  return {has_base, base};
}

void XVIZPrimitiveBuilder::Reset() {
  type_ = nullptr;

  image_ = nullptr;
  vertices_ = nullptr;
  radius_ = nullptr;
  text_ = nullptr;
  colors_ = nullptr;

  id_ = nullptr;
  style_ = nullptr;
  classes_ = nullptr;
}