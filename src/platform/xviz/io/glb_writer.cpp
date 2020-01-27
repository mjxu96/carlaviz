/*
 * File: glb_write.cpp
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Monday, 20th January 2020 4:27:46 pm
 */

#include "io/glb_writer.h"

using namespace xviz;
using namespace fx::gltf;

uint32_t AddPadding(std::vector<uint8_t>& buffer) {
  auto padding_size = ((buffer.size() + 3u) & (~3u)) - buffer.size();
  for (int i = 0; i < padding_size; i++) {
    buffer.push_back(0);
  }
  return padding_size;
}

void AddOneImage(Document& document, std::vector<uint8_t>& buffer, uint32_t image_idx, uint32_t buffer_idx, ::xviz::Image* image_ptr) {
  document.bufferViews.push_back(fx::gltf::BufferView());
  document.bufferViews.back().byteOffset = buffer.size();
  document.bufferViews.back().byteLength = image_ptr->data().size();
  document.bufferViews.back().buffer = 0;

  document.images.push_back(fx::gltf::Image());
  document.images.back().bufferView = buffer_idx;
  document.images.back().mimeType = "image/png";

  buffer.insert(buffer.end(), image_ptr->data().begin(), image_ptr->data().end());
  image_ptr->set_data("#/images/" + std::to_string(image_idx)); 

  document.bufferViews.back().byteLength += AddPadding(buffer);

}

void AddOnePoint(Document& document, std::vector<uint8_t>& buffer, uint32_t point_idx, uint32_t buffer_idx, ::xviz::Point* point_ptr) {
  if (point_ptr->points().list_value().values_size() <= 0u) {
    return;
  }
  auto value_size = point_ptr->points().list_value().values_size();
  document.bufferViews.push_back(fx::gltf::BufferView());
  document.bufferViews.back().byteOffset = buffer.size();
  document.bufferViews.back().byteLength = value_size * 4u;
  document.bufferViews.back().buffer = 0;

  document.accessors.push_back(fx::gltf::Accessor());
  document.accessors.back().bufferView = buffer_idx;
  document.accessors.back().type = fx::gltf::Accessor::Type::Vec3;
  document.accessors.back().componentType = fx::gltf::Accessor::ComponentType::Float;
  document.accessors.back().count = value_size / 3u;

  std::vector<float> points;
  for (auto i = 0; i < value_size; i++) {
    points.push_back((float)(point_ptr->points().list_value().values(i).number_value()));
  }
  uint32_t byte_size = points.size() * 4u;
  auto pointer = reinterpret_cast<const char*>(&points[0]);
  std::stringstream ss;
  ss.write(pointer, byte_size);
  std::string sstr = ss.str();
  buffer.insert(buffer.end(), sstr.begin(), sstr.end());

  point_ptr->mutable_points()->clear_list_value();
  point_ptr->mutable_points()->set_string_value("#/accessors/" + std::to_string(point_idx));


}

void GetStateUpdateData(std::string& sink, xviz::XVIZMessage& message) {
  sink.clear();

  std::vector<uint8_t> buffer;

  Document document;

  auto update = message.GetStateUpdate();

  uint32_t image_idx = 0u;
  uint32_t accessor_idx = 0u;
  for (auto itr = update->mutable_updates()->begin(); itr != update->mutable_updates()->end(); itr++) {
    for (auto& [k, v] : *(itr->mutable_primitives())) {
      for (uint32_t i = 0; i < v.images_size(); i++) {
        AddOneImage(document, buffer, image_idx, (image_idx + accessor_idx), v.mutable_images(i));
        image_idx++;
      }
      for (uint32_t i = 0; i < v.points_size(); i++) {
        AddOnePoint(document, buffer, accessor_idx, (image_idx + accessor_idx), v.mutable_points(i));
        accessor_idx++;
      }
    }
  }

  // for (auto itr = update->mutable_updates()->begin(); itr != update->mutable_updates()->end(); itr++) {
  //   for (auto& [k, v] : *(itr->mutable_primitives())) {
  //   }
  // }

  std::string xviz_str = message.ToObjectString();
  // std::cout << xviz_str << std::endl;

  if (!buffer.empty()) {
    document.buffers.push_back(fx::gltf::Buffer());
    document.buffers.back().byteLength = buffer.size();
    document.buffers.back().data = std::move(buffer);
    std::stringstream ss;
    xviz_str = ",\"xviz\":{\"type\":\"xviz/state_update\",\"data\":" + std::move(xviz_str) + "}}";
    fx::gltf::Save(document, ss, "", true, xviz_str);
    sink = ss.str();
  } else {
    sink = std::move(xviz_str);
  }


  
}

XVIZGLBWriter::XVIZGLBWriter(const std::shared_ptr<std::string>& sink) {
  sink_ = sink;
}

void XVIZGLBWriter::WriteMessage(std::string& sink, xviz::XVIZMessage& message) {
  if (message.GetStateUpdate() != nullptr) {
    GetStateUpdateData(sink, message);
  }
  // auto update = message.GetStateUpdate();
  // auto metadata = message.GetMetadata();

  // if (update != nullptr) {
  //   GetStateUpdateData(sink, update);
  // } else if (metadata != nullptr) {
  // }
}

void XVIZGLBWriter::WriteMessage(std::string& sink, xviz::XVIZMessage&& message) {
  if (message.GetStateUpdate() != nullptr) {
    GetStateUpdateData(sink, message);
  }
}