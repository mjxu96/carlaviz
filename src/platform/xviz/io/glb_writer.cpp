/*
 * File: glb_write.cpp
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Monday, 20th January 2020 4:27:46 pm
 */

#include "io/glb_writer.h"

using namespace xviz;
using namespace fx::gltf;

void AddOneImage(Document& document, std::vector<uint8_t>& buffer, uint32_t image_idx, ::xviz::Image* image_ptr) {
  document.bufferViews.push_back(fx::gltf::BufferView());
  document.bufferViews.back().byteOffset = buffer.size();
  document.bufferViews.back().byteLength = image_ptr->data().size();
  document.bufferViews.back().buffer = 0;

  document.images.push_back(fx::gltf::Image());
  document.images.back().bufferView = image_idx;
  document.images.back().mimeType = "image/png";
  document.images.back().width = image_ptr->width_px();
  document.images.back().height = image_ptr->height_px();

  std::vector<uint8_t> tmp_data = std::vector<uint8_t>(image_ptr->data().begin(), image_ptr->data().end());
  buffer.insert(buffer.end(), tmp_data.begin(), tmp_data.end());
  image_ptr->set_data("#/images/" + std::to_string(image_idx)); 
}

void GetStateUpdateData(std::string& sink, xviz::XVIZMessage& message) {
  sink.clear();

  std::vector<uint8_t> buffer;

  Document document;
  document.buffers.push_back(fx::gltf::Buffer());

  auto update = message.GetStateUpdate();

  uint32_t image_idx = 0u;
  for (auto itr = update->mutable_updates()->begin(); itr != update->mutable_updates()->end(); itr++) {
    for (auto& [k, v] : *(itr->mutable_primitives())) {
      for (uint32_t i = 0; i < v.images_size(); i++) {
        AddOneImage(document, buffer, image_idx, v.mutable_images(i));
        image_idx++;
      }
    }
  }

  document.buffers.back().byteLength = buffer.size();
  document.buffers.back().data = std::move(buffer);

  std::stringstream ss;
  fx::gltf::Save(document, ss, "", true);
  sink = std::move(ss.str());

  std::string xviz_str = message.ToObjectString();
  xviz_str = "\"xviz\":{\"type\":\"xviz/status_update\",\"data\":" + std::move(xviz_str) + "}";
  
  auto first_left_bracket = sink.find_first_of('{');
  sink = std::move(sink.substr(0, first_left_bracket + 1)) + std::move(xviz_str) + "," + std::move(sink.substr(first_left_bracket + 1));
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

}