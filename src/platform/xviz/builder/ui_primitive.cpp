/*
 * File: ui_primitive.cpp
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Tuesday, 7th January 2020 3:06:21 am
 */

#include "builder/ui_primitive.h"

using namespace xviz;

XVIZTreeTableRowBuilder::XVIZTreeTableRowBuilder(int id, const std::vector<std::string>& values, std::optional<int> parent) {
  node_.set_id(id);
  if (parent != std::nullopt) {
    node_.set_parent(parent.value());
  }
  
  for (const auto& value : values) {
    auto added_string_ptr = node_.add_column_values();
    *added_string_ptr = value;
  }
}

XVIZTreeTableRowBuilder::XVIZTreeTableRowBuilder(int id, std::vector<std::string>&& values, std::optional<int> parent) {
  node_.set_id(id);
  if (parent != std::nullopt) {
    node_.set_parent(parent.value());
  }

  for (auto& value : values) {
    auto added_string_ptr = node_.add_column_values();
    *added_string_ptr = std::move(value);
  }
}

XVIZTreeTableRowBuilder& XVIZTreeTableRowBuilder::Children(int id, const std::vector<std::string>& values) {
  children_.emplace_back(id, values);
  return *this;
}

XVIZTreeTableRowBuilder& XVIZTreeTableRowBuilder::Children(int id, std::vector<std::string>&& values) {
  children_.emplace_back(id, std::move(values));
  return *this;
}

std::vector<TreeTableNode> XVIZTreeTableRowBuilder::GetData() {
  std::vector<TreeTableNode> nodes;
  nodes.push_back(std::move(node_));

  for (auto& child : children_) {
    auto child_nodes = child.GetData();
    for (auto& n : child_nodes) {
      nodes.push_back(std::move(n));
    }
  }

  return std::move(nodes);
}

XVIZUIPrimitiveBuilder::XVIZUIPrimitiveBuilder(const std::shared_ptr<Metadata>& metadata) : 
  XVIZBaseBuilder(Category::StreamMetadata_Category_UI_PRIMITIVE, metadata) {
  
  Reset();
  primitives_ = std::make_shared<std::unordered_map<std::string, UIPrimitiveState>>();
}

XVIZUIPrimitiveBuilder& XVIZUIPrimitiveBuilder::Stream(const std::string& stream_id) {
  if (stream_id_.size() != 0) {
    Flush();
  }

  stream_id_ = stream_id;
  return *this;
}

XVIZUIPrimitiveBuilder& XVIZUIPrimitiveBuilder::TreeTable(const std::vector<TreeTableColumn>& tree_table_columns) {
  if (type_ != nullptr) {
    Flush();
  }
  columns_ = std::make_shared<std::vector<TreeTableColumn>>(tree_table_columns);
  type_ = std::make_shared<UIPrimitiveType>(UIPrimitiveType::StreamMetadata_UIPrimitiveType_TREETABLE);
  return *this;
}

XVIZUIPrimitiveBuilder& XVIZUIPrimitiveBuilder::TreeTable(std::vector<TreeTableColumn>&& tree_table_columns) {
  if (type_ != nullptr) {
    Flush();
  }
  columns_ = std::make_shared<std::vector<TreeTableColumn>>(std::move(tree_table_columns));
  type_ = std::make_shared<UIPrimitiveType>(UIPrimitiveType::StreamMetadata_UIPrimitiveType_TREETABLE);
  return *this;
}

XVIZUIPrimitiveBuilder& XVIZUIPrimitiveBuilder::Row(int id, const std::vector<std::string>& values) {
  row_ = std::make_shared<XVIZTreeTableRowBuilder>(id, values);
  type_ = std::make_shared<UIPrimitiveType>(UIPrimitiveType::StreamMetadata_UIPrimitiveType_TREETABLE);
  return *this;
}

XVIZUIPrimitiveBuilder& XVIZUIPrimitiveBuilder::Row(int id, std::vector<std::string>&& values) {
  row_ = std::make_shared<XVIZTreeTableRowBuilder>(id, std::move(values));
  type_ = std::make_shared<UIPrimitiveType>(UIPrimitiveType::StreamMetadata_UIPrimitiveType_TREETABLE);
  return *this;
}

std::shared_ptr<std::unordered_map<std::string, UIPrimitiveState>> XVIZUIPrimitiveBuilder::GetData() {
  if (type_ != nullptr) {
    Flush();
  }

  if (primitives_->size() != 0) {
    return primitives_;
  }

  return nullptr;
}

void XVIZUIPrimitiveBuilder::Flush() {
  XVIZBaseBuilder::Validate();
  FlushPrimitives();
}

void XVIZUIPrimitiveBuilder::FlushPrimitives() {
  if (type_ == nullptr) {
    LOG_ERROR("Please at least indicate a type for ui primitive");
  }
  switch (*type_) {
    case UIPrimitiveType::StreamMetadata_UIPrimitiveType_TREETABLE: {
      if (primitives_->find(stream_id_) == primitives_->end()) {
        (*primitives_)[stream_id_] = UIPrimitiveState();
      }

      if (columns_ == nullptr) {
        LOG_ERROR("Plase first call TreeTable()");
        Reset();
        return;
      }

      auto tree_table_ptr = (*primitives_)[stream_id_].mutable_treetable();
      for (auto& column : *columns_) {
        auto new_column_ptr = tree_table_ptr->add_columns();
        *new_column_ptr = std::move(column);
      }

      if (row_ != nullptr) {
        auto rows = row_->GetData();
        for (auto& row : rows) {
          auto new_row_ptr = tree_table_ptr->add_nodes();
          *new_row_ptr = std::move(row);
        }
      }
      break;
    }
    default:
      LOG_INFO("Unknown type");
  }

  Reset();
}

void XVIZUIPrimitiveBuilder::Reset() {
  type_ = nullptr;
  columns_ = nullptr;
  row_ = nullptr;
}