
#include "connector/utils/package.h"

namespace rothberg {
namespace utils {


Package::Package(boost::shared_ptr<carla::client::World> world_ptr) :
  world_ptr_(std::move(world_ptr)) {
  map_ptr_ = world_ptr_->GetMap();
}

void Package::TmpOutput() {
  std::cout << "road count: " << map_detail_->GetMap().GetRoads().size() << std::endl;
  std::cout << "actor number: " << (*actor_list_ptr_).size() << std::endl;
  for (auto actor_ptr : *actor_list_ptr_) {
    std::cout << "--actor: " << actor_ptr->GetId() << " (" << actor_ptr->GetLocation().x <<
      ", " << actor_ptr->GetLocation().y << ") type: " << actor_ptr->GetTypeId() << std::endl;
  }
}

void Package::Update() {
  boost::optional<carla::road::Map> map_option = carla::opendrive::OpenDriveParser::Load(map_ptr_->GetOpenDrive());
  if (map_option != boost::none) {
    map_detail_ = boost::make_shared<carla::road::Map>(std::move(map_option.get()));
  }
  actor_list_ptr_ = world_ptr_->GetActors();
}

boost::shared_ptr<carla::client::ActorList> Package::GetActorListPtr() const {
  return actor_list_ptr_;
}
}
}