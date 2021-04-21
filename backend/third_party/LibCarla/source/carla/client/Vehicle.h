// Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/client/Actor.h"
#include "carla/rpc/TrafficLightState.h"
#include "carla/rpc/VehicleLightState.h"
#include "carla/rpc/VehicleControl.h"
#include "carla/rpc/VehiclePhysicsControl.h"
#include "carla/trafficmanager/TrafficManager.h"

using carla::traffic_manager::constants::Networking::TM_DEFAULT_PORT;

namespace carla {

namespace traffic_manager {
  class TrafficManager;
}

namespace client {

  class TrafficLight;

  class Vehicle : public Actor {
  public:

    using Control = rpc::VehicleControl;
    using PhysicsControl = rpc::VehiclePhysicsControl;
    using LightState = rpc::VehicleLightState::LightState;
    using TM = traffic_manager::TrafficManager;

    explicit Vehicle(ActorInitializer init);

    /// Switch on/off this vehicle's autopilot.
    void SetAutopilot(bool enabled = true, uint16_t tm_port = TM_DEFAULT_PORT);

    /// Apply @a control to this vehicle.
    void ApplyControl(const Control &control);

    /// Apply physics control to this vehicle.
    void ApplyPhysicsControl(const PhysicsControl &physics_control);

    /// Sets a @a LightState to this vehicle.
    void SetLightState(const LightState &light_state);

    /// Return the control last applied to this vehicle.
    ///
    /// @note This function does not call the simulator, it returns the data
    /// received in the last tick.
    Control GetControl() const;

    /// Return the physics control last applied to this vehicle.
    ///
    /// @warning This function does call the simulator.
    PhysicsControl GetPhysicsControl() const;

    /// Return the current open lights (LightState) of this vehicle.
    ///
    /// @note This function does not call the simulator, it returns the data
    /// received in the last tick.
    LightState GetLightState() const;

    /// Return the speed limit currently affecting this vehicle.
    ///
    /// @note This function does not call the simulator, it returns the data
    /// received in the last tick.
    float GetSpeedLimit() const;

    /// Return the state of the traffic light currently affecting this vehicle.
    ///
    /// @return Green If no traffic light is affecting the vehicle.
    ///
    /// @note This function does not call the simulator, it returns the data
    /// received in the last tick.
    rpc::TrafficLightState GetTrafficLightState() const;

    /// Return whether a traffic light is affecting this vehicle.
    ///
    /// @note This function does not call the simulator, it returns the data
    /// received in the last tick.
    bool IsAtTrafficLight();

    /// Retrieve the traffic light actor currently affecting this vehicle.
    SharedPtr<TrafficLight> GetTrafficLight() const;

    /// Enables CarSim simulation if it is availiable
    void EnableCarSim(std::string simfile_path);

    /// Enables the use of CarSim internal road definition instead of unreal's
    void UseCarSimRoad(bool enabled);

  private:

    const bool _is_control_sticky;

    Control _control;
  };

} // namespace client
} // namespace carla
