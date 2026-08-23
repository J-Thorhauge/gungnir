// Copyright 2023 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef GUNGNIR_INTERFACE__GUNGNIR_INTERFACE_HPP_
#define GUNGNIR_INTERFACE__GUNGNIR_INTERFACE_HPP_

#include "string"
#include "unordered_map"
#include "vector"
#include "rclcpp/rclcpp.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "myactuator_rmd/myactuator_rmd.hpp"
#include "besfoc_driver/besfoc_driver.hpp"
#include "amt21_encoder_driver/amt21_encoder_driver.hpp"


using hardware_interface::return_type;

namespace gungnir
{



using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class HARDWARE_INTERFACE_PUBLIC RobotSystem : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;


  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

  int homing_sequence(besfoc::CanMotor& motor);

  void retract_motor(besfoc::CanMotor& motor){
    motor.set_acceleration(10000); // Set acceleration to 1000 rpm/s
    motor.set_deceleration(10000); // Set deceleration to 1000 rpm

    motor.set_velocity(1000); // Set velocity to 1000

    rclcpp::sleep_for(std::chrono::milliseconds(200)); // Wait for 1 second

    motor.stop_motor(); // Stop the motor
  }
  void go_to_stop(besfoc::CanMotor& motor, int torque){
   int last_velocity = 0;
    int velocity;
    int hits = 0;

    motor.set_tourque(-torque);
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    motor.get_velocity(velocity);
    last_velocity = velocity;

    while(true){

        motor.get_velocity(velocity);
        int vel_diff = abs(last_velocity) - abs(velocity);

        RCLCPP_INFO(rclcpp::get_logger("test_node"), "Vel Diff: %d, Hits: %d", vel_diff, hits);
        
        if(vel_diff > 30){
            break;
        }
        last_velocity = velocity;
        rclcpp::sleep_for(std::chrono::milliseconds(10));
    }

    RCLCPP_INFO(rclcpp::get_logger("test_node"), "Motor has stopped. Current Velocity: %d", velocity);

    motor.stop_motor(); // Stop the motor
}

protected:

  const int BESFOC_1_CAN_ID = 0x65; // Numeric value: 101
  const int BESFOC_1_GEAR_RATIO = 50*5.14; // Gear ratio for joint 1 (20:1 gearbox and 5.14:1 belt reduction)
  const int AMT21_1_NODE_ADDRESS = 0x54;

  const int MYACTUATOR_2_CAN_ID = 0x02;
  const int MYACTUATOR_3_CAN_ID = 0x03;

  const int BESFOC_4_CAN_ID = 0x68; // Numeric value: 104
  const int BESFOC_4_GEAR_RATIO = 20*2.86; // Gear ratio for joint 4 (50:1 gearbox)
  const int BESFOC_4_CPR = 1000; // Counts per revolution for joint 4 (50000 CPR)

  const int MYACTUATOR_5_CAN_ID = 0x05;
  const int MYACTUATOR_5_GEAR_RATIO = 1;
  const int MYACTUATOR_MAX_VELOCITY = 498; // Max velocity for MyActuator motors in degrees/s
  const int AMT21_5_NODE_ADDRESS = 0x58;

  const int MYACTUATOR_6_CAN_ID = 0x06;
  const int MYACTUATOR_6_GEAR_RATIO = 1;
  const int MYACTUATOR_6_MAX_VELOCITY = 498; // Max velocity for MyActuator motors in degrees/s
  const int AMT21_6_NODE_ADDRESS = 0x5C;


  const bool joint4_is_homed = false; // Set to true if joint 4 is homed, false otherwise

  //Create hardware maps for the motors and encoders. The keys are the joint numbers (starting at 0) and the values are the corresponding motor/encoder objects
  map<int, besfoc::CanMotor> besfoc_motors;
  map<int, AMT21> amt21_encoders;
  map<int, myactuator_rmd::ActuatorInterface> rmd_motors;

  //CAN drivers for each type of motor
  myactuator_rmd::CanDriver* my_actuator_bus = nullptr;
  std::shared_ptr<CanBus> besfoc_bus = nullptr;

  /// The size of this vector is (standard_interfaces_.size() x nr_joints)
  std::vector<double> joint_velocities_command_;
  std::vector<double> joint_position_;
  std::vector<double> joint_velocities_;

  std::unordered_map<std::string, std::vector<std::string>> joint_interfaces = {
    {"position", {}}, {"velocity", {}}};
};

}  // namespace gungnir

#endif  // GUNGNIR_INTERFACE__GUNGNIR_INTERFACE_HPP_
