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

#include "gungnir_interface/gungnir_interface.hpp"
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include <cmath>


namespace gungnir
{
CallbackReturn RobotSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  //Initialize the can drivers
  RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Initializing CAN interfaces...");
  besfoc_bus = std::make_shared<CanBus>("can0");
  besfoc_bus->connect();

  my_actuator_bus = new myactuator_rmd::CanDriver("can0");
   //Add the motors and encoders to the hardware maps
  RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Initializing motors and encoders...");


  //Joint 2 motor(MyActuator RMD)
  RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Initializing MyActuator motors for joint 2");
  auto [rmd_it_2, rmd_inserted_2] = rmd_motors.emplace(1, myactuator_rmd::ActuatorInterface(*my_actuator_bus, MYACTUATOR_2_CAN_ID));
  rmd_it_2->second.setAcceleration(30000, myactuator_rmd::AccelerationType::VELOCITY_PLANNING_ACCELERATION);
  rmd_it_2->second.setAcceleration(30000, myactuator_rmd::AccelerationType::VELOCITY_PLANNING_DECELERATION);


  //Joint 3 motor(MyActuator RMD)
  RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Initializing MyActuator motors for joint 3");
  auto [rmd_it_3, rmd_inserted_3] = rmd_motors.emplace(2, myactuator_rmd::ActuatorInterface(*my_actuator_bus, MYACTUATOR_3_CAN_ID));
  rmd_it_3->second.setAcceleration(30000, myactuator_rmd::AccelerationType::VELOCITY_PLANNING_ACCELERATION);
  rmd_it_3->second.setAcceleration(30000, myactuator_rmd::AccelerationType::VELOCITY_PLANNING_DECELERATION);


  //Joint 5 motor and Encoder(MyActuator RMD and AMT21)
  RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Initializing MyActuator motors for joint 5");
  auto [rmd_it_5, rmd_inserted_5] = rmd_motors.emplace(4, myactuator_rmd::ActuatorInterface(*my_actuator_bus, MYACTUATOR_5_CAN_ID));
  rmd_it_5->second.setAcceleration(30000, myactuator_rmd::AccelerationType::VELOCITY_PLANNING_ACCELERATION);
  rmd_it_5->second.setAcceleration(30000, myactuator_rmd::AccelerationType::VELOCITY_PLANNING_DECELERATION);
  
  amt21_encoders.try_emplace(4, AMT21_5_NODE_ADDRESS, "/dev/ttyUSB0", 115200, true);
  auto encoder_it_5 = amt21_encoders.find(4);
  encoder_it_5->second.setZero(); // Reset encoder at address 0x58

  //Joint 6 motor and Encoder(MyActuator RMD and AMT21)
  RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Initializing MyActuator motors for joint 6");
  auto [rmd_it_6, rmd_inserted_6] = rmd_motors.emplace(5, myactuator_rmd::ActuatorInterface(*my_actuator_bus, MYACTUATOR_6_CAN_ID));
  rmd_it_6->second.setAcceleration(30000, myactuator_rmd::AccelerationType::VELOCITY_PLANNING_ACCELERATION);
  rmd_it_6->second.setAcceleration(30000, myactuator_rmd::AccelerationType::VELOCITY_PLANNING_DECELERATION);

  amt21_encoders.try_emplace(5, AMT21_6_NODE_ADDRESS, "/dev/ttyUSB0", 115200, false);
  auto encoder_it_6 = amt21_encoders.find(5);
  encoder_it_6->second.setZero(); // Reset encoder at address 0x5C  
  
  //Joint 1 motor and encoder(BesFoc and AMT21)
  RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Initializing BesFoc motor and AMT21 encoder for joint 1...");
  auto [besfoc_it, besfoc_inserted] = besfoc_motors.emplace(0, besfoc::CanMotor(BESFOC_1_CAN_ID, besfoc_bus));
  (void)besfoc_inserted;
  besfoc_it->second.set_acceleration(10000); // Set acceleration to 1000 rpm/s
  besfoc_it->second.set_deceleration(10000); // Set deceleration to 1000 rpm/s
  besfoc_it->second.set_mode(besfoc::SPEED_MODE); // Set mode to velocity control

  amt21_encoders.try_emplace(0, AMT21_1_NODE_ADDRESS, "/dev/ttyUSB0", 115200, true);
  auto encoder_it = amt21_encoders.find(0);
  encoder_it->second.setZero(); // Reset encoder at address 0x54  


  //Joint 4 motor, no encoder(BesFoc)
  RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Initializing BesFoc motor for joint 4...");
  auto [besfoc_it_4, besfoc_inserted_4] = besfoc_motors.emplace(3, besfoc::CanMotor(BESFOC_4_CAN_ID, besfoc_bus));
  (void)besfoc_inserted_4;
  besfoc_it_4->second.set_acceleration(10000); // Set acceleration to 1000 rpm/s
  besfoc_it_4->second.set_deceleration(10000); // Set deceleration to 1000 rpm/s
  besfoc_it_4->second.set_mode(besfoc::SPEED_MODE); // Set mode to velocity control

  besfoc_it_4->second.set_zero_position(); // Reset position to zero for joint 4

  RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Motors and encoders initialized successfully.");

  homing_sequence(besfoc_it_4->second); // Perform homing sequence for joint 4

  joint_position_.assign(6, 0);
  joint_velocities_.assign(6, 0);
  joint_velocities_command_.assign(6, 0);

  for (const auto & joint : info_.joints)
  {
    for (const auto & interface : joint.state_interfaces)
    {
      joint_interfaces[interface.name].push_back(joint.name);
    }
  }

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotSystem::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Stop all motors when deactivating
  for (auto & [joint_num, motor] : besfoc_motors)
  {
    motor.stop_motor();
  }

  for (auto & [joint_num, motor] : rmd_motors)
  {
    motor.stopMotor();
    motor.shutdownMotor();
  }

  return CallbackReturn::SUCCESS;
}

int RobotSystem::homing_sequence(besfoc::CanMotor& motor){
    const int POS_TOLERANCE = 100; // Define a position tolerance for homing
    const int MAX_HITS = 4; // Define the number of consecutive hits required to confirm homing

    int velocity;
    int torque = 80;

    int hits = 0;

    vector<int> positions = {};
    int current_position;
    int home_position;

    motor.set_zero_position();
    motor.set_tourque_slope(-1000); // Set velocity to 1000
    motor.set_tourque_speed_limit(1000); // Set velocity to 1000
    motor.set_tourque(-torque); // Set velocity to 1000

    RCLCPP_INFO(rclcpp::get_logger("test_node"), "Starting homing sequence. Torque windup in progress...");
    while(abs(velocity) < 30)
    {
        torque++;
        motor.set_tourque(-torque); // Set velocity to 1000
        
        rclcpp::sleep_for(std::chrono::milliseconds(10));
        motor.get_velocity(velocity);
        
    }

    RCLCPP_INFO(rclcpp::get_logger("test_node"), "Torque windup complete. Torque set to: %d", torque);
    RCLCPP_INFO(rclcpp::get_logger("test_node"), "Starting homing sequence.");
    for(int i = 0; i < MAX_HITS; i++)
    {
        go_to_stop(motor, torque);

        RCLCPP_INFO(rclcpp::get_logger("test_node"), "Position hit %d/%d. Checking if motor postition...", i + 1, MAX_HITS);
        rclcpp::sleep_for(std::chrono::milliseconds(100)); // Wait for 1 second

        motor.get_position(current_position);

        if(positions.size() == 0){
            positions.push_back(current_position);
            retract_motor(motor);
            continue;
        }

        int postion_total = accumulate(positions.begin(), positions.end(), 0);
        int size = positions.size();
        int avg_position = postion_total / size;
        int pos_diff = abs(abs(current_position) - abs(avg_position));
        RCLCPP_INFO(rclcpp::get_logger("test_node"), "Pos Diff: %d, size: %d, total: %d Current Position: %d, Average Position: %d", pos_diff, size, postion_total, current_position, avg_position);

    
        if(abs(abs(current_position) - abs(avg_position)) > POS_TOLERANCE)
        {
            RCLCPP_INFO(rclcpp::get_logger("test_node"), "Motor position not stable. Resetting hit count.");
            i = -1; // Reset hit count
            positions.clear(); // Clear recorded positions
        }

        positions.push_back(current_position);
        RCLCPP_INFO(rclcpp::get_logger("test_node"), "Motor position stable. Recorded position: %d", current_position);
        retract_motor(motor);
    }

    int postion_total = accumulate(positions.begin(), positions.end(), 0);
    int size = positions.size();
    home_position = postion_total / size;


    bool at_target = false;
    motor.set_position_absolute(home_position, 1000); // Move to home position

    while(at_target == false) {
        motor.at_postion_target(at_target);
        rclcpp::sleep_for(std::chrono::milliseconds(10));
    }
    RCLCPP_INFO(rclcpp::get_logger("test_node"), "Homing sequence completed. Home position established at: %d", home_position);

    at_target = false;
    motor.set_position_relative((BESFOC_4_CPR*BESFOC_4_GEAR_RATIO)/2, 1000); // Move to home position

    while(at_target == false) {
        motor.at_postion_target(at_target);
        rclcpp::sleep_for(std::chrono::milliseconds(10));
    }
    
    motor.set_zero_position(); // Reset position to zero for joint 4

    return 0;
}


std::vector<hardware_interface::StateInterface> RobotSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // TODO Link the state interfaces to the actual variables in the cpp CAN interface
  int ind = 0;
  for (const auto & joint_name : joint_interfaces["position"])
  {
    state_interfaces.emplace_back(joint_name, "position", &joint_position_[ind++]);
  }

  ind = 0;
  for (const auto & joint_name : joint_interfaces["velocity"])
  {
    state_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_[ind++]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RobotSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // TODO Link the state interfaces to the actual variables in the cpp CAN interface
  int ind = 0;
  for (const auto & joint_name : joint_interfaces["velocity"])
  {
    command_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_command_[ind++]);
  }

  return command_interfaces;
}

return_type RobotSystem::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // TODO Make the pyhton CAN to c++ CAN to use it here

  // // TODO(pac48) set sensor_states_ values from subscriber

  for (auto i = 0ul; i < joint_velocities_command_.size(); i++)
  {
   
    auto encoder_it = amt21_encoders.find(i);
    auto my_actuator_it = rmd_motors.find(i);
    auto besfoc_it = besfoc_motors.find(i);

    if(encoder_it != amt21_encoders.end()) {
     
      // Update joint_position_ from the AMT21 encoder state
      int raw_position = encoder_it->second.readPosition();
      double radian_position = (static_cast<double>(raw_position) / AMT21::MAX_VALUE) * 2 * M_PI; // Convert raw position to radians
      
      RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Read position from encoder for joint %zu: %d (raw), %f (radians)", i, raw_position, radian_position);
  
      joint_velocities_[i] = (radian_position - joint_position_[i]) / period.seconds(); // Calculate velocity based on change in position over time
      joint_position_[i] = radian_position; // Update position
      
    }else if(my_actuator_it != rmd_motors.end()) {
      // Update joint_position_ and joint_velocities_ from the MyActuator RMD motor state
      double position = (my_actuator_it->second.getMultiTurnAngle()) * M_PI / 180.0; // Convert degrees to radians
      double velocity = (position - joint_position_[i]) / period.seconds(); // Calculate velocity based on change in position over time

      RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Read position and velocity from MyActuator for joint %zu: %f (position), %f (velocity)", i, position, velocity);
      
      joint_position_[i] = position;
      joint_velocities_[i] = velocity;

    }else if(besfoc_it != besfoc_motors.end()) {
      // Update joint_position_ and joint_velocities_ from the BesFoc motor state
      double velocity;
      int rpm;
      double position;
      int raw_position;

      besfoc_it->second.get_velocity(rpm); // Get velocity in rpm
      velocity = static_cast<double>(rpm) * (2.0 * M_PI) / (60.0 * BESFOC_4_GEAR_RATIO); // Convert rpm to rad/s

      besfoc_it->second.get_position(raw_position); // Get position in raw units
      position = static_cast<double>(raw_position) * (2.0 * M_PI) / (BESFOC_4_CPR*BESFOC_4_GEAR_RATIO); // Convert raw position to radians    

      // Set postion from 0 to 2pi to -pi to pi
      if (position > M_PI) {
        position -= 2.0 * M_PI;
      }
      RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Read velocity from BesFoc for joint %zu: %f (velocity), estimated position: %f", i, velocity, position);
      
      joint_position_[i] = position;
      joint_velocities_[i] = velocity;

    }else {
      //If no hardware is associated with this joint
      RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Current velocity command for joint %zu: %f", i, joint_velocities_command_[i]);
      
      joint_velocities_[i] = joint_velocities_command_[i];
      joint_position_[i] += joint_velocities_command_[i] * period.seconds();
    }
  }

  return return_type::OK;
}

return_type RobotSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // TODO Make the pyhton CAN to c++ CAN to use it here
  for (auto i = 0ul; i < joint_velocities_command_.size(); i++)
  {
    auto besfoc_it = besfoc_motors.find(i);
    auto my_actuator_it = rmd_motors.find(i);
    if(besfoc_it != besfoc_motors.end()) {
      int gear_ratio = (i == 0) ? BESFOC_1_GEAR_RATIO : BESFOC_4_GEAR_RATIO; // Use the appropriate gear ratio for joint 1 or joint 4
      // Send velocity command to the BesFoc motor
      double velocity_command = joint_velocities_command_[i];
      int rpm_command = static_cast<int>(velocity_command * 60.0 * gear_ratio/ (2.0 * M_PI)); // Convert rad/s to rpm at the motor shaft

      RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Sending velocity commandf to motor for joint %zu: %d rpm", i, rpm_command);
      besfoc_it->second.set_velocity(static_cast<int>(rpm_command));
    }else if(my_actuator_it != rmd_motors.end()) {

      // Send velocity command to the MyActuator RMD motor
      if(i < 4){

        double velocity_command = (joint_velocities_command_[i]) * (180.0 / M_PI); // Convert rad/s to degrees/s

        RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Sending velocity command to MyActuator for joint %zu: %f rpm", i, velocity_command);
        my_actuator_it->second.sendVelocitySetpoint(velocity_command);

        continue;
      }

      // For joints 5 and 6, use differential velocity control
      double vel5_command = (joint_velocities_command_[4]) * (180.0 / M_PI); // Convert rad/s to degrees/s
      double vel6_command = (joint_velocities_command_[5]) * (180.0 / M_PI); // Convert rad/s to degrees/s  

      double motor1_velocity = (vel5_command + vel6_command) * MYACTUATOR_5_GEAR_RATIO; // Average velocity for motor 1
      double motor2_velocity = (vel5_command - vel6_command) * MYACTUATOR_6_GEAR_RATIO; // Differential velocity for motor 2

      // Check if the commanded velocities exceed the maximum limits for either motor
      if(abs(motor1_velocity) > MYACTUATOR_MAX_VELOCITY || abs(motor2_velocity) > MYACTUATOR_6_MAX_VELOCITY) {
        RCLCPP_WARN(rclcpp::get_logger("RobotSystem"), "Velocity command exceeds maximum limit for joint %zu. Command: %f, Max: %d, Scaling Down Speeds.", i, motor1_velocity, MYACTUATOR_MAX_VELOCITY);
        // Scale down the velocities proportionally
        double scale_factor = std::max(abs(motor1_velocity) / MYACTUATOR_MAX_VELOCITY, abs(motor2_velocity) / MYACTUATOR_6_MAX_VELOCITY);
        motor1_velocity /= scale_factor;
        motor2_velocity /= scale_factor;
      }

      auto my_actuator_it_6 = rmd_motors.find(5);
      
      if(my_actuator_it_6 != rmd_motors.end()) {
        my_actuator_it->second.sendVelocitySetpoint(motor1_velocity);
        my_actuator_it_6->second.sendVelocitySetpoint(motor2_velocity);
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("RobotSystem"), "MyActuator for joint 6 not found. Cannot send velocity command.");
      }

      i = 5; // Skip the next iteration since we already handled joint 6
    }
  }

  return return_type::OK;
}

}  // namespace ros2_control_demo_example_7

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  gungnir::RobotSystem, hardware_interface::SystemInterface)
