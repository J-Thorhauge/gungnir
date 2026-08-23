#include "besfoc_driver/besfoc_driver.hpp"
#include "amt21_encoder_driver/amt21_encoder_driver.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>

void test_relative_postion(besfoc::CanMotor motor){
    motor.set_mode(besfoc::POSITION_MODE); // Set mode to position control
    
    motor.set_position_relative(51200, 1000); // Move to position 100

    bool at_target = false;
    motor.at_postion_target(at_target);
    RCLCPP_INFO(rclcpp::get_logger("test_node"), "Motor at target position: %s", at_target ? "true" : "false");    
    

    rclcpp::sleep_for(std::chrono::milliseconds(7000)); // Wait for 1 second

    motor.at_postion_target(at_target);
    RCLCPP_INFO(rclcpp::get_logger("test_node"), "Motor at target position: %s", at_target ? "true" : "false");  

}

void test_absolute_postion(besfoc::CanMotor motor){
    motor.set_zero_position();
    
    
    motor.set_position_absolute(20000, 1000); // Move to position 100

    int position;
    int velocity;
    int torque;
    bool at_target = false;
    while(!at_target){
        motor.at_postion_target(at_target);
        motor.get_position(position);
        motor.get_velocity(velocity);
        motor.get_tourque(torque);
        RCLCPP_INFO(rclcpp::get_logger("test_node"), "Motor at target position: %s, Current Position: %d, Velocity: %d, Torque: %d", at_target ? "true" : "false", position, velocity, torque);      
        rclcpp::sleep_for(std::chrono::milliseconds(100)); //
    }
    
    motor.set_position_absolute(0, 1000); // Move back to position 0
    at_target = false;
    while(!at_target){
        motor.at_postion_target(at_target);
        motor.get_position(position);
        motor.get_velocity(velocity);
        motor.get_tourque(torque);
        RCLCPP_INFO(rclcpp::get_logger("test_node"), "Motor at target position: %s, Current Position: %d, Velocity: %d, Torque: %d", at_target ? "true" : "false", position, velocity, torque);      
        rclcpp::sleep_for(std::chrono::milliseconds(100)); //
    }

    motor.set_position_absolute(-20000, 1000); // Move to positio5, 10n -100
    RCLCPP_INFO(rclcpp::get_logger("test_node"), "Motor moving to position -20000");
    rclcpp::sleep_for(std::chrono::milliseconds(1000)); // Wait for 1 second
    RCLCPP_INFO(rclcpp::get_logger("test_node"), "Motor moving with speed 1000");
    motor.set_velocity(1000); // Stop the motor
    RCLCPP_INFO(rclcpp::get_logger("test_node"), "Motor moving with speed 1000");

    rclcpp::sleep_for(std::chrono::milliseconds(1000)); // Wait for 1 second
    motor.stop_motor(); // Stop the motor

    
}

void test_force(besfoc::CanMotor& motor){
    int motor_status;
    int velocity;

    motor.set_mode(besfoc::TORQUE_MODE); // Set mode to velocity control

    motor.set_tourque_slope(1000); // Set velocity to 1000
    motor.set_tourque_speed_limit(1000); // Set velocity to 1000
    motor.set_tourque(0); // Set velocity to 1000

    //Delay
    int time = 4*100; // 5 seconds at 100ms sleep intervals
    int torque = 0;

    while(velocity < 30){

        int force;
        motor.get_tourque(force);
        
        motor.get_velocity(velocity);
        RCLCPP_INFO(rclcpp::get_logger("test_node"), "Motor Torque:%d;  Motor Velocity:%d", force, velocity);

        torque++;
        motor.set_tourque(torque); // Set velocity to 1000
        rclcpp::sleep_for(std::chrono::milliseconds(100));  
    }
    
    RCLCPP_INFO(rclcpp::get_logger("test_node"), "Motor Torque:%d;  Motor Velocity:%d", torque, velocity);
    
}

void test_velocity(besfoc::CanMotor& motor){
    int motor_status;
    int velocity;

    motor.set_acceleration(10000); // Set acceleration to 1000 rpm/s
    motor.set_deceleration(10000); // Set deceleration to 1000 rpm
    motor.set_mode(besfoc::SPEED_MODE); // Set mode to velocity control
   
    motor.set_zero_position();

    motor.set_velocity(1000); // Set velocity to 1000

    //Delay
    int time = 0.5*100; // 5 seconds at 100ms sleep intervals
    while(time > 0){
        
        // int encoder_position = encoder.readPosition(); // Read position from encoder at address 0x54
        // RCLCPP_INFO(rclcpp::get_logger("test_node"), "Encoder Position: %d", encoder_position);
        int force;
        motor.get_position(force);
        RCLCPP_INFO(rclcpp::get_logger("test_node"), "Motor Torque:%d;  Motor Velocity:%d", force, velocity);
        
        rclcpp::sleep_for(std::chrono::milliseconds(1));  
        time--;
    }
    motor.stop_motor();
    motor.get_status(motor_status);
    string state_str = besfoc::state_dict.at(motor_status);
    RCLCPP_INFO(rclcpp::get_logger("test_node"), "Motor Status: %s", state_str.c_str());
}
int test_encoder (AMT21& encoder, string name = "Encoder"){

        int pos1 = encoder.readPosition(true); // encoder 1 default address

        RCLCPP_INFO(rclcpp::get_logger("test_node"), "%s Position: %d", name.c_str(), pos1);

        rclcpp::sleep_for(std::chrono::milliseconds(10));

        return 0;

}
void retract_motor(besfoc::CanMotor& motor){
    motor.set_acceleration(10000); // Set acceleration to 1000 rpm/s
    motor.set_deceleration(10000); // Set deceleration to 1000 rpm

    RCLCPP_INFO(rclcpp::get_logger("test_node"), "Retracting motor...");

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

int homing_secuence_test (besfoc::CanMotor& motor){
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
    motor.set_position_relative(50000/2, 1000); // Move to home position

    while(at_target == false) {
        motor.at_postion_target(at_target);
        rclcpp::sleep_for(std::chrono::milliseconds(10));
    }



    return 0;
}

int main()
{
    // Example usage of CanMotor class

    const int ENCODER_ADDRESS_1 = 0x58; // Address for encoder 1
    const int ENCODER_ADDRESS_2 = 0x5C; // Address for encoder 2

    AMT21 encoder1(ENCODER_ADDRESS_1 ,"/dev/ttyUSB0", 115200, true);
    AMT21 encoder2(ENCODER_ADDRESS_2 ,"/dev/ttyUSB0", 115200, false);

    encoder1.setZero(); // Reset encoder 1

    // auto bus = std::make_shared<CanBus>("can0");
    // bus->connect();

    // besfoc::CanMotor motor4(0x65, bus);
    // RCLCPP_INFO(rclcpp::get_logger("test_node"), "CanBus connected successfully.");

    //test_relative_postion(motor4);

    //test_absolute_postion(motor4);
    //test_force(motor4);
    //test_velocity(motor4);
    //test_encoder(encoder1, "Encoder 1");
    while(true)
    {
        test_encoder(encoder1, "Encoder 1");
        test_encoder(encoder2, "Encoder 2");
    }

    // rclcpp::sleep_for(std::chrono::milliseconds(2000)); // Wait for 1 second

    //homing_secuence_test(motor4);

    //bus->disconnect();

    RCLCPP_INFO(rclcpp::get_logger("test_node"), "CanBus disconnected successfully.");
    
    return 0;
}

