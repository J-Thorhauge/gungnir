#ifndef BESFOC_DRIVER_HPP
#define BESFOC_DRIVER_HPP

#include "linux_canbus_cpp/Bus/CanBus.hpp"
#include <iostream>
#include <vector>
#include <map>
#include <memory>
#include <algorithm>
#include <thread>
#include <chrono>
#include <bits/stdc++.h>

using namespace std;

namespace besfoc
{
    const int RUNNING = 1;
    const int ERROR = 2;
    const int UNINITIALIZED = 4;
    const int IDLE = 8;

    const int POSITION_MODE = 1;
    const int SPEED_MODE = 3;
    const int TORQUE_MODE = 4;

    const map<int, string> mode_dict = {
        {POSITION_MODE, "Position Mode"},
        {SPEED_MODE, "Speed Mode"},
        {TORQUE_MODE, "Torque Mode"}
    };

    const int WRITE_BYTES_1 = 0x2F;
    const int WRITE_BYTES_2 = 0x2B;
    const int WRITE_BYTES_3 = 0x27;
    const int WRITE_BYTES_4 = 0x23;

    const int WRITE_RESPONSE = 0x60;

    const int READ_BYTES = 0x40;
    const int READ_RESPONSE_BYTES_1 = 0x4f;
    const int READ_RESPONSE_BYTES_2 = 0x4b;
    const int READ_RESPONSE_BYTES_3 = 0x47;
    const int READ_RESPONSE_BYTES_4 = 0x43;

    extern const map<int, int> data_length_dict;

    const int STATE_NOT_READY_TO_SWITCH_ON = 0;
    const int STATE_SWITCH_ON_DISABLED = 1;
    const int STATE_READY_TO_SWITCH_ON = 2;
    const int STATE_SWITCHED_ON = 3;
    const int STATE_OPERATION_ENABLED = 4;
    const int STATE_QUICK_STOP_ACTIVE = 5;
    const int STATE_FAULT_REACTION_ACTIVE = 6;
    const int STATE_FAULT = 7;
    const int STATE_UNKNOWN = -1;

    const map<int, string> state_dict = {
        {STATE_NOT_READY_TO_SWITCH_ON, "Not Ready to Switch On"},
        {STATE_SWITCH_ON_DISABLED, "Switch On Disabled"},
        {STATE_READY_TO_SWITCH_ON, "Ready to Switch On"},
        {STATE_SWITCHED_ON, "Switched On"},
        {STATE_OPERATION_ENABLED, "Operation Enabled"},
        {STATE_QUICK_STOP_ACTIVE, "Quick Stop Active"},
        {STATE_FAULT_REACTION_ACTIVE, "Fault Reaction Active"},
        {STATE_FAULT, "Fault"},
        {STATE_UNKNOWN, "Unknown"}
    };

    class CanMotor
    {
        public:
            CanMotor(int can_id, std::shared_ptr<CanBus> bus, int acc = 100, int dec = 100);
            ~CanMotor();

            void set_mode(int mode);
            
            void set_velocity(int velocity);
            void get_velocity(int& velocity);

            void set_zero_position();

            void set_position_relative(int position, int velocity = 100);
            void set_position_absolute(int position, int velocity = 100);

            void get_position(int &position); 

            void set_tourque_slope(int16_t slope);
            void set_tourque_speed_limit(int limit);
            void set_tourque(int16_t torque);
            
            void get_tourque(int &torque);

            void set_acceleration(int acceleration);
            void set_deceleration(int deceleration);

            void at_postion_target(bool &at_target);

            void initialize_motor();
            void get_mode(int &mode);
            void pause_motor();
            void stop_motor();
            void reset_fault();

            void get_status(int& motor_status);
            void get_status(int& motor_status, int32_t& status_word);

            void disable_motor();
            void reinitialize_motor();
            void shutdown();//Not made yet

        private:
            void to_bytes(int8_t value, vector<int>& bytes);
            void to_bytes(int16_t value, vector<int>& bytes);
            void to_bytes(int32_t value, vector<int>& bytes);

            bool send_can_write_command(array<int, 2> index, int subindex, vector<int> data, int data_length);
            bool send_can_read_command(array<int, 2> index, int subindex, vector<int> data_in, int32_t& data_out);

            void send_sdo_command(array<int, 2> index, int subindex, vector<int> data, int data_length);
            bool read_sdo_response(array<int, 2>& index_out, int& subindex_out, int32_t& data_out, int timeout_ms = 1000);
            
            bool check_response(array<int, 2>& index, int& subindex, int timeout_ms=1000);
            bool check_response(array<int, 2>& index, int& subindex, int32_t& data_out, int timeout_ms=1000);
            
            

            int tx_can_id_;
            int rx_can_id_;

            int acc;
            int dec;

            bool initialized;
            std::shared_ptr<CanBus> bus_;
            

    };
}


#endif