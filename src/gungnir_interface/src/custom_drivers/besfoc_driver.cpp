#include "besfoc_driver/besfoc_driver.hpp"

namespace
{
std::string format_sdo_index(const std::array<int, 2> & index)
{
    std::ostringstream stream;
    stream << "0x" << std::uppercase << std::hex << std::setfill('0')
           << std::setw(2) << (index[1] & 0xFF)
           << std::setw(2) << (index[0] & 0xFF);
    return stream.str();
}

std::string build_timeout_message(
    const char * operation,
    const std::array<int, 2> & index,
    int subindex,
    int timeout_ms,
    int tx_can_id,
    int rx_can_id)
{
    std::ostringstream stream;
    stream << "Timed out waiting for BesFoc " << operation
           << " response. Expected index=" << format_sdo_index(index)
           << " subindex=0x" << std::uppercase << std::hex << std::setfill('0')
           << std::setw(2) << (subindex & 0xFF)
           << std::dec << " timeout_ms=" << timeout_ms
           << " tx_can_id=0x" << std::uppercase << std::hex << tx_can_id
           << " rx_can_id=0x" << rx_can_id;
    return stream.str();
}
}  // namespace

const std::map<int, int> besfoc::data_length_dict = {
    {besfoc::WRITE_BYTES_1, 1},
    {besfoc::WRITE_BYTES_2, 2},
    {besfoc::WRITE_BYTES_3, 3},
    {besfoc::WRITE_BYTES_4, 4},
    {besfoc::READ_BYTES, 0},
    {besfoc::READ_RESPONSE_BYTES_1, 1},
    {besfoc::READ_RESPONSE_BYTES_2, 2},
    {besfoc::READ_RESPONSE_BYTES_3, 3},
    {besfoc::READ_RESPONSE_BYTES_4, 4}
};

besfoc::CanMotor::CanMotor(int can_id, std::shared_ptr<CanBus> bus, int acc, int dec)
    : tx_can_id_(can_id + 0x600), rx_can_id_(can_id + 0x580), bus_(bus)
{  
    reset_fault();
    initialize_motor(); // Initialize motors on object creation
    
    set_acceleration(acc);
    set_deceleration(dec);
}

bool besfoc::CanMotor::send_can_write_command(array<int, 2> index, int subindex, vector<int> data, int data_length)
{
    send_sdo_command(index, subindex, data, data_length);

    if(check_response(index, subindex, 1000)){
        return true;
    }
    
    throw runtime_error(
        build_timeout_message(
            "write",
            index,
            subindex,
            1000,
            tx_can_id_,
            rx_can_id_));

}

bool besfoc::CanMotor::send_can_read_command(array<int, 2> index, int subindex, vector<int> data_in, int32_t& data_out)
{
    send_sdo_command(index, subindex, data_in, READ_BYTES);

    if(check_response(index, subindex, data_out, 1000)){
        return true;
    }
    
    throw runtime_error(
        build_timeout_message(
            "read",
            index,
            subindex,
            1000,
            tx_can_id_,
            rx_can_id_));

}

void besfoc::CanMotor::send_sdo_command(array<int, 2> index, int subindex, vector<int> data, int data_length) {
    // Construct CAN frame based on command parameters
    CanFrame frame;
    frame.frame.can_id = this->tx_can_id_;
    frame.frame.can_dlc = 8;

    frame.data()[0] = data_length;
    frame.data()[1] = index[1];
    frame.data()[2] = index[0];
    frame.data()[3] = subindex;

    int data_length_bytes = data_length_dict.at(data_length);

    for(int i = data_length_bytes; i > 0; i--) {
        //cout << "Data byte " << i << ": " << data[i - 1] << endl;
        frame.data()[3 + (data_length_bytes - i + 1)] = data[i - 1];
    }

    // Send the CAN frame
    bus_->send(&frame);
}

bool besfoc::CanMotor::check_response(array<int, 2>& index, int& subindex, int timeout_ms) {
    auto start = chrono::steady_clock::now();
    while (chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count() < timeout_ms) {
        array<int, 2> response_index;
        int response_subindex;
        int32_t response_data;

        const int remaining_timeout_ms =
            timeout_ms - static_cast<int>(chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count());

        if (remaining_timeout_ms <= 0) {
            break;
        }

        if (!read_sdo_response(response_index, response_subindex, response_data, remaining_timeout_ms)) {
            break;
        }

        if (response_index == index && response_subindex == subindex) {
            return true;
        }
    }

    return false;
}

bool besfoc::CanMotor::check_response(array<int, 2>& index, int& subindex, int32_t& data_out, int timeout_ms) {
    auto start = chrono::steady_clock::now();
    while (chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count() < timeout_ms) {
        array<int, 2> response_index;
        int response_subindex;

        const int remaining_timeout_ms =
            timeout_ms - static_cast<int>(chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count());

        if (remaining_timeout_ms <= 0) {
            break;
        }

        if (!read_sdo_response(response_index, response_subindex, data_out, remaining_timeout_ms)) {
            break;
        }

        if (response_index == index && response_subindex == subindex) {
            return true;
        }
    }

    return false;

}

bool besfoc::CanMotor::read_sdo_response(array<int, 2>& index_out, 
                                          int& subindex_out, 
                                          int32_t& data_out, 
                                          int timeout_ms) {
    CanFrame frame;
    auto start = chrono::steady_clock::now();

    while (chrono::duration_cast<chrono::milliseconds>(
               chrono::steady_clock::now() - start).count() < timeout_ms) {

        if (bus_->read(&frame)) {
            if (frame.frame.can_id == this->rx_can_id_) {
        
                index_out[1] = frame.data()[1];
                index_out[0] = frame.data()[2];
                subindex_out  = frame.data()[3];

                uint8_t cmd = frame.data()[0];

                if(cmd == WRITE_RESPONSE){
                    data_out = frame.data()[4]; // Acknowledgement, no data
                }
                else if(cmd == READ_RESPONSE_BYTES_4){
                    data_out = frame.data()[4] | (frame.data()[5] << 8) | (frame.data()[6] << 16) | (frame.data()[7] << 24);
                }
                else if(cmd == READ_RESPONSE_BYTES_3){
                    data_out = frame.data()[4] | (frame.data()[5] << 8) | (frame.data()[6] << 16);
                }
                else if(cmd == READ_RESPONSE_BYTES_2){
                    data_out = frame.data()[4] | (frame.data()[5] << 8);
                }
                else if(cmd == READ_RESPONSE_BYTES_1){
                    data_out = frame.data()[4];
                }
                else {
                    continue; // Not a valid response command, keep waiting
                }

                // cout << "Response cmd: 0x" << hex << (int)cmd
                //      << " Index: 0x" << (int)index_out[1] 
                //      << (int)index_out[0]
                //      << " Sub: 0x" << subindex_out
                //      << " Data: 0x" << data_out << dec << endl;
                return true;
            }
        }
        this_thread::sleep_for(chrono::milliseconds(1));
    }
    return false;
}

void besfoc::CanMotor::initialize_motor() {
    send_can_write_command({0x60, 0x40}, 0x00, {0x00, 0x06}, WRITE_BYTES_2); // Motor Release
    send_can_write_command({0x60, 0x40}, 0x00, {0x00, 0x07}, WRITE_BYTES_2); // Motor Ready
    send_can_write_command({0x60, 0x40}, 0x00, {0x00, 0x0F}, WRITE_BYTES_2); // Motor Switch On
    initialized = true;
}

void besfoc::CanMotor::disable_motor() {

    if(!initialized) {
        return;
    }

    //Make sure the motor is in quick stop state.
    int state;
    get_status(state);
    if(state != STATE_QUICK_STOP_ACTIVE) {
        stop_motor();
    }

    send_can_write_command({0x60, 0x40}, 0x00, {0x00, 0x0F}, WRITE_BYTES_2); // Motor Release
    send_can_write_command({0x60, 0x40}, 0x00, {0x00, 0x07}, WRITE_BYTES_2); // Motor Ready
    initialized = false;
}

void besfoc::CanMotor::get_status(int& motor_status) {
    int32_t status;

    if(send_can_read_command({0x60, 0x41}, 0x00, {0x00, 0x00}, status)){

       // Extract relevant bits per CIA402 protocol
        uint16_t state_bits = status & 0xFF;  // Bits 0-7 determine the state
        
        // Decode CIA402 state machine (Table from manual)
        if ((state_bits & 0x4F) == 0x00) {           // 0XX 0000
            motor_status = 0; // Not Ready To Switch On
        } else if ((state_bits & 0x4F) == 0x40) {   // 1XX 0000
            motor_status = 1; // Switch On Disabled
        } else if ((state_bits & 0x6F) == 0x21) {   // 01X 0001
            motor_status = 2; // Ready To Switch On
        } else if ((state_bits & 0x6F) == 0x23) {   // 01X 0011
            motor_status = 3; // Switched On
        } else if ((state_bits & 0x6F) == 0x27) {   // 01X 0111
            motor_status = 4; // Operation Enabled
        } else if ((state_bits & 0x4F) == 0x07) {   // 00X 0111
            motor_status = 5; // Quick Stop Active
        } else if ((state_bits & 0x4F) == 0x0F) {   // 0XX 1111
            motor_status = 6; // Fault Reaction Active
        } else if ((state_bits & 0x4F) == 0x08) {   // 0XX 1000
            motor_status = 7; // Fault
        } else {
            motor_status = -1; // Unknown state
        }
    }
}


void besfoc::CanMotor::get_status(int& motor_status, int32_t& status_word) {
    int32_t status;

    if(send_can_read_command({0x60, 0x41}, 0x00, {0x00, 0x00}, status)){

       // Extract relevant bits per CIA402 protocol
        uint16_t state_bits = status & 0xFF;  // Bits 0-7 determine the state
        
        // Decode CIA402 state machine (Table from manual)
        if ((state_bits & 0x4F) == 0x00) {           // 0XX 0000
            motor_status = 0; // Not Ready To Switch On
        } else if ((state_bits & 0x4F) == 0x40) {   // 1XX 0000
            motor_status = 1; // Switch On Disabled
        } else if ((state_bits & 0x6F) == 0x21) {   // 01X 0001
            motor_status = 2; // Ready To Switch On
        } else if ((state_bits & 0x6F) == 0x23) {   // 01X 0011
            motor_status = 3; // Switched On
        } else if ((state_bits & 0x6F) == 0x27) {   // 01X 0111
            motor_status = 4; // Operation Enabled
        } else if ((state_bits & 0x4F) == 0x07) {   // 00X 0111
            motor_status = 5; // Quick Stop Active
        } else if ((state_bits & 0x4F) == 0x0F) {   // 0XX 1111
            motor_status = 6; // Fault Reaction Active
        } else if ((state_bits & 0x4F) == 0x08) {   // 0XX 1000
            motor_status = 7; // Fault
        } else {
            motor_status = -1; // Unknown state
        }

        status_word = status; // Return the full status word        
    }
}



void besfoc::CanMotor::get_mode(int &mode) {

    if(initialized){
        initialize_motor();
    }

    send_can_read_command({0x60, 0x61}, 0x00, {0x00, 0x00}, mode);
}

void besfoc::CanMotor::set_mode(int mode) {
    if(!initialized){
        initialize_motor();
    }

    int current_mode;
    get_mode(current_mode);

    if(current_mode == besfoc::TORQUE_MODE) {
        reinitialize_motor(); // Disable torque mode before switching to speed mode
    }

    send_can_write_command({0x60, 0x60}, 0x00, {static_cast<int>(mode)}, WRITE_BYTES_1);
}

void besfoc::CanMotor::get_velocity(int& velocity) {
    if(!initialized){
        initialize_motor();
    }

    send_can_read_command({0x60, 0x6C}, 0x00, {0x00, 0x00}, velocity);
}

void besfoc::CanMotor::set_tourque_slope(int16_t slope) {
    if(!initialized){
        initialize_motor();
    }

    vector<int> data;
    to_bytes(slope, data);
    send_can_write_command({0x60, 0x87}, 0x00, data, WRITE_BYTES_2);
}

void besfoc::CanMotor::set_tourque_speed_limit(int limit) {
    if(!initialized){
        initialize_motor();
    }

    vector<int> data;
    to_bytes(limit, data);
    send_can_write_command({0x60, 0xFF}, 0x00, data, WRITE_BYTES_4);
}

void besfoc::CanMotor::set_tourque(int16_t torque) {
    if(!initialized){
        initialize_motor();
    }

    int mode;
    get_mode(mode);
    if(mode != besfoc::TORQUE_MODE) {
        set_mode(besfoc::TORQUE_MODE);
    }

    vector<int> data;
    to_bytes(torque, data);
    send_can_write_command({0x60, 0x71}, 0x00, data, WRITE_BYTES_2);
}

void besfoc::CanMotor::at_postion_target(bool &at_target) {
    if(!initialized){
        initialize_motor();
    }

    int status;
    int32_t status_word;
    get_status(status, status_word);

    // Check bit 10 of the status word to determine if the motor is at the target position
    at_target = (status_word & (1 << 10)) != 0;    
}


void besfoc::CanMotor::set_velocity(int velocity) {
    if(!initialized){
        initialize_motor();
    }
    
    int mode;
    get_mode(mode);
    if(mode != besfoc::SPEED_MODE) {
        set_mode(besfoc::SPEED_MODE);
    }

    if(abs(velocity) > 6000) {
        velocity = (velocity > 0) ? 6000 : -6000; // Limit velocity to valid range
    }

    vector<int> data;
    to_bytes(velocity, data);
    send_can_write_command({0x60, 0xFF}, 0x00, data, WRITE_BYTES_4);
}

void besfoc::CanMotor::get_position(int &position) {
    if(!initialized){
        initialize_motor();
    }

    send_can_read_command({0x60, 0x64}, 0x00, {0x00, 0x00}, position);
}

void besfoc::CanMotor::set_position_relative(int position, int velocity) {
    vector<int> posData;
    vector<int> velData;

    if(!initialized){
        initialize_motor();
    }

    int mode;
    get_mode(mode);
    if(mode != besfoc::POSITION_MODE) {
        set_mode(besfoc::POSITION_MODE);
    }

    to_bytes(position, posData);

    if(abs(position) > 100000000) {
        return; // Invalid position value, do not send command
    }

    if (velocity > 0 && velocity <= 6000) {
        to_bytes(velocity, velData);
    } else {
        to_bytes(100, velData); // Default velocity if invalid value provided
    }

    send_can_write_command({0x60, 0x81}, 0x00, velData, WRITE_BYTES_4); // Set speed
    send_can_write_command({0x60, 0x7A}, 0x00, posData, WRITE_BYTES_4); // Set position

    //Start Relative Positioning Movement
    send_can_write_command({0x60, 0x40}, 0x00, {0x00, 0x0F}, WRITE_BYTES_2);
    send_can_write_command({0x60, 0x40}, 0x00, {0x00, 0x4F}, WRITE_BYTES_2); 
    send_can_write_command({0x60, 0x40}, 0x00, {0x00, 0x5F}, WRITE_BYTES_2); 
}

void besfoc::CanMotor::get_tourque(int &torque) {
    if(!initialized){
        initialize_motor();
    }

    send_can_read_command({0x60, 0x77}, 0x00, {0x00, 0x00}, torque);

    if(torque > 32767) {
        torque -= 65536; // Convert to signed 16-bit integer
    }
}

void besfoc::CanMotor::set_zero_position(){
    send_can_write_command({0x21, 0x01}, 0x00, {0x00,0x01}, WRITE_BYTES_2);
}

void besfoc::CanMotor::reset_fault(){
    send_can_write_command({0x60, 0x40}, 0x00, {0x00, 0x80}, WRITE_BYTES_2);
}

void besfoc::CanMotor::set_position_absolute(int position, int velocity) {
    vector<int> posData;
    vector<int> velData;

    if(!initialized){
        initialize_motor();
    }

    int mode;
    if(mode != besfoc::POSITION_MODE) {

        if(mode == besfoc::TORQUE_MODE) {
            reinitialize_motor(); // Disable torque mode before switching to speed mode
        }

        set_mode(besfoc::POSITION_MODE);
    }

    to_bytes(position, posData);

    if(abs(position) > 100000000) {
        return; // Invalid position value, do not send command
    }

    if (velocity > 0 && velocity <= 6000) {
        to_bytes(velocity, velData);
    } else {
        to_bytes(100, velData); // Default velocity if invalid value provided
    }

    send_can_write_command({0x60, 0x81}, 0x00, velData, WRITE_BYTES_4); // Set speed
    send_can_write_command({0x60, 0x7A}, 0x00, posData, WRITE_BYTES_4); // Set position

    //Start Absolute Positioning Movement
    send_can_write_command({0x60, 0x40}, 0x00, {0x00, 0x0F}, WRITE_BYTES_2);
    send_can_write_command({0x60, 0x40}, 0x00, {0x00, 0x1F}, WRITE_BYTES_2); 
}

void besfoc::CanMotor::set_acceleration(int acceleration) {
    acc = acceleration;

    vector<int> data;
    to_bytes(acceleration, data);
    send_can_write_command({0x60, 0x83}, 0x00, data, WRITE_BYTES_4);
}

void besfoc::CanMotor::set_deceleration(int deceleration) {
    dec = deceleration;

    vector<int> data;
    to_bytes(deceleration, data);
    send_can_write_command({0x60, 0x84}, 0x00, data, WRITE_BYTES_4);
}

void besfoc::CanMotor::pause_motor() {
    if(!initialized){
        initialize_motor();
    }

    send_can_write_command({0x60, 0x40}, 0x00, {0x01, 0x0F}, WRITE_BYTES_2); // Start deceleration to stop

    initialized=false;
}

void besfoc::CanMotor::stop_motor() {
    if(!initialized){
        initialize_motor();
    }

    send_can_write_command({0x60, 0x40}, 0x00, {0x00, 0x02}, WRITE_BYTES_2); // Start deceleration to stop

    initialized=false;
}

void besfoc::CanMotor::reinitialize_motor() {

    disable_motor(); // Disable the motor
    std::this_thread::sleep_for(std::chrono::milliseconds(200)); // Wait for 100 milliseconds
    initialize_motor(); // Re-enable the motor

    set_acceleration(acc); // Restore acceleration
    set_deceleration(dec); // Restore deceleration
}

void besfoc::CanMotor::to_bytes(int32_t value, vector<int>& bytes) {
    for(int i = 0; i < 4; i++) {
        bytes.push_back((value >> (8 * i)) & 0xFF);
    }
    reverse(bytes.begin(), bytes.end()); // Reverse to get big-endian format
}

void besfoc::CanMotor::to_bytes(int16_t value, vector<int>& bytes) {
    for(int i = 0; i < 2; i++) {
        bytes.push_back((value >> (8 * i)) & 0xFF);
    }
    reverse(bytes.begin(), bytes.end()); // Reverse to get big-endian format
}

void besfoc::CanMotor::to_bytes(int8_t value, vector<int>& bytes) {
    bytes.push_back(value & 0xFF);
}


besfoc::CanMotor::~CanMotor() {
}