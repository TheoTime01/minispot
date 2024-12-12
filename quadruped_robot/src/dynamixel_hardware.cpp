#include "dynamixel_hardware.hpp"

QuadrupedDynamixelHardware::QuadrupedDynamixelHardware() {
    joint_positions_.resize(12, 0.0);  // 4 legs x 3 joints per leg
    joint_velocities_.resize(12, 0.0);
    joint_efforts_.resize(12, 0.0);
    joint_commands_.resize(12, 0.0);
}

QuadrupedDynamixelHardware::~QuadrupedDynamixelHardware() {
    if (port_handler_) port_handler_->closePort();
}

bool QuadrupedDynamixelHardware::init(ros::NodeHandle &nh) {
    // Initialize port handler and packet handler
    port_handler_ = dynamixel::PortHandler::getPortHandler("/dev/ttyUSB0");
    packet_handler_ = dynamixel::PacketHandler::getPacketHandler(2.0);

    if (!port_handler_->openPort() || !port_handler_->setBaudRate(57600)) {
        ROS_ERROR("Failed to initialize port");
        return false;
    }

    nh.getParam("joint_names", joint_names_);
    nh.getParam("joint_ids", joint_ids_);

    for (size_t i = 0; i < joint_names_.size(); ++i) {
        hardware_interface::JointStateHandle state_handle(joint_names_[i],
            &joint_positions_[i], &joint_velocities_[i], &joint_efforts_[i]);
        joint_state_interface_.registerHandle(state_handle);

        hardware_interface::JointHandle position_handle(joint_state_interface_.getHandle(joint_names_[i]), 
            &joint_commands_[i]);
        position_joint_interface_.registerHandle(position_handle);
    }

    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);

    return true;
}

void QuadrupedDynamixelHardware::read() {
    // Read joint positions from Dynamixel servos
    for (size_t i = 0; i < joint_ids_.size(); ++i) {
        uint16_t position;
        int result = packet_handler_->read2ByteTxRx(port_handler_, joint_ids_[i], 36, &position);
        if (result == COMM_SUCCESS) {
            joint_positions_[i] = (double)position * (M_PI / 2048.0) - M_PI;
        } else {
            ROS_ERROR("Read error for joint ID %d", joint_ids_[i]);
        }
    }
}

void QuadrupedDynamixelHardware::write() {
    // Write joint commands to Dynamixel servos
    for (size_t i = 0; i < joint_ids_.size(); ++i) {
        uint16_t position = (uint16_t)((joint_commands_[i] + M_PI) * (2048.0 / M_PI));
        int result = packet_handler_->write2ByteTxRx(port_handler_, joint_ids_[i], 30, position);
        if (result != COMM_SUCCESS) {
            ROS_ERROR("Write error for joint ID %d", joint_ids_[i]);
        }
    }
}
