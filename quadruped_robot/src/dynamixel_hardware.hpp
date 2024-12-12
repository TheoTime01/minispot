#ifndef QUADRUPED_DYNAMIXEL_HARDWARE_HPP
#define QUADRUPED_DYNAMIXEL_HARDWARE_HPP

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include "visibility_control.h"

class QuadrupedDynamixelHardware : public hardware_interface::RobotHW {
public:
    QUADRUPED_HARDWARE_PUBLIC
    QuadrupedDynamixelHardware();

    QUADRUPED_HARDWARE_PUBLIC
    ~QuadrupedDynamixelHardware();

    QUADRUPED_HARDWARE_PUBLIC
    bool init(ros::NodeHandle &nh);

    QUADRUPED_HARDWARE_PUBLIC
    void read();

    QUADRUPED_HARDWARE_PUBLIC
    void write();

private:
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::PositionJointInterface position_joint_interface_;

    std::vector<std::string> joint_names_;
    std::vector<int> joint_ids_;
    std::vector<double> joint_positions_;
    std::vector<double> joint_velocities_;
    std::vector<double> joint_efforts_;
    std::vector<double> joint_commands_;

    dynamixel::PortHandler *port_handler_;
    dynamixel::PacketHandler *packet_handler_;
};

#endif // QUADRUPED_DYNAMIXEL_HARDWARE_HPP
