/****************************************************************************
 * BSD 3-Clause License
 *
 * Copyright 2021-2022 NXP
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/*
 * The TrajectorySetpoint message and the OFFBOARD mode in general are under an ongoing update.
 * Please refer to PR: https://github.com/PX4/PX4-Autopilot/pull/16739 for more info.
 * As per PR: https://github.com/PX4/PX4-Autopilot/pull/17094, the format
 * of the TrajectorySetpoint message shall change.
*/

#include <stdint.h>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <iostream>
#include <memory>

#include "std_srvs/srv/empty.hpp"


/*
 * These definitions can be found here: https://mavlink.io/en/messages/common.html
 */
#define MAV_COMP_ID_AUTOPILOT1 1
#define MAV_COMP_ID_ONBOARD_COMPUTER 191

/*
 *The default mavlink systemid is 1 (https://mavlink.io/en/guide/routing.html#overview)
 */
#define MAV_SYS_ID 1

using std::isnan;
using std::chrono::milliseconds;
using namespace std::chrono_literals;
using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::Timesync;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleControlMode;
using std_srvs::srv::Empty;

class Controller : public rclcpp::Node
{
public:
    Controller() : Node("controller")
    {
        // Mavlink id of the PX4 instance
        this->declare_parameter("system_id", MAV_SYS_ID);

        /*
            Custom mavlink system id can be provided as a parameter
            In simulation and real-life tests, this id should match
            MAVLink system ID of the vehicle.
        */

        this->get_parameter("system_id", system_id);

        offboard_control_mode_publisher_ =
            this->create_publisher<OffboardControlMode>("fmu/offboard_control_mode/in", 10);
        trajectory_setpoint_publisher_ =
            this->create_publisher<TrajectorySetpoint>("fmu/trajectory_setpoint/in", 10);
        vehicle_command_publisher_ =
            this->create_publisher<VehicleCommand>("fmu/vehicle_command/in", 10);

        // get common timestamp
        timesync_sub_ =
            this->create_subscription<Timesync>("fmu/timesync/out", 10,
                                                [this](const Timesync::UniquePtr msg)
                                                {
                                                    timestamp_.store(msg->timestamp);
                                                });

        goal_trajectory_sub_ =
            this->create_subscription<TrajectorySetpoint>("GoalTrajectory_PubSubTopic", 1,
                                                [this](const TrajectorySetpoint::UniquePtr msg)
                                                {
                                                    // copy received msg to goal_point_
                                                    goal_point_ = *msg;
                                                });

        land_service_ =
            this->create_service<Empty>("land_service",
                                        std::bind(&Controller::land_vehicle,
                                        this, std::placeholders::_1, std::placeholders::_2)
                                        );

        auto timer_callback = [this]() -> void
        {
            // offboard_control_mode needs to be paired with trajectory_setpoint
            if (!land_triggered_)
            {
                publish_offboard_control_mode();
                publish_trajectory_setpoint();
            }
        };
        timer_ = this->create_wall_timer(100ms, timer_callback);
    }

    void arm() const;
    void disarm() const;
    void land() const;
    void land_vehicle(const std::shared_ptr<Empty::Request>, std::shared_ptr<Empty::Response>);

private:
    /* This point shall be updated by in a subscriber */
    int system_id;
    TrajectorySetpoint goal_point_{};
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Subscription<Timesync>::SharedPtr timesync_sub_;

    rclcpp::Service<Empty>::SharedPtr land_service_;
    /* This subscriber receives the goal trajectory setpoint from external node */
    rclcpp::Subscription<TrajectorySetpoint>::SharedPtr goal_trajectory_sub_;

    std::atomic<uint64_t> timestamp_;    // common synced timestamped
    bool land_triggered_ = false;

    void publish_offboard_control_mode() const;
    void publish_trajectory_setpoint();
    void publish_vehicle_command(uint16_t command, float param1 = 0.0,
                                 float param2 = 0.0, float param3 = 0.0) const;
};

/**
 * @brief Send a command to Arm the vehicle
 */
void Controller::arm() const
{
    this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

    RCLCPP_INFO(this->get_logger(), "Arm command sent");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void Controller::disarm() const
{
    this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

    RCLCPP_INFO(this->get_logger(), "Disarm command sent");
}

/**
 * @brief Sens the LAND command to land the vehicle in the current position
 */
void Controller::land() const
{
    this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);

    RCLCPP_INFO_ONCE(this->get_logger(), "Land command sent");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only velocity controls are active.
 */
void Controller::publish_offboard_control_mode() const
{
    OffboardControlMode msg{};
    msg.timestamp = timestamp_.load();
    msg.position = false;
    msg.velocity = true;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;

    offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
    It publishes the goal_point_ as a TrajectorySetpoint message
 */
void Controller::publish_trajectory_setpoint()
{
    goal_point_.timestamp = timestamp_.load();
    trajectory_setpoint_publisher_->publish(goal_point_);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Parameter 1, as defined by MAVLink uint16 VEHICLE_CMD enum.
 * @param param2    Parameter 2, as defined by MAVLink uint16 VEHICLE_CMD enum.
 * @param param3    Parameter 3, as defined by MAVLink uint16 VEHICLE_CMD enum.
 */
void Controller::publish_vehicle_command(uint16_t command, float param1,
                                         float param2, float param3) const
{
    VehicleCommand msg{};
    msg.timestamp = timestamp_.load();
    msg.param1 = param1;
    msg.param2 = param2;
    msg.param3 = param3;
    msg.command = command;                                // Command ID
    msg.target_system = this->system_id;                  // ID of the system executing the command
    msg.target_component = MAV_COMP_ID_AUTOPILOT1;        // ID of component executing the command
    msg.source_system = this->system_id;                  // ID of the system sending the command
    msg.source_component = MAV_COMP_ID_ONBOARD_COMPUTER;  // ID of component sending the command
    msg.from_external = true;

    RCLCPP_INFO(this->get_logger(), "Publishing command: %d for system id: %d",
                                     command, this->system_id);
    vehicle_command_publisher_->publish(msg);
}

void Controller::land_vehicle(const std::shared_ptr<Empty::Request>,
                                std::shared_ptr<Empty::Response>)
{
    this->land();
    // response->success = true;
    land_triggered_ = true;
}

int main(int argc, char *argv[])
{
    std::cout << "Starting the controller node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Controller>());

    rclcpp::shutdown();
    return 0;
}
