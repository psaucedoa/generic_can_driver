/*
 * Copyright 2024 Construction Engineering Research Laboratory (CERL)
 * Engineer Reseach and Development Center (ERDC)
 * U.S. Army Corps of Engineers
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef GENERIC_CAN_DRIVER__GENERIC_CAN_DRIVER_HPP_
#define GENERIC_CAN_DRIVER__GENERIC_CAN_DRIVER_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "can_msgs/msg/frame.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "can_dbc_parser/Dbc.hpp"
#include "can_dbc_parser/DbcBuilder.hpp"
#include "can_dbc_parser/DbcMessage.hpp"
#include "can_dbc_parser/DbcSignal.hpp"

using LifecycleNodeInterface = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
namespace rlc = rclcpp_lifecycle;

namespace ros2_j1939
{

class GenericCanDriver : public rlc::LifecycleNode
{
public:
  explicit GenericCanDriver(const rclcpp::NodeOptions & OPTIONS);

  ~GenericCanDriver();

  // DATABASE MANAGEMENT FUNCTIONS //
  /**
   * @brief Instantiates a NewEagle dbc database object using the dbc file specified in params.
   * Strips the priority and source address id from the message ID in the DBC.
   * These stripped IDs are stored as keys in a map with the NewEagle DBC messages as values.
   */
  void setupDatabase();

  /**
   * @brief Checks the messages in the DBC and creates a publisher for each one
   * 
   * The publishers are of type "j1939_interfaces::msg::CanData" with a topic name folling a
   * "sensor_name/key_message" pattern
   */
  void configurePublishers();

  /**
   * Goes through the configured publishers and activates them
   */
  void activatePublishers();

  /**
   * Goes through the configured publishers and activates them
   */
  void deactivatePublishers();

  /**
   * @brief functions that takes list of full addresses (such as 0x0CEEFFA1) as defined in params
   * yaml and attempts and address claim attack. It adopts the lowest value name and publishes it on
   * that address, forcing the target device on that address to either stop publishing or move to a
   * different address, depending on its internal logic.
  */
  void generateAddressClaimAttackMsg(
    can_msgs::msg::Frame::SharedPtr MSG, const std::vector<uint32_t> source_addresses
  );

  void createDataArray(
    const std::vector<uint16_t> data_in, 
    const std::vector<uint16_t> data_lengths, 
    std::array<uint8_t, 8UL> &data_out
  );


  // /** 
  //  * @brief Generic function that publishes ros2 CAN frames
  //  *
  // */
  // void txFrame(const can_msgs::msg::Frame MSG);

  // /**
  //  * @brief changes the source address of the device given its name and desired source address
  // */
  // void txRename(const std::array<uint8_t, 8UL> name, const uint8_t new_source_address);

  // params
  std::string dbw_dbc_file_;  // set in launch file. Files such as MV5.dbc
  std::string frame_id_;      // such as: base, etc.
  std::string sensor_name_;   // such as: /Imu/microstrain/joint_1_L, or w/e
  uint8_t device_ID_;         // such as 226
  bool address_claim_attack_;
  std::vector<long int> addressess_to_claim_attack_;
  NewEagle::Dbc dbw_dbc_db_;   // new eagle dbc database

  bool set_new_source_address_;
  uint8_t new_source_address_;
  std::array<uint8_t, 8UL> device_name_;

  bool heartbeat_flag_;
};

}  // namespace generic_can_driver

#endif  // GENERIC_CAN_DRIVER__GENERIC_CAN_DRIVER_HPP_
