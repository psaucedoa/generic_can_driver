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
: rclcpp_lifecycle::LifecycleNode("generic_can_driver", OPTIONS)
{

}

GenericCanDriver::~GenericCanDriver() {}

// BGN ROS2 LIFECYCLE MANAGEMENT //
LNI::CallbackReturn GenericCanDriver::on_configure(const rlc::State & state)
{
  RCLCPP_INFO(this->get_logger(), "Configuring...");

  LNI::on_configure(state);

  dbw_dbc_file_ = this->declare_parameter<std::string>("dbw_dbc_file", "");
  frame_id_ = this->declare_parameter<std::string>("frame_id", "");
  sensor_name_ = this->declare_parameter<std::string>("sensor_name", "");
  device_ID_ = this->declare_parameter<uint8_t>("device_ID", 0);
  can_interface_ = this->declare_parameter<std::string>("can_interface", "can0");
  sub_topic_can_ = this->declare_parameter<std::string>("can_sub_topic", "");
  pub_topic_can_ = this->declare_parameter<std::string>("pub_topic_can", "");
  search_queue_ = this->declare_parameter<int>("search_queue", 0);
  use_full_dbc_ = this->declare_parameter<bool>("use_full_dbc", false);
  
  device_ID_str_ = boost::lexical_cast<std::string>(static_cast<int>(device_ID_));

  // printing to user
  RCLCPP_INFO(this->get_logger(), "dbw_dbc_file: %s", dbw_dbc_file_.c_str());
  RCLCPP_INFO(this->get_logger(), "frame_id: %s", frame_id_.c_str());
  RCLCPP_INFO(this->get_logger(), "sensor_name: %s", sensor_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "device_id: %d", device_ID_);
  RCLCPP_INFO(this->get_logger(), "sub_topic_can: %s", sub_topic_can_.c_str());
  RCLCPP_INFO(this->get_logger(), "pub_topic_can: %s", pub_topic_can_.c_str());
  
  // setup dbc database - brings in j1939 standard
  this->setupDatabase();
  RCLCPP_INFO(this->get_logger(), "Setup DBC Database");

  // find IDs present in the CAN data stream...
  while(!this->database_decimated_ && !this->use_full_dbc_)
  {
    // ... and then decimate the dbc database to only contain messages seen in CAN data stream
    rxSearchIDs();
    RCLCPP_INFO(this->get_logger(), "Decimated DBC Database");
  }
  
  // automatically configure publishers
  this->configurePublishers();
  
  RCLCPP_DEBUG(this->get_logger(), "Generic Can Driver Configured!");

  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn GenericCanDriver::on_activate(const rlc::State & state)
{
  LNI::on_activate(state);

  // activate all configured publishers
  this->activatePublishers();

  // setup subscriber, bind rxFrame
  this->sub_can_ = this->create_subscription<can_msgs::msg::Frame>(
      this->sub_topic_can_, 500, std::bind(&GenericCanDriver::rxFrame, this,
      std::placeholders::_1));

  RCLCPP_DEBUG(this->get_logger(), "Generic Can Driver Activated.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn GenericCanDriver::on_deactivate(const rlc::State & state)
{
  LNI::on_deactivate(state);

  // deactivate all publishers  
  this->deactivatePublishers();

  RCLCPP_DEBUG(this->get_logger(), "Generic Can Driver Deactivated.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn GenericCanDriver::on_cleanup(const rlc::State & state)
{
  LNI::on_cleanup(state);

  RCLCPP_DEBUG(this->get_logger(), "Generic Can Driver Cleaned Up.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn GenericCanDriver::on_shutdown(const rlc::State & state)
{
  LNI::on_shutdown(state);

  RCLCPP_DEBUG(this->get_logger(), "Generic Can Driver Shutting Down.");
  return LNI::CallbackReturn::SUCCESS;
}
// END ROS2 LIFECYCLE MANAGEMENT //

// BGN CANUSB COMMS FUNCTIONS //
void GenericCanDriver::rxSearchIDs()
{
  // setup a direct connection to the CAN line (since pub/sub stuff would still have to activate)
  std::unique_ptr<drivers::can::CanDriver> rx_can = std::make_unique<drivers::can::CanDriver>();
  rx_can->setupConnection(this->can_interface_.c_str());
  
  // loop over the amount of messages specified in the search_queue_ param 
  while(this->message_count_ < this->search_queue_)
  {
    // grab the CAN ID of the current message
    uint32_t incoming_ID = rx_can->receive().can_id;

    // make sure it's the correct source address (should prob just add a filter at the socket level)
    if((device_ID_ == (incoming_ID & 0x000000FFu)))
    {
      // add to found_ids_ map, updating the message count or creating a key if not already present
      this->found_ids_[incoming_ID & 0x00FFFF00u]++;
    }
    this->message_count_++;
  }

  // close the direct socket can connection
  rx_can->closeConnection();

  // create a copy to iterate over
  const std::map<uint32_t, NewEagle::DbcMessage> dbc_id_msg_map_copy = this->dbc_id_msg_map_; 
  for (auto [key, value] : dbc_id_msg_map_copy)
  {
    // if we did not see the key in any of the incoming CAN frames
    if (this->found_ids_.count(key) == 0)
    {
      // get the name of the message
      std::string message_name = this->dbc_id_msg_map_[key].GetName();
      // RCLCPP_INFO(this->get_logger(), "GETNAME: %s", message_name.c_str());

      // use this name as the key to delete it from the dbc_name_msg_map_ 
      // (this is used to generate publishers later on)
      this->dbc_name_msg_map_.erase(message_name);
      
      // then delete it from the id map
      this->dbc_id_msg_map_.erase(key);
    }
    else
    {}
  }
  RCLCPP_INFO(this->get_logger(), "Found %ld unique IDs on CAN interface", dbc_id_msg_map_.size());
  this->database_decimated_ = true;
  
  // print found IDs to user
  for(auto [key, value] : this->dbc_id_msg_map_)
  {
    RCLCPP_INFO(this->get_logger(), "REMAINING Ids | KEY: %ld VALUE: %s", key, value.GetName().c_str());
  }
}

void GenericCanDriver::rxFrame(const can_msgs::msg::Frame::SharedPtr MSG)
{
  // if message is not a request, error, and matches device ID
  if(!MSG->is_rtr && !MSG->is_error && (device_ID_ == (MSG->id & 0x000000FFu)))
  {
    // local const to store incoming message
    const can_msgs::msg::Frame::SharedPtr incoming_MSG = MSG;

    // if the message type / PGN is found in the dbc
    if(dbc_id_msg_map_.count(MSG->id & 0x00FFFF00u) )
    {
      // RCLCPP_INFO(this->get_logger(), "Key: %s", msg_name.c_str());

      // then create a local ros2 message
      j1939_interfaces::msg::CanData can_data;

      // translate the message data
      NewEagle::DbcMessage message = dbc_id_msg_map_[incoming_MSG->id & 0x00FFFF00u];
      message.SetFrame(incoming_MSG);

      // populate the local ros2 message header, frame, and message name
      can_data.header.stamp = this->now();
      can_data.header.frame_id = sensor_name_;
      can_data.message_name = message.GetName();
      can_data.hardware_id = device_ID_str_;

      // get the signals (e.g. x, y, z) within the message (e.g. acceleration)
      std::map<std::string, NewEagle::DbcSignal> signals_map = *message.GetSignals();
      for (auto [key_signal, value_signal] : signals_map)
      {
        // get the data for the current signal
        double result = message.GetSignal(key_signal)->GetResult();
        
        // populate the local ros2 message
        j1939_interfaces::msg::KeyFloatValue key_float_value;
        key_float_value.key = key_signal;
        key_float_value.value = result;
        can_data.values.push_back(key_float_value);
      }

      // publish finalized message
      publishers_[message.GetName()]->publish(can_data);
    }
  }
}
// END CANUSB COMMS FUNCTIONS //

// BEGIN MANAGEMENT FUNCTIONS //
void GenericCanDriver::setupDatabase()
{
  // build the new eagle dbc database
  this->dbw_dbc_db_ = NewEagle::DbcBuilder().NewDbc(dbw_dbc_file_);
  this->dbc_name_msg_map_ = * this->dbw_dbc_db_.GetMessages();

  // for every message in the database, leave only PGN
  for (auto [key, value] : dbc_name_msg_map_)
  {
    // strip id of priority and source address info
    uint32_t stripped_id = value.GetId() & 0x00FFFF00u;
    dbc_id_msg_map_[stripped_id] = value;
  }
}

void GenericCanDriver::configurePublishers()
{
  // iterate over the dbc to spawn an equal amount of publishers
  for (auto [key_message, value_message] : dbc_name_msg_map_)
  {
    RCLCPP_DEBUG(this->get_logger(), "Configuring Publishers - found key_message: %s", key_message.c_str());
    publishers_[key_message] = this->create_publisher<j1939_interfaces::msg::CanData>(sensor_name_ + "/" + key_message, 20);
  }
}

void GenericCanDriver::activatePublishers()
{
  // activate all publishers
  for (auto [name, publisher] : publishers_)
  {
    RCLCPP_DEBUG(this->get_logger(), "Activating Publisher: %s", name.c_str());
    publisher->on_activate();
  }
}

void GenericCanDriver::deactivatePublishers()
{
  // deactivate all publishers
  for (auto [name, publisher] : publishers_)
  {
    RCLCPP_DEBUG(this->get_logger(), "Deactivating Publisher: %s", name.c_str());
    publisher->on_deactivate();
  }
}

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

// END MANAGEMENT FUNCTIONS //

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
