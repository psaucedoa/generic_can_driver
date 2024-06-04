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

#include "generic_can_driver/generic_can_driver_node.hpp"

#include <memory>
#include <string>

using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
namespace rlc = rclcpp_lifecycle;

namespace generic_can_driver
{

GenericCanDriverNode::GenericCanDriverNode(const rclcpp::NodeOptions & OPTIONS)
: rclcpp_lifecycle::LifecycleNode("generic_driver_node", OPTIONS)
{
}

GenericCanDriverNode::~GenericCanDriverNode() {}

LNI::CallbackReturn GenericCanDriverNode::on_configure(const rlc::State & state)
{
  // (void)state;

  LNI::on_configure(state);

  try {
    RCLCPP_INFO(this->get_logger(), "test_param_config: ");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Error w/ on_configure: %s", e.what());
    return LNI::CallbackReturn::FAILURE;
  }

  RCLCPP_DEBUG(this->get_logger(), "Successfully Configured!");

  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn GenericCanDriverNode::on_activate(const rlc::State & state)
{
  LNI::on_activate(state);

  RCLCPP_DEBUG(this->get_logger(), "Generic activated.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn GenericCanDriverNode::on_deactivate(const rlc::State & state)
{
  // (void)state;

  LNI::on_deactivate(state);

  RCLCPP_DEBUG(this->get_logger(), "Generic deactivated.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn GenericCanDriverNode::on_cleanup(const rlc::State & state)
{
  // (void)state;

  LNI::on_cleanup(state);

  RCLCPP_DEBUG(this->get_logger(), "Generic cleaned up.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn GenericCanDriverNode::on_shutdown(const rlc::State & state)
{
  // (void)state;

  LNI::on_shutdown(state);

  RCLCPP_DEBUG(this->get_logger(), "Generic shutting down.");
  return LNI::CallbackReturn::SUCCESS;
}

void GenericCanDriverNode::txRename(
  const std::array<uint8_t, 8UL> name, const uint8_t new_source_address)
{
  std::array<uint8_t, 8UL> BAM_data_out = {0x20u, 0x09u, 0x00u, 0x02u, 0xFFu, 0xD8u, 0xFEu, 0x00u};
  can_msgs::msg::Frame BAM_frame_out;
  uint32_t j1939_id = 0x1CECFF00u;
  BAM_frame_out.header.stamp = this->now();
  BAM_frame_out.header.frame_id = "ROS2_command";
  BAM_frame_out.id = j1939_id;
  BAM_frame_out.is_rtr = false;
  BAM_frame_out.is_extended = true;
  BAM_frame_out.is_error = false;
  BAM_frame_out.dlc = 8;
  BAM_frame_out.data = BAM_data_out;

  std::array<uint8_t, 8UL> name_data_out_1 = {0x01u, name[0], name[1], name[2],
    name[3], name[4], name[5], name[6]};
  can_msgs::msg::Frame name_frame_out_1;
  j1939_id = 0x1CEBFF00u;
  name_frame_out_1.header.stamp = this->now();
  name_frame_out_1.header.frame_id = "ROS2_command";
  name_frame_out_1.id = j1939_id;
  name_frame_out_1.is_rtr = false;
  name_frame_out_1.is_extended = true;
  name_frame_out_1.is_error = false;
  name_frame_out_1.dlc = 8;
  name_frame_out_1.data = name_data_out_1;

  std::array<uint8_t, 8UL> name_data_out_2 = {0x02u, name[7], new_source_address, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF};
  can_msgs::msg::Frame name_frame_out_2;
  j1939_id = 0x1CEBFF00u;
  name_frame_out_2.header.stamp = this->now();
  name_frame_out_2.header.frame_id = "ROS2_command";
  name_frame_out_2.id = j1939_id;
  name_frame_out_2.is_rtr = false;
  name_frame_out_2.is_extended = true;
  name_frame_out_2.is_error = false;
  name_frame_out_2.dlc = 8;
  name_frame_out_2.data = name_data_out_2;

  pub_can_->publish(BAM_frame_out);
  rclcpp::sleep_for(std::chrono::milliseconds(100));
  pub_can_->publish(name_frame_out_1);
  rclcpp::sleep_for(std::chrono::milliseconds(100));
  pub_can_->publish(name_frame_out_2);
  RCLCPP_INFO(this->get_logger(), "Published renaming thing!!!!!!!! %d", new_source_address);
}

void GenericCanDriverNode::createDataArray(
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

// STANDARD J1939 CAN PARSING FUNCTIONS //

void GenericCanDriverNode::recvAMB(const can_msgs::msg::Frame::SharedPtr MSG)
{
  NewEagle::DbcMessage * message = dbw_dbc_db.GetMessage("AMB"); // 2566845950
  // if minimum datalength is met
  if (MSG->dlc >= message->GetDlc())
  {
    message->SetFrame(MSG);
    
    const double ambient_air_temp = message->GetSignal("AmbientAirTemperature")->GetResult();
    diagnostic_msgs::msg::KeyValue ambient_air_temp_key_value;
    ambient_air_temp_key_value.key = "Ambient Air Temperature";
    ambient_air_temp_key_value.value = std::to_string(ambient_air_temp);

    const double barometric_pressure = message->GetSignal("BarometricPressure")->GetResult();
    diagnostic_msgs::msg::KeyValue barometric_pressure_key_value;
    barometric_pressure_key_value.key = "Barometric Pressure";
    barometric_pressure_key_value.value = std::to_string(barometric_pressure);
    
    const double cab_temp = message->GetSignal("CabInteriorTemperature")->GetResult();
    diagnostic_msgs::msg::KeyValue cab_temp_key_value;
    cab_temp_key_value.key = "Cab Interior Temperature";
    cab_temp_key_value.value = std::to_string(cab_temp);
    
    const double engine_in_temp = message->GetSignal("EngineIntake1AirTemperature")->GetResult();
    diagnostic_msgs::msg::KeyValue engine_in_temp_key_value;
    engine_in_temp_key_value.key = "Engine Intake 1 Air Temperature";
    engine_in_temp_key_value.value = std::to_string(engine_in_temp);
    
    const double road_surface_temp = message->GetSignal("RoadSurfaceTemperature")->GetResult();
    diagnostic_msgs::msg::KeyValue road_surface_temp_key_value;
    road_surface_temp_key_value.key = "Road Surface Temperature Temperature";
    road_surface_temp_key_value.value = std::to_string(road_surface_temp);
    
    AMB_status_.level = 0;
    AMB_status_.name = "AMB";
    AMB_status_.message = "Ambient Conditions";
    AMB_status_.hardware_id = "Source Address: " + std::to_string(MSG->id & 0x000000FFu);
    AMB_status_.values = {
      ambient_air_temp_key_value,
      barometric_pressure_key_value,
      cab_temp_key_value,
      engine_in_temp_key_value,
      road_surface_temp_key_value
    };

    AMB_.header.stamp = MSG->header.stamp;
    AMB_.header.frame_id = frame_id_;
    AMB_.status = {AMB_status_};

    pub_AMB_->publish(AMB_);
  }
}

void GenericCanDriverNode::recvAT1T1I1(const can_msgs::msg::Frame::SharedPtr MSG)
{
  NewEagle::DbcMessage * message = dbw_dbc_db.GetMessage("AT1T1I1"); // 2566805246
  // if minimum datalength is met
  if (MSG->dlc >= message->GetDlc())
  {
    message->SetFrame(MSG);

    const double DEF_heater = message->GetSignal("Aftrtrtmnt1DslExhstFldTnkHtr")->GetResult();
    diagnostic_msgs::msg::KeyValue DEF_heater_key_value_;
    DEF_heater_key_value_.key = "Aftertreatment 1 Diesel Exhaust Fluid Tank Heater [%]";
    DEF_heater_key_value_.value = std::to_string(DEF_heater);

    const double DEF_level = message->GetSignal("Aftrtrtmnt1DslExhstFldTnkLvl")->GetResult();
    diagnostic_msgs::msg::KeyValue DEF_level_key_value_;
    DEF_level_key_value_.key = "Aftertreatment 1 Diesel Exhaust Fluid Tank Level [mm]";
    DEF_level_key_value_.value = std::to_string(DEF_level);

    const double DEF_volume = message->GetSignal("Aftrtrtmnt1DslExhstFldTnkVlm")->GetResult();
    diagnostic_msgs::msg::KeyValue DEF_volume_key_value_;
    DEF_volume_key_value_.key = "Aftertreatment 1 Diesel Exhaust Fluid Tank Volume [%]";
    DEF_volume_key_value_.value = std::to_string(DEF_volume);

    const double DEF_temp = message->GetSignal("Atttt1DsExstFdTTpt1")->GetResult();
    diagnostic_msgs::msg::KeyValue DEF_temp_key_value_;
    DEF_temp_key_value_.key = "Aftertreatment 1 Diesel Exhaust Fluid Tank Temperature [degC]";
    DEF_temp_key_value_.value = std::to_string(DEF_temp);

    const double DEF_low = message->GetSignal("AttttDsExstFdTLwLvIdt")->GetResult();
    diagnostic_msgs::msg::KeyValue DEF_low_key_value_;
    DEF_low_key_value_.key = "Aftertreatment Diesel Exhaust Fluid Tank Low Level";
    DEF_low_key_value_.value = std::to_string(DEF_low);

    const double DEF_fmi = message->GetSignal("Atttt1DsExstFdTLvVPF")->GetResult();
    diagnostic_msgs::msg::KeyValue DEF_fmi_key_value_;
    DEF_fmi_key_value_.key = "Aftertreatment 1 Diesel Exhaust Fluid Tank Level/Volume Preliminary FMI";
    DEF_fmi_key_value_.value = std::to_string(DEF_fmi);

    const double DEF_temp_fmi = message->GetSignal("Atttt1DsExstFdT1TptPF")->GetResult();
    diagnostic_msgs::msg::KeyValue DEF_temp_fmi_key_value_;
    DEF_temp_fmi_key_value_.key = "Aftertreatment 1 Diesel Exhaust Fluid Tank 1 Temperature Preliminary FMI";
    DEF_temp_fmi_key_value_.value = std::to_string(DEF_temp_fmi);

    const double SCR = message->GetSignal("AftrtrtmntSrOprtrIndmntSvrt")->GetResult();
    diagnostic_msgs::msg::KeyValue SCR_key_value_;
    SCR_key_value_.key = "Aftertreatment SCR Operator Inducement Severity";
    SCR_key_value_.value = std::to_string(SCR);

    const double DEF_heater_fmi = message->GetSignal("Atttt1DsExstFdT1HtPF")->GetResult();
    diagnostic_msgs::msg::KeyValue DEF_heater_fmi_key_value_;
    DEF_heater_fmi_key_value_.key = "Aftertreatment 1 Diesel Exhaust Fluid Tank 1 Heater Preliminary FMI";
    DEF_heater_fmi_key_value_.value = std::to_string(DEF_heater_fmi);

    AT1T1I1_status_.level = 0;
    AT1T1I1_status_.name = "AT1T1I1";
    AT1T1I1_status_.message = "Aftertreatment 1 Diesel Exhaust Fluid Tank 1 Information 1";
    AT1T1I1_status_.hardware_id = "Source Address: " + std::to_string(MSG->id & 0x000000FFu);
    AT1T1I1_status_.values = {
      DEF_heater_key_value_,
      DEF_level_key_value_,
      DEF_volume_key_value_,
      DEF_temp_key_value_,
      DEF_low_key_value_,
      DEF_fmi_key_value_,
      DEF_temp_fmi_key_value_,
      SCR_key_value_,
      DEF_heater_fmi_key_value_
    };

    AT1T1I1_.header.stamp = MSG->header.stamp;
    AT1T1I1_.header.frame_id = frame_id_;
    AT1T1I1_.status = {AT1T1I1_status_};

    pub_AT1T1I1_->publish(AT1T1I1_);
  }
}

void GenericCanDriverNode::recvBJM1(const can_msgs::msg::Frame::SharedPtr MSG)
{
  NewEagle::DbcMessage * message = dbw_dbc_db.GetMessage("BJM1"); // 2566845950
  // if minimum datalength is met
  if (MSG->dlc >= message->GetDlc())
  {
    message->SetFrame(MSG);
    
    const double joy1_button_1 = message->GetSignal("Joystick1Button1PressedStatus")->GetResult();
    diagnostic_msgs::msg::KeyValue joy1_button_1_key_value;
    joy1_button_1_key_value.key = "Joystick 1 Button 1 Pressed Status";
    joy1_button_1_key_value.value = std::to_string(joy1_button_1);

    const double joy1_button_2 = message->GetSignal("Joystick1Button2PressedStatus")->GetResult();
    diagnostic_msgs::msg::KeyValue joy1_button_2_key_value;
    joy1_button_2_key_value.key = "Joystick 1 Button 2 Pressed Status";
    joy1_button_2_key_value.value = std::to_string(joy1_button_2);

    const double joy1_button_3 = message->GetSignal("Joystick1Button3PressedStatus")->GetResult();
    diagnostic_msgs::msg::KeyValue joy1_button_3_key_value;
    joy1_button_3_key_value.key = "Joystick 1 Button 3 Pressed Status";
    joy1_button_3_key_value.value = std::to_string(joy1_button_3);

    const double joy1_button_4 = message->GetSignal("Joystick1Button4PressedStatus")->GetResult();
    diagnostic_msgs::msg::KeyValue joy1_button_4_key_value;
    joy1_button_4_key_value.key = "Joystick 1 Button 4 Pressed Status";
    joy1_button_4_key_value.value = std::to_string(joy1_button_4);

    const double joy1_button_5 = message->GetSignal("Joystick1Button5PressedStatus")->GetResult();
    diagnostic_msgs::msg::KeyValue joy1_button_5_key_value;
    joy1_button_5_key_value.key = "Joystick 1 Button 5 Pressed Status";
    joy1_button_5_key_value.value = std::to_string(joy1_button_5);

    const double joy1_button_6 = message->GetSignal("Joystick1Button6PressedStatus")->GetResult();
    diagnostic_msgs::msg::KeyValue joy1_button_6_key_value;
    joy1_button_6_key_value.key = "Joystick 1 Button 6 Pressed Status";
    joy1_button_6_key_value.value = std::to_string(joy1_button_6);

    const double joy1_button_7 = message->GetSignal("Joystick1Button7PressedStatus")->GetResult();
    diagnostic_msgs::msg::KeyValue joy1_button_7_key_value;
    joy1_button_7_key_value.key = "Joystick 1 Button 7 Pressed Status";
    joy1_button_7_key_value.value = std::to_string(joy1_button_7);

    const double joy1_button_8 = message->GetSignal("Joystick1Button8PressedStatus")->GetResult();
    diagnostic_msgs::msg::KeyValue joy1_button_8_key_value;
    joy1_button_8_key_value.key = "Joystick 1 Button 8 Pressed Status";
    joy1_button_8_key_value.value = std::to_string(joy1_button_8);

    const double joy1_button_9 = message->GetSignal("Joystick1Button9PressedStatus")->GetResult();
    diagnostic_msgs::msg::KeyValue joy1_button_9_key_value;
    joy1_button_9_key_value.key = "Joystick 1 Button 9 Pressed Status";
    joy1_button_9_key_value.value = std::to_string(joy1_button_9);

    const double joy1_button_10 = message->GetSignal("Joystick1Button10PressedStatus")->GetResult();
    diagnostic_msgs::msg::KeyValue joy1_button_10_key_value;
    joy1_button_10_key_value.key = "Joystick 1 Button 10 Pressed Status";
    joy1_button_10_key_value.value = std::to_string(joy1_button_10);

    const double joy1_button_11 = message->GetSignal("Joystick1Button11PressedStatus")->GetResult();
    diagnostic_msgs::msg::KeyValue joy1_button_11_key_value;
    joy1_button_11_key_value.key = "Joystick 1 Button 11 Pressed Status";
    joy1_button_11_key_value.value = std::to_string(joy1_button_11);

    const double joy1_button_12 = message->GetSignal("Joystick1Button12PressedStatus")->GetResult();
    diagnostic_msgs::msg::KeyValue joy1_button_12_key_value;
    joy1_button_12_key_value.key = "Joystick 1 Button 12 Pressed Status";
    joy1_button_12_key_value.value = std::to_string(joy1_button_12);

    const double joy1_X_axis = message->GetSignal("Joystick1XAxisPosition")->GetResult();
    diagnostic_msgs::msg::KeyValue joy1_X_axis_key_value;
    joy1_X_axis_key_value.key = "Joystick 1 X-Axis Position";
    joy1_X_axis_key_value.value = std::to_string(joy1_X_axis);

    const double joy1_Y_axis = message->GetSignal("Joystick1YAxisPosition")->GetResult();
    diagnostic_msgs::msg::KeyValue joy1_Y_axis_key_value;
    joy1_Y_axis_key_value.key = "Joystick 1 Y-Axis Position";
    joy1_Y_axis_key_value.value = std::to_string(joy1_Y_axis);

    const double joy1_X_axis_detent = message->GetSignal("Jstk1XAxsDtntPstnStts")->GetResult();
    diagnostic_msgs::msg::KeyValue joy1_X_axis_detent_key_value;
    joy1_X_axis_detent_key_value.key = "Joystick 1 X-Axis Detent Position Status";
    joy1_X_axis_detent_key_value.value = std::to_string(joy1_X_axis_detent);

    const double joy1_X_axis_lvr_l = message->GetSignal("Jstk1XAxsLvrLftNgtvPstnStts")->GetResult();
    diagnostic_msgs::msg::KeyValue joy1_X_axis_lvr_l_key_value;
    joy1_X_axis_lvr_l_key_value.key = "Joystick 1 X-Axis Lever Left Negative Position Status";
    joy1_X_axis_lvr_l_key_value.value = std::to_string(joy1_X_axis_lvr_l);

    const double joy1_X_axis_lvr_r = message->GetSignal("Jstk1XAxsLvrRghtPstvPstnStts")->GetResult();
    diagnostic_msgs::msg::KeyValue joy1_X_axis_lvr_r_key_value;
    joy1_X_axis_lvr_r_key_value.key = "Joystick 1 X-Axis Lever Right Positive Position Status";
    joy1_X_axis_lvr_r_key_value.value = std::to_string(joy1_X_axis_lvr_r);

    const double joy1_X_axis_neutral = message->GetSignal("Jstk1XAxsNtrlPstnStts")->GetResult();
    diagnostic_msgs::msg::KeyValue joy1_X_axis_neutral_key_value;
    joy1_X_axis_neutral_key_value.key = "Joystick 1 X-Axis Neutral Position Status";
    joy1_X_axis_neutral_key_value.value = std::to_string(joy1_X_axis_neutral);

    const double joy1_Y_axis_detent = message->GetSignal("Jstk1YAxsDtntPstnStts")->GetResult();
    diagnostic_msgs::msg::KeyValue joy1_Y_axis_detent_key_value;
    joy1_Y_axis_detent_key_value.key = "Joystick 1 Y-Axis Detent Position Status";
    joy1_Y_axis_detent_key_value.value = std::to_string(joy1_Y_axis_detent);

    const double joy1_Y_axis_lvr_b = message->GetSignal("Jstk1YAxsLvrBkNgtvPstnStts")->GetResult();
    diagnostic_msgs::msg::KeyValue joy1_Y_axis_lvr_b_key_value;
    joy1_Y_axis_lvr_b_key_value.key = "Joystick 1 Y-Axis Lever Back Negative Position Status";
    joy1_Y_axis_lvr_b_key_value.value = std::to_string(joy1_Y_axis_lvr_b);

    const double joy1_Y_axis_lvr_f = message->GetSignal("Jstk1YAxsLvrFrwrdPstvPstnStts")->GetResult();
    diagnostic_msgs::msg::KeyValue joy1_Y_axis_lvr_f_key_value;
    joy1_Y_axis_lvr_f_key_value.key = "Joystick 1 Y-Axis Lever Forward Positive Position Status";
    joy1_Y_axis_lvr_f_key_value.value = std::to_string(joy1_Y_axis_lvr_f);

    const double joy1_Y_axis_neutral = message->GetSignal("Jstk1YAxsNtrlPstnStts")->GetResult();
    diagnostic_msgs::msg::KeyValue joy1_Y_axis_neutral_key_value;
    joy1_Y_axis_neutral_key_value.key = "Joystick 1 Y-Axis Neutral Position Status";
    joy1_Y_axis_neutral_key_value.value = std::to_string(joy1_Y_axis_neutral);

    BJM1_status_.level = 0;
    BJM1_status_.name = "BJM1";
    BJM1_status_.message = "Basic Joystick Message 1";
    BJM1_status_.hardware_id = "Source Address: " + std::to_string(MSG->id & 0x000000FFu);
    BJM1_status_.values = {
      joy1_button_1_key_value,
      joy1_button_2_key_value,
      joy1_button_3_key_value,
      joy1_button_4_key_value,
      joy1_button_5_key_value,
      joy1_button_6_key_value,
      joy1_button_7_key_value,
      joy1_button_8_key_value,
      joy1_button_9_key_value,
      joy1_button_10_key_value,
      joy1_button_11_key_value,
      joy1_button_12_key_value,
      joy1_X_axis_key_value,
      joy1_Y_axis_key_value,
      joy1_X_axis_detent_key_value,
      joy1_X_axis_lvr_l_key_value,
      joy1_X_axis_lvr_r_key_value,
      joy1_X_axis_neutral_key_value,
      joy1_Y_axis_detent_key_value,
      joy1_Y_axis_lvr_b_key_value,
      joy1_Y_axis_lvr_f_key_value,
      joy1_Y_axis_neutral_key_value
    };

    BJM1_.header.stamp = MSG->header.stamp;
    BJM1_.header.frame_id = frame_id_;
    BJM1_.status = {BJM1_status_};

    /////////////
    // TODO (Arturo) : Add params for different controller configurations?

    float left_stick_horizontal = joy1_X_axis / 100.;
    if(joy1_X_axis_lvr_l == 1.0)
    {
      left_stick_horizontal = -1*left_stick_horizontal;
    }
    
    float left_stick_vertical = joy1_Y_axis / 100.;
    if(joy1_Y_axis_lvr_b == 1.0)
    {
      left_stick_vertical = -1*left_stick_vertical;
    }

    xbox_.header.stamp = MSG->header.stamp;
    xbox_.header.frame_id = frame_id_;

    // buttons
    // xbox_.buttons[0]  = (0);  // A
    xbox_.buttons[1]  = (1);  // B
    xbox_.buttons[2]  = (1);  // X
    // xbox_.buttons[3]  = (0);  // Y
    // xbox_.buttons[4]  = (0);  // Left Shoulder
    // xbox_.buttons[5]  = (0);  // Right Shoulder
    // xbox_.buttons[6]  = (0);  // Back
    // xbox_.buttons[7]  = (0);  // Start
    // xbox_.buttons[8]  = (0);  // Power
    // xbox_.buttons[9]  = (0);  // Left Stick Press
    // xbox_.buttons[10] = (0);  // Right Stick Press

    // sticks - if this (BJM1) is movement, i want this to be the left stick
    xbox_.axes[0] = (0.0);  // Left Stick Horizontal
    xbox_.axes[1] = (0.0);  // Left Stick Vertical
    xbox_.axes[2] = (0.0);  // Left Trigger
    xbox_.axes[3] = (left_stick_horizontal);  // Right Stick Horizontal
    xbox_.axes[4] = (left_stick_vertical);  // Right Stick Vertical
    xbox_.axes[5] = (0.0);  // Right Trigger
    xbox_.axes[6] = (0.0);  // D-Pad Horizontal
    xbox_.axes[7] = (0.0);  // D-Pad Vertical

    pub_BJM1_->publish(BJM1_);
  }
}

void GenericCanDriverNode::recvBJM2(const can_msgs::msg::Frame::SharedPtr MSG)
{
  NewEagle::DbcMessage * message = dbw_dbc_db.GetMessage("BJM2"); // 2566845950
  // if minimum datalength is met
  if (MSG->dlc >= message->GetDlc())
  {
    message->SetFrame(MSG);
    
    const double joy2_button_1 = message->GetSignal("Joystick2Button1PressedStatus")->GetResult();
    diagnostic_msgs::msg::KeyValue joy2_button_1_key_value;
    joy2_button_1_key_value.key = "Joystick 2 Button 1 Pressed Status";
    joy2_button_1_key_value.value = std::to_string(joy2_button_1);

    const double joy2_button_2 = message->GetSignal("Joystick2Button2PressedStatus")->GetResult();
    diagnostic_msgs::msg::KeyValue joy2_button_2_key_value;
    joy2_button_2_key_value.key = "Joystick 2 Button 2 Pressed Status";
    joy2_button_2_key_value.value = std::to_string(joy2_button_2);

    const double joy2_button_3 = message->GetSignal("Joystick2Button3PressedStatus")->GetResult();
    diagnostic_msgs::msg::KeyValue joy2_button_3_key_value;
    joy2_button_3_key_value.key = "Joystick 2 Button 3 Pressed Status";
    joy2_button_3_key_value.value = std::to_string(joy2_button_3);

    const double joy2_button_4 = message->GetSignal("Joystick2Button4PressedStatus")->GetResult();
    diagnostic_msgs::msg::KeyValue joy2_button_4_key_value;
    joy2_button_4_key_value.key = "Joystick 2 Button 4 Pressed Status";
    joy2_button_4_key_value.value = std::to_string(joy2_button_4);

    const double joy2_button_5 = message->GetSignal("Joystick2Button5PressedStatus")->GetResult();
    diagnostic_msgs::msg::KeyValue joy2_button_5_key_value;
    joy2_button_5_key_value.key = "Joystick 2 Button 5 Pressed Status";
    joy2_button_5_key_value.value = std::to_string(joy2_button_5);

    const double joy2_button_6 = message->GetSignal("Joystick2Button6PressedStatus")->GetResult();
    diagnostic_msgs::msg::KeyValue joy2_button_6_key_value;
    joy2_button_6_key_value.key = "Joystick 2 Button 6 Pressed Status";
    joy2_button_6_key_value.value = std::to_string(joy2_button_6);

    const double joy2_button_7 = message->GetSignal("Joystick2Button7PressedStatus")->GetResult();
    diagnostic_msgs::msg::KeyValue joy2_button_7_key_value;
    joy2_button_7_key_value.key = "Joystick 2 Button 7 Pressed Status";
    joy2_button_7_key_value.value = std::to_string(joy2_button_7);

    const double joy2_button_8 = message->GetSignal("Joystick2Button8PressedStatus")->GetResult();
    diagnostic_msgs::msg::KeyValue joy2_button_8_key_value;
    joy2_button_8_key_value.key = "Joystick 2 Button 8 Pressed Status";
    joy2_button_8_key_value.value = std::to_string(joy2_button_8);

    const double joy2_button_9 = message->GetSignal("Joystick2Button9PressedStatus")->GetResult();
    diagnostic_msgs::msg::KeyValue joy2_button_9_key_value;
    joy2_button_9_key_value.key = "Joystick 2 Button 9 Pressed Status";
    joy2_button_9_key_value.value = std::to_string(joy2_button_9);

    const double joy2_button_10 = message->GetSignal("Joystick2Button10PressedStatus")->GetResult();
    diagnostic_msgs::msg::KeyValue joy2_button_10_key_value;
    joy2_button_10_key_value.key = "Joystick 2 Button 10 Pressed Status";
    joy2_button_10_key_value.value = std::to_string(joy2_button_10);

    const double joy2_button_11 = message->GetSignal("Joystick2Button11PressedStatus")->GetResult();
    diagnostic_msgs::msg::KeyValue joy2_button_11_key_value;
    joy2_button_11_key_value.key = "Joystick 2 Button 11 Pressed Status";
    joy2_button_11_key_value.value = std::to_string(joy2_button_11);

    const double joy2_button_12 = message->GetSignal("Joystick2Button12PressedStatus")->GetResult();
    diagnostic_msgs::msg::KeyValue joy2_button_12_key_value;
    joy2_button_12_key_value.key = "Joystick 2 Button 12 Pressed Status";
    joy2_button_12_key_value.value = std::to_string(joy2_button_12);

    const double joy2_X_axis = message->GetSignal("Joystick2XAxisPosition")->GetResult();
    diagnostic_msgs::msg::KeyValue joy2_X_axis_key_value;
    joy2_X_axis_key_value.key = "Joystick 2 X-Axis Position";
    joy2_X_axis_key_value.value = std::to_string(joy2_X_axis);

    const double joy2_Y_axis = message->GetSignal("Joystick2YAxisPosition")->GetResult();
    diagnostic_msgs::msg::KeyValue joy2_Y_axis_key_value;
    joy2_Y_axis_key_value.key = "Joystick 2 Y-Axis Position";
    joy2_Y_axis_key_value.value = std::to_string(joy2_Y_axis);

    const double joy2_X_axis_detent = message->GetSignal("Jstk2XAxsDtntPstnStts")->GetResult();
    diagnostic_msgs::msg::KeyValue joy2_X_axis_detent_key_value;
    joy2_X_axis_detent_key_value.key = "Joystick 2 X-Axis Detent Position Status";
    joy2_X_axis_detent_key_value.value = std::to_string(joy2_X_axis_detent);

    const double joy2_X_axis_lvr_l = message->GetSignal("Jstk2XAxsLvrLftNgtvPstnStts")->GetResult();
    diagnostic_msgs::msg::KeyValue joy2_X_axis_lvr_l_key_value;
    joy2_X_axis_lvr_l_key_value.key = "Joystick 2 X-Axis Lever Left Negative Position Status";
    joy2_X_axis_lvr_l_key_value.value = std::to_string(joy2_X_axis_lvr_l);

    const double joy2_X_axis_lvr_r = message->GetSignal("Jstk2XAxsLvrRghtPstvPstnStts")->GetResult();
    diagnostic_msgs::msg::KeyValue joy2_X_axis_lvr_r_key_value;
    joy2_X_axis_lvr_r_key_value.key = "Joystick 2 X-Axis Lever Right Positive Position Status";
    joy2_X_axis_lvr_r_key_value.value = std::to_string(joy2_X_axis_lvr_r);

    const double joy2_X_axis_neutral = message->GetSignal("Jstk2XAxsNtrlPstnStts")->GetResult();
    diagnostic_msgs::msg::KeyValue joy2_X_axis_neutral_key_value;
    joy2_X_axis_neutral_key_value.key = "Joystick 2 X-Axis Neutral Position Status";
    joy2_X_axis_neutral_key_value.value = std::to_string(joy2_X_axis_neutral);

    const double joy2_Y_axis_detent = message->GetSignal("Jstk2YAxsDtntPstnStts")->GetResult();
    diagnostic_msgs::msg::KeyValue joy2_Y_axis_detent_key_value;
    joy2_Y_axis_detent_key_value.key = "Joystick 2 Y-Axis Detent Position Status";
    joy2_Y_axis_detent_key_value.value = std::to_string(joy2_Y_axis_detent);

    const double joy2_Y_axis_lvr_b = message->GetSignal("Jstk2YAxsLvrBkNgtvPstnStts")->GetResult();
    diagnostic_msgs::msg::KeyValue joy2_Y_axis_lvr_b_key_value;
    joy2_Y_axis_lvr_b_key_value.key = "Joystick 2 Y-Axis Lever Back Negative Position Status";
    joy2_Y_axis_lvr_b_key_value.value = std::to_string(joy2_Y_axis_lvr_b);

    const double joy2_Y_axis_lvr_f = message->GetSignal("Jstk2YAxsLvrFrwrdPstvPstnStts")->GetResult();
    diagnostic_msgs::msg::KeyValue joy2_Y_axis_lvr_f_key_value;
    joy2_Y_axis_lvr_f_key_value.key = "Joystick 2 Y-Axis Lever Forward Positive Position Status";
    joy2_Y_axis_lvr_f_key_value.value = std::to_string(joy2_Y_axis_lvr_f);

    const double joy2_Y_axis_neutral = message->GetSignal("Jstk2YAxsNtrlPstnStts")->GetResult();
    diagnostic_msgs::msg::KeyValue joy2_Y_axis_neutral_key_value;
    joy2_Y_axis_neutral_key_value.key = "Joystick 2 Y-Axis Neutral Position Status";
    joy2_Y_axis_neutral_key_value.value = std::to_string(joy2_Y_axis_neutral);

    BJM2_status_.level = 0;
    BJM2_status_.name = "BJM2";
    BJM2_status_.message = "Basic Joystick Message 2";
    BJM2_status_.hardware_id = "Source Address: " + std::to_string(MSG->id & 0x000000FFu);
    BJM2_status_.values = {
      joy2_button_1_key_value,
      joy2_button_2_key_value,
      joy2_button_3_key_value,
      joy2_button_4_key_value,
      joy2_button_5_key_value,
      joy2_button_6_key_value,
      joy2_button_7_key_value,
      joy2_button_8_key_value,
      joy2_button_9_key_value,
      joy2_button_10_key_value,
      joy2_button_11_key_value,
      joy2_button_12_key_value,
      joy2_X_axis_key_value,
      joy2_Y_axis_key_value,
      joy2_X_axis_detent_key_value,
      joy2_X_axis_lvr_l_key_value,
      joy2_X_axis_lvr_r_key_value,
      joy2_X_axis_neutral_key_value,
      joy2_Y_axis_detent_key_value,
      joy2_Y_axis_lvr_b_key_value,
      joy2_Y_axis_lvr_f_key_value,
      joy2_Y_axis_neutral_key_value
    };

    BJM2_.header.stamp = MSG->header.stamp;
    BJM2_.header.frame_id = frame_id_;
    BJM2_.status = {BJM2_status_};

    /////////////
    // TODO (Arturo) : Add params for different controller configurations?

    float right_stick_horizontal = joy2_X_axis / 100.;
    if(joy2_X_axis_lvr_l == 1.0)
    {
      right_stick_horizontal = -1*right_stick_horizontal;
    }
    
    float right_stick_vertical = joy2_Y_axis / 100.;
    if(joy2_Y_axis_lvr_b == 1.0)
    {
      right_stick_vertical = -1*right_stick_vertical;
    }

    // xbox_.header.stamp = MSG->header.stamp;
    // xbox_.header.frame_id = frame_id_;

    // buttons - idk what's what so I'll just set these to zero in BJM1 
    // xbox_.buttons.[0] = (0);  // A
    // xbox_.buttons.[1] = (0);  // B
    // xbox_.buttons.[2] = (0);  // X
    // xbox_.buttons.[3] = (0);  // Y
    // xbox_.buttons.[4] = (0);  // Left Shoulder
    // xbox_.buttons.[5] = (0);  // Right Shoulder
    // xbox_.buttons.[6] = (0);  // Back
    // xbox_.buttons.[7] = (0);  // Start
    // xbox_.buttons.[8] = (0);  // Power
    // xbox_.buttons.[9] = (0);  // Left Stick Press
    // xbox_.buttons.[10] = (0);  // Right Stick Press

    // sticks - if this (BJM2) is tool, i want this to be the right stick
    xbox_.axes[0] = (right_stick_horizontal);  // Left Stick Horizontal
    xbox_.axes[1] = (right_stick_vertical);  // Left Stick Vertical
    // xbox_.axes[2] = (0.0);  // Left Trigger
    // xbox_.axes[3] = (right_stick_horizontal);  // Right Stick Horizontal
    // xbox_.axes[4] = (right_stick_vertical);  // Right Stick Vertical
    // xbox_.axes[5] = (0.0);  // Right Trigger
    // xbox_.axes[6] = (0.0);  // D-Pad Horizontal
    // xbox_.axes[7] = (0.0);  // D-Pad Vertical

    pub_BJM2_->publish(BJM2_);
    // pub_xbox_->publish(xbox_);

    for (int i = 0; i < 11; i++)
    {
      xbox_.axes[i] = 0.0;
    }

    for (int i = 0; i < 8; i++)
    {
      xbox_.buttons[i] = 0;
    }

  }
}

void GenericCanDriverNode::recvCCVS1(const can_msgs::msg::Frame::SharedPtr MSG)
{
  NewEagle::DbcMessage * message = dbw_dbc_db.GetMessage("CCVS1"); // 2566844926
  // if minimum datalength is met
  if (MSG->dlc >= message->GetDlc())
  {
    message->SetFrame(MSG);

    double parking_brake_value = message->GetSignal("ParkingBrakeSwitch")->GetResult();
    diagnostic_msgs::msg::KeyValue parking_brake_key_value;
    parking_brake_key_value.key = "Parking Brake";
    if (parking_brake_value == 0.)
    {
      parking_brake_key_value.value = "Parking brake NOT set";
      CCVS1_status_.level = 0;
    }
    if (parking_brake_value == 1.)
    {
      parking_brake_key_value.value = "Parking brake set";
      CCVS1_status_.level = 1; 
    }
    if (parking_brake_value == 2.)
    {
      parking_brake_key_value.value = "ERROR";
      CCVS1_status_.level = 2; 
    }
    if (parking_brake_value == 3.)
    {
      parking_brake_key_value.value = "Not Available";
      CCVS1_status_.level = 0; 
    }
    else
    {
      parking_brake_key_value.value = "Not Available";
      CCVS1_status_.level = 0;
    }

    const double brake_switch = message->GetSignal("BrakeSwitch")->GetResult();
    diagnostic_msgs::msg::KeyValue brake_switch_key_value;
    brake_switch_key_value.key = "Brake Switch";
    brake_switch_key_value.value = std::to_string(brake_switch);

    const double clutch_switch = message->GetSignal("ClutchSwitch")->GetResult();
    diagnostic_msgs::msg::KeyValue clutch_switch_key_value;
    clutch_switch_key_value.key = "Clutch Switch";
    clutch_switch_key_value.value = std::to_string(clutch_switch);

    const double cruise_coast_switch = message->GetSignal("CrsCntrlCstDlrtSwth")->GetResult();
    diagnostic_msgs::msg::KeyValue cruise_coast_switch_key_value;
    cruise_coast_switch_key_value.key = "Cruise Control Coast (Decelerate) Switch";
    cruise_coast_switch_key_value.value = std::to_string(cruise_coast_switch);

    const double cruise_accel_switch = message->GetSignal("CruiseControlAccelerateSwitch")->GetResult();
    diagnostic_msgs::msg::KeyValue cruise_accel_switch_key_value;
    cruise_accel_switch_key_value.key = "Cruise Control Accelerate Switch";
    cruise_accel_switch_key_value.value = std::to_string(cruise_accel_switch);

    const double cruise_activate_switch = message->GetSignal("CruiseControlActive")->GetResult();
    diagnostic_msgs::msg::KeyValue cruise_activate_switch_key_value;
    cruise_activate_switch_key_value.key = "Cruise Control Activate Switch";
    cruise_activate_switch_key_value.value = std::to_string(cruise_activate_switch);

    const double cruise_enable_switch = message->GetSignal("CruiseControlEnableSwitch")->GetResult();
    diagnostic_msgs::msg::KeyValue cruise_enable_switch_key_value;
    cruise_enable_switch_key_value.key = "Cruise Control Enable Switch";
    cruise_enable_switch_key_value.value = std::to_string(cruise_enable_switch);

    const double cruise_pause_switch = message->GetSignal("CruiseControlPauseSwitch")->GetResult();
    diagnostic_msgs::msg::KeyValue cruise_pause_switch_key_value;
    cruise_pause_switch_key_value.key = "Cruise Control Pause Switch";
    cruise_pause_switch_key_value.value = std::to_string(cruise_pause_switch);

    const double cruise_resume_switch = message->GetSignal("CruiseControlResumeSwitch")->GetResult();
    diagnostic_msgs::msg::KeyValue cruise_resume_switch_key_value;
    cruise_resume_switch_key_value.key = "Cruise Control Resume Switch";
    cruise_resume_switch_key_value.value = std::to_string(cruise_resume_switch);

    const double cruise_set_speed = message->GetSignal("CruiseControlSetSpeed")->GetResult();
    diagnostic_msgs::msg::KeyValue cruise_set_speed_key_value;
    cruise_set_speed_key_value.key = "Cruise Control Set Speed [km/h]";
    cruise_set_speed_key_value.value = std::to_string(cruise_set_speed);

    const double cruise_set_switch = message->GetSignal("CruiseControlSetSwitch")->GetResult();
    diagnostic_msgs::msg::KeyValue cruise_set_switch_key_value;
    cruise_set_switch_key_value.key = "Cruise Control Set Switch";
    cruise_set_switch_key_value.value = std::to_string(cruise_set_switch);

    const double cruise_states = message->GetSignal("CruiseControlStates")->GetResult();
    diagnostic_msgs::msg::KeyValue cruise_states_key_value;
    cruise_states_key_value.key = "Cruise Control States";
    cruise_states_key_value.value = std::to_string(cruise_states);

    const double diag_switch = message->GetSignal("EngineDiagnosticTestModeSwitch")->GetResult();
    diagnostic_msgs::msg::KeyValue diag_switch_key_value;
    diag_switch_key_value.key = "Engine Diagnostic Test Mode Switch";
    diag_switch_key_value.value = std::to_string(diag_switch);

    const double idle_dec_switch = message->GetSignal("EngineIdleDecrementSwitch")->GetResult();
    diagnostic_msgs::msg::KeyValue idle_dec_switch_key_value;
    idle_dec_switch_key_value.key = "Engine Idle Decrement Switch Switch";
    idle_dec_switch_key_value.value = std::to_string(idle_dec_switch);

    const double idle_inc_switch = message->GetSignal("EngineIdleIncrementSwitch")->GetResult();
    diagnostic_msgs::msg::KeyValue idle_inc_switch_key_value;
    idle_inc_switch_key_value.key = "Engine Idle Increment Switch";
    idle_inc_switch_key_value.value = std::to_string(idle_inc_switch);

    const double shutdown_switch = message->GetSignal("EngineShutdownOverrideSwitch")->GetResult();
    diagnostic_msgs::msg::KeyValue shutdown_switch_key_value;
    shutdown_switch_key_value.key = "ENgine Shutdown Override Switch";
    shutdown_switch_key_value.value = std::to_string(shutdown_switch);

    const double park_inhibit = message->GetSignal("ParkBrakeReleaseInhibitRequest")->GetResult();
    diagnostic_msgs::msg::KeyValue park_inhibit_key_value;
    park_inhibit_key_value.key = "Park Brake Release Inhibit Request";
    park_inhibit_key_value.value = std::to_string(park_inhibit);

    const double PTO_governor = message->GetSignal("PtoGovernorState")->GetResult();
    diagnostic_msgs::msg::KeyValue PTO_governor_key_value;
    PTO_governor_key_value.key = "PTO Governor State";
    PTO_governor_key_value.value = std::to_string(PTO_governor);

    const double axle_switch = message->GetSignal("TwoSpeedAxleSwitch")->GetResult();
    diagnostic_msgs::msg::KeyValue axle_switch_key_value;
    axle_switch_key_value.key = "Two Speed Axel Switch";
    axle_switch_key_value.value = std::to_string(axle_switch);

    const double wheel_speed = message->GetSignal("WheelBasedVehicleSpeed")->GetResult();
    diagnostic_msgs::msg::KeyValue wheel_speed_key_value;
    wheel_speed_key_value.key = "Wheel-Based Vehicle Speed";
    wheel_speed_key_value.value = std::to_string(wheel_speed);

    CCVS1_status_.name = "CCVS1";
    CCVS1_status_.message = "Cruise Control/Vehicle Speed 1";
    CCVS1_status_.hardware_id = "Source Address: " + std::to_string(MSG->id & 0x000000FFu);
    CCVS1_status_.values = {
      brake_switch_key_value,
      clutch_switch_key_value,
      cruise_coast_switch_key_value,
      cruise_accel_switch_key_value,
      cruise_activate_switch_key_value,
      cruise_enable_switch_key_value,
      cruise_pause_switch_key_value,
      cruise_resume_switch_key_value,
      cruise_set_speed_key_value,
      cruise_set_switch_key_value,
      cruise_states_key_value,
      diag_switch_key_value,
      idle_dec_switch_key_value,
      idle_inc_switch_key_value,
      shutdown_switch_key_value,
      park_inhibit_key_value,
      PTO_governor_key_value,
      axle_switch_key_value,
      wheel_speed_key_value
    };

    CCVS1_.header.stamp = MSG->header.stamp;
    CCVS1_.header.frame_id = frame_id_;
    CCVS1_.status = {CCVS1_status_};

    // Just the Wheel Based Vehicle Speed
    double vehicle_wheel_speed = message->GetSignal("WheelBasedVehicleSpeed")->GetResult();
    vehicle_speed_.header.stamp = MSG->header.stamp;
    vehicle_speed_.header.frame_id = frame_id_;
    vehicle_speed_.twist.linear.x = vehicle_wheel_speed;

    pub_CCVS1_->publish(CCVS1_);
    pub_vehicle_speed_->publish(vehicle_speed_);
  }
}

void GenericCanDriverNode::recvCCVS5(const can_msgs::msg::Frame::SharedPtr MSG)
{
  NewEagle::DbcMessage * message = dbw_dbc_db.GetMessage("CCVS5"); // 2365322238
  // if minimum datalength is met
  if (MSG->dlc >= message->GetDlc())
  {
    message->SetFrame(MSG);

    directional_vehicle_speed_.header.stamp = MSG->header.stamp;
    directional_vehicle_speed_.header.frame_id = frame_id_;
    directional_vehicle_speed_.twist.linear.x = message->GetSignal("DirectionalVehicleSpeed")->GetResult();

    pub_directional_vehicle_speed_->publish(directional_vehicle_speed_);

    const double dir_vehicle_speed = message->GetSignal("DirectionalVehicleSpeed")->GetResult();
    diagnostic_msgs::msg::KeyValue dir_vehicle_speed_key_value;
    dir_vehicle_speed_key_value.key = "Directional Vehicle Speed";
    dir_vehicle_speed_key_value.value = std::to_string(dir_vehicle_speed);

    CCVS5_status_.level = 0;
    CCVS5_status_.name = "VF";
    CCVS5_status_.message = "Vehicle Fluids";
    CCVS5_status_.hardware_id = "Source Address: " + std::to_string(MSG->id & 0x000000FFu);
    CCVS5_status_.values = {
      dir_vehicle_speed_key_value
    };

    CCVS5_.header.stamp = MSG->header.stamp;
    CCVS5_.header.frame_id = frame_id_;
    CCVS5_.status = {CCVS5_status_};

    pub_CCVS5_->publish(CCVS5_); 
  }
}

void GenericCanDriverNode::recvDD1(const can_msgs::msg::Frame::SharedPtr MSG)
{
  NewEagle::DbcMessage * message = dbw_dbc_db.GetMessage("DD1"); // 2566847742
  // if minimum datalength is met
  if (MSG->dlc >= message->GetDlc())
  {
    message->SetFrame(MSG);

    const double cargo_temp = message->GetSignal("CargoAmbientTemperature")->GetResult();
    diagnostic_msgs::msg::KeyValue cargo_temp_key_value_;
    cargo_temp_key_value_.key = "Cargo Ambient Temperature [degC]";
    cargo_temp_key_value_.value = std::to_string(cargo_temp);

    const double fuel_filter_d_p = message->GetSignal("EngnFlFltrDffrntlPrssr")->GetResult();
    diagnostic_msgs::msg::KeyValue fuel_filter_d_p_key_value_;
    fuel_filter_d_p_key_value_.key = "Engine Fuel Filter Differential Pressure [kPa]";
    fuel_filter_d_p_key_value_.value = std::to_string(fuel_filter_d_p);

    const double oil_filter_d_p = message->GetSignal("EngnOlFltrDffrntlPrssr")->GetResult();
    diagnostic_msgs::msg::KeyValue oil_filter_d_p_key_value_;
    oil_filter_d_p_key_value_.key = "Engine Oil Filter Differential Pressure [kPa]";
    oil_filter_d_p_key_value_.value = std::to_string(oil_filter_d_p);

    const double oil_filter_dp_ext = message->GetSignal("EngnOlFltrDffrntlPrssrExtnddRng")->GetResult();
    diagnostic_msgs::msg::KeyValue oil_filter_dp_ext_key_value_;
    oil_filter_dp_ext_key_value_.key = "Engine Oil Filter Differential Pressure (Extended Range) [kPa]";
    oil_filter_dp_ext_key_value_.value = std::to_string(oil_filter_dp_ext);

    const double fuel_level_1 = message->GetSignal("FuelLevel1")->GetResult();
    diagnostic_msgs::msg::KeyValue fuel_level_1_key_value_;
    fuel_level_1_key_value_.key = "Fuel Level 1 [%]";
    fuel_level_1_key_value_.value = std::to_string(fuel_level_1);

    const double fuel_level_2 = message->GetSignal("FuelLevel2")->GetResult();
    diagnostic_msgs::msg::KeyValue fuel_level_2_key_value_;
    fuel_level_2_key_value_.key = "Fuel Level 2 [%]";
    fuel_level_2_key_value_.value = std::to_string(fuel_level_2);

    const double washer_fluid_level = message->GetSignal("WasherFluidLevel")->GetResult();
    diagnostic_msgs::msg::KeyValue washer_fluid_level_key_value_;
    fuel_level_1_key_value_.key = "Washer Fluid Level [%]";
    fuel_level_1_key_value_.value = std::to_string(washer_fluid_level);

    DD1_status_.level = 0;
    DD1_status_.name = "DD1";
    DD1_status_.message = "Dash Display 1";
    DD1_status_.hardware_id = "Source Address: " + std::to_string(MSG->id & 0x000000FFu);
    DD1_status_.values = {
      cargo_temp_key_value_,
      fuel_filter_d_p_key_value_,
      oil_filter_d_p_key_value_,
      oil_filter_dp_ext_key_value_,
      fuel_level_1_key_value_,
      fuel_level_2_key_value_,
      washer_fluid_level_key_value_
    };

    DD1_.header.stamp = MSG->header.stamp;
    DD1_.header.frame_id = frame_id_;
    DD1_.status = {DD1_status_};

    pub_DD1_->publish(DD1_);
  }
}

void GenericCanDriverNode::recvEEC1(const can_msgs::msg::Frame::SharedPtr MSG)
{
  NewEagle::DbcMessage * message = dbw_dbc_db.GetMessage("EEC1");  // 2364540158

  // if minimum datalength is met
  if (MSG->dlc >= message->GetDlc())
  {
    message->SetFrame(MSG);
    // add thing for actual engine percent torque. OR ya know, the entire message
    const double rpm = message->GetSignal("EngineSpeed")->GetResult();
    diagnostic_msgs::msg::KeyValue engine_speed_key_value; 
    engine_speed_key_value.key = "RPM";
    engine_speed_key_value.value = std::to_string(rpm);

    const double torque_percent = message->GetSignal("ActualEnginePercentTorque")->GetResult();
    diagnostic_msgs::msg::KeyValue torque_percent_key_value;
    torque_percent_key_value.key = "Torque [%]";
    torque_percent_key_value.value = std::to_string(torque_percent);

    EEC1_status_.level = 0;
    EEC1_status_.name = "EEC1";
    EEC1_status_.message = "Electronic Engine Controller 1";
    EEC1_status_.hardware_id = "Source Address: " + std::to_string(MSG->id & 0x000000FFu);
    EEC1_status_.values = {engine_speed_key_value, torque_percent_key_value};

    EEC1_.header.stamp = MSG->header.stamp;
    EEC1_.header.frame_id = frame_id_;
    EEC1_.status = {EEC1_status_};

    pub_EEC1_->publish(EEC1_);
  }
}

void GenericCanDriverNode::recvEEC2(const can_msgs::msg::Frame::SharedPtr MSG)
{
  NewEagle::DbcMessage * message = dbw_dbc_db.GetMessage("EEC2"); // 2364539902
  // if minimum datalength is met
  if (MSG->dlc >= message->GetDlc())
  {
    message->SetFrame(MSG);
    
    const double low_idle_1 = message->GetSignal("AcceleratorPedal1LowIdleSwitch")->GetResult();
    diagnostic_msgs::msg::KeyValue low_idle_1_key_value;
    low_idle_1_key_value.key = "Accelerator Pedal 1 Low Idle Switch";
    low_idle_1_key_value.value = std::to_string(low_idle_1);

    const double low_idle_2 = message->GetSignal("AcceleratorPedal2LowIdleSwitch")->GetResult();
    diagnostic_msgs::msg::KeyValue low_idle_2_key_value;
    low_idle_2_key_value.key = "Accelerator Pedal 2 Low Idle Switch";
    low_idle_2_key_value.value = std::to_string(low_idle_2);

    const double kickdown_switch = message->GetSignal("AcceleratorPedalKickdownSwitch")->GetResult();
    diagnostic_msgs::msg::KeyValue kickdown_switch_key_value;
    kickdown_switch_key_value.key = "Kickdown Switch";
    kickdown_switch_key_value.value = std::to_string(kickdown_switch);

    const double position_1 = message->GetSignal("AcceleratorPedalPosition1")->GetResult();
    diagnostic_msgs::msg::KeyValue position_1_key_value;
    position_1_key_value.key = "Accelerator Pedal 1 Position";
    position_1_key_value.value = std::to_string(position_1);

    const double position_2 = message->GetSignal("AcceleratorPedal2Position")->GetResult();
    diagnostic_msgs::msg::KeyValue position_2_key_value;
    position_2_key_value.key = "Accelerator Pedal 2 Position";
    position_2_key_value.value = std::to_string(position_2);

    const double speed_limit_status = message->GetSignal("RoadSpeedLimitStatus")->GetResult();
    diagnostic_msgs::msg::KeyValue speed_limit_status_key_value;
    speed_limit_status_key_value.key = "Road Speed Limit Status";
    speed_limit_status_key_value.value = std::to_string(speed_limit_status);

    const double remote_pedal_pos = message->GetSignal("RemoteAcceleratorPedalPosition")->GetResult();
    diagnostic_msgs::msg::KeyValue remote_pedal_pos_key_value;
    remote_pedal_pos_key_value.key = "Remote Pedal Position";
    remote_pedal_pos_key_value.value = std::to_string(remote_pedal_pos);

    const double engine_load = message->GetSignal("EnginePercentLoadAtCurrentSpeed")->GetResult();
    diagnostic_msgs::msg::KeyValue engine_load_key_value;
    engine_load_key_value.key = "Engine Load at Current Speed";
    engine_load_key_value.value = std::to_string(engine_load);

    EEC2_status_.level = 0;
    EEC2_status_.name = "EEC2";
    EEC2_status_.message = "Electronic Engine Controller 2";
    EEC2_status_.hardware_id = "Source Address: " + std::to_string(MSG->id & 0x000000FFu);
    EEC2_status_.values = {
      engine_load_key_value,
      position_1_key_value,
      low_idle_1_key_value,
      position_2_key_value,
      low_idle_2_key_value,
      kickdown_switch_key_value,
      speed_limit_status_key_value,
      remote_pedal_pos_key_value,
    };

    EEC2_.header.stamp = MSG->header.stamp;
    EEC2_.header.frame_id = frame_id_;
    EEC2_.status = {EEC2_status_};

    pub_EEC2_->publish(EEC2_);
  }
}

void GenericCanDriverNode::recvEEC3(const can_msgs::msg::Frame::SharedPtr MSG)
{
  NewEagle::DbcMessage * message = dbw_dbc_db.GetMessage("EEC3"); // 2364539902
  // if minimum datalength is met
  if (MSG->dlc >= message->GetDlc())
  {
    message->SetFrame(MSG);
    
    const double exhaust_dew_pt_1 = message->GetSignal("Aftertreatment1ExhaustDewPoint")->GetResult();
    diagnostic_msgs::msg::KeyValue exhaust_dew_pt_1_key_value;
    exhaust_dew_pt_1_key_value.key = "Aftertreatment 1 Exhaust Dew Point";
    exhaust_dew_pt_1_key_value.value = std::to_string(exhaust_dew_pt_1);

    const double exhaust_dew_pt_2 = message->GetSignal("Aftertreatment1IntakeDewPoint")->GetResult();
    diagnostic_msgs::msg::KeyValue exhaust_dew_pt_2_key_value;
    exhaust_dew_pt_2_key_value.key = "Aftertreatment 2 Exhaust Dew Point";
    exhaust_dew_pt_2_key_value.value = std::to_string(exhaust_dew_pt_2);

    const double intake_dew_pt_1 = message->GetSignal("Aftertreatment2ExhaustDewPoint")->GetResult();
    diagnostic_msgs::msg::KeyValue intake_dew_pt_1_key_value;
    intake_dew_pt_1_key_value.key = "Aftertreatment 1 Intake Dew Point";
    intake_dew_pt_1_key_value.value = std::to_string(intake_dew_pt_1);

    const double intake_dew_pt_2 = message->GetSignal("Aftertreatment2IntakeDewPoint")->GetResult();
    diagnostic_msgs::msg::KeyValue intake_dew_pt_2_key_value;
    intake_dew_pt_2_key_value.key = "Aftertreatment 2 Intake Dew Point";
    intake_dew_pt_2_key_value.value = std::to_string(intake_dew_pt_2);

    const double exhaust_mass_flow_rate = message->GetSignal("Aftrtrtmnt1ExhstGsMssFlwRt")->GetResult();
    diagnostic_msgs::msg::KeyValue exhaust_mass_flow_rate_key_value;
    exhaust_mass_flow_rate_key_value.key = "Aftertreament 1 Exhaust Mass Flow Rate [kg/h]";
    exhaust_mass_flow_rate_key_value.value = std::to_string(exhaust_mass_flow_rate);

    const double engine_desired_op_speed = message->GetSignal("EngineSDesiredOperatingSpeed")->GetResult();
    diagnostic_msgs::msg::KeyValue engine_desired_op_speed_key_value;
    engine_desired_op_speed_key_value.key = "Engine's Desired Operating Speed [rpm]";
    engine_desired_op_speed_key_value.value = std::to_string(engine_desired_op_speed);

    const double ESDsdOptSpdAstAdstt = message->GetSignal("ESDsdOptSpdAstAdstt")->GetResult();
    diagnostic_msgs::msg::KeyValue ESDsdOptSpdAstAdstt_key_value;
    ESDsdOptSpdAstAdstt_key_value.key = "Engine's Desired Operating Speed Asymmetry Adjustment";
    ESDsdOptSpdAstAdstt_key_value.value = std::to_string(ESDsdOptSpdAstAdstt);

    const double estimated_loss_percent_torque = message->GetSignal("EstmtdEngnPrstLsssPrntTrq")->GetResult();
    diagnostic_msgs::msg::KeyValue estimated_loss_percent_torque_key_value;
    estimated_loss_percent_torque_key_value.key = "Estimated Engine Parasitic Losses - Percent Torque [%]";
    estimated_loss_percent_torque_key_value.value = std::to_string(estimated_loss_percent_torque);

    const double nominal_friction_percent_torque = message->GetSignal("NominalFrictionPercentTorque")->GetResult();
    diagnostic_msgs::msg::KeyValue nominal_friction_percent_trq_key_value;
    nominal_friction_percent_trq_key_value.key = "Nominal Friction - Percent Torque [%]";
    nominal_friction_percent_trq_key_value.value = std::to_string(nominal_friction_percent_torque);

    EEC3_status_.level = 0;
    EEC3_status_.name = "EEC3";
    EEC3_status_.message = "Electronic Engine Controller 3";
    EEC3_status_.hardware_id = "Source Address: " + std::to_string(MSG->id & 0x000000FFu);
    EEC3_status_.values = {
      exhaust_dew_pt_1_key_value,
      exhaust_dew_pt_2_key_value,
      intake_dew_pt_1_key_value,
      intake_dew_pt_2_key_value,
      exhaust_mass_flow_rate_key_value,
      engine_desired_op_speed_key_value,
      ESDsdOptSpdAstAdstt_key_value,
      estimated_loss_percent_torque_key_value,
      nominal_friction_percent_trq_key_value,
    };

    EEC3_.header.stamp = MSG->header.stamp;
    EEC3_.header.frame_id = frame_id_;
    EEC3_.status = {EEC3_status_};

    pub_EEC3_->publish(EEC3_);
  }
}

void GenericCanDriverNode::recvEOI(const can_msgs::msg::Frame::SharedPtr MSG)
{
  NewEagle::DbcMessage * message = dbw_dbc_db.GetMessage("EOI"); // 2365428478
  // if minimum datalength is met
  if (MSG->dlc >= message->GetDlc())
  {
    message->SetFrame(MSG);

    const double shutdown = message->GetSignal("EngineControlledShutdownRequest")->GetResult();
    diagnostic_msgs::msg::KeyValue shutdown_key_value;
    shutdown_key_value.key = "Engine Controllet Shutdown Request";
    shutdown_key_value.value = std::to_string(shutdown);

    const double pump_control = message->GetSignal("EngineCoolantPumpControl")->GetResult();
    diagnostic_msgs::msg::KeyValue pump_control_key_value;
    pump_control_key_value.key = "Engine Coolant Pump Control";
    pump_control_key_value.value = std::to_string(pump_control);

    const double derate = message->GetSignal("EngineDerateRequest")->GetResult();
    diagnostic_msgs::msg::KeyValue derate_key_value;
    derate_key_value.key = "Engine Derate Request [%]";
    derate_key_value.value = std::to_string(derate);
    
    const double torque = message->GetSignal("EngineDesiredTorqueRequest")->GetResult();
    diagnostic_msgs::msg::KeyValue torque_key_value;
    torque_key_value.key = "Engine Desired Torque Request";
    torque_key_value.value = std::to_string(torque);

    const double fuel_shutoff_1 = message->GetSignal("EngineFuelShutoff1Control")->GetResult();
    diagnostic_msgs::msg::KeyValue fuel_shutoff_1_key_value;
    fuel_shutoff_1_key_value.key = "Engine Fuel Shutoff 1 Control";
    fuel_shutoff_1_key_value.value = std::to_string(fuel_shutoff_1);
    
    const double fuel_shutoff_2 = message->GetSignal("EngineFuelShutoff2Control")->GetResult();
    diagnostic_msgs::msg::KeyValue fuel_shutoff_2_key_value;
    fuel_shutoff_2_key_value.key = "Engine Fuel Shutoff 2 Control";
    fuel_shutoff_2_key_value.value = std::to_string(fuel_shutoff_2);

    const double shutoff_vent = message->GetSignal("EngineFuelShutoffVentControl")->GetResult();
    diagnostic_msgs::msg::KeyValue shutoff_vent_key_value;
    shutoff_vent_key_value.key = "Engine Fuel Shutoff Vent Control";
    shutoff_vent_key_value.value = std::to_string(shutoff_vent);
    
    const double oil_preheater = message->GetSignal("EngineOilPreHeaterControl")->GetResult();
    diagnostic_msgs::msg::KeyValue oil_preheater_key_value;
    oil_preheater_key_value.key = "Engine Oil Preheater Control";
    oil_preheater_key_value.value = std::to_string(oil_preheater);
    
    const double oil_priming_pump = message->GetSignal("EngineOilPrimingPumpControl")->GetResult();
    diagnostic_msgs::msg::KeyValue oil_priming_pump_key_value;
    oil_priming_pump_key_value.key = "Engine Oil Priming Pump Control";
    oil_priming_pump_key_value.value = std::to_string(oil_priming_pump);
    
    const double operating_state = message->GetSignal("EngineOperatingState")->GetResult();
    diagnostic_msgs::msg::KeyValue operating_state_key_value;
    operating_state_key_value.key = "Engine Operating State";
    operating_state_key_value.value = std::to_string(operating_state);
    
    const double preheater_control = message->GetSignal("EnginePreHeaterControl")->GetResult();
    diagnostic_msgs::msg::KeyValue preheater_control_key_value;
    preheater_control_key_value.key = "Engine Preheater Control";
    preheater_control_key_value.value = std::to_string(preheater_control);
        
    const double starter_relay = message->GetSignal("EngineStarterMotorRelayControl")->GetResult();
    diagnostic_msgs::msg::KeyValue starter_relay_key_value;
    starter_relay_key_value.key = "Engine Starter Motor Relay Control";
    starter_relay_key_value.value = std::to_string(starter_relay);
        
    const double cold_idle_status = message->GetSignal("EngnCldAmntElvtdIdlStts")->GetResult();
    diagnostic_msgs::msg::KeyValue cold_idle_status_key_value;
    cold_idle_status_key_value.key = "Engine Cold Ambient Elevated Idle Status";
    cold_idle_status_key_value.value = std::to_string(cold_idle_status);
        
    const double electrical_sys = message->GetSignal("EngnEltrlSstmPwrCnsrvtnCntrl")->GetResult();
    diagnostic_msgs::msg::KeyValue electrical_sys_key_value;
    electrical_sys_key_value.key = "Engine Electrical System Power Conservation Control";
    electrical_sys_key_value.value = std::to_string(electrical_sys);
            
    const double emergency_shutdown = message->GetSignal("EngnEmrgnImmdtShtdwnIndtn")->GetResult();
    diagnostic_msgs::msg::KeyValue emergency_shutdown_key_value;
    emergency_shutdown_key_value.key = "Engine Emergency (Immediate) Shutdown Indication";
    emergency_shutdown_key_value.value = std::to_string(emergency_shutdown);
            
    const double shutoff_valve_test = message->GetSignal("EngnFlShtffVlvLkTstCntrl")->GetResult();
    diagnostic_msgs::msg::KeyValue shutoff_valve_test_key_value;
    shutoff_valve_test_key_value.key = "Engine Fuel Shutoff Valve Leak Test Control";
    shutoff_valve_test_key_value.value = std::to_string(shutoff_valve_test);
            
    const double fuel_pump_primer = message->GetSignal("FuelPumpPrimerControl")->GetResult();
    diagnostic_msgs::msg::KeyValue fuel_pump_primer_key_value;
    fuel_pump_primer_key_value.key = "Fuel Pump Primer Control";
    fuel_pump_primer_key_value.value = std::to_string(fuel_pump_primer);
            
    const double time_remaining = message->GetSignal("TmRmnngInEngnOprtngStt")->GetResult();
    diagnostic_msgs::msg::KeyValue time_remaining_key_value;
    time_remaining_key_value.key = "Time Remaining in Engine Operating State [s]";
    time_remaining_key_value.value = std::to_string(time_remaining);

    EOI_status_.level = 0;
    EOI_status_.name = "EOI";
    EOI_status_.message = "Engine Operating Information";
    EOI_status_.hardware_id = "Source Address: " + std::to_string(MSG->id & 0x000000FFu);
    EOI_status_.values = {
      shutdown_key_value,
      pump_control_key_value,
      derate_key_value,
      torque_key_value,
      fuel_shutoff_1_key_value,
      fuel_shutoff_2_key_value,
      shutoff_vent_key_value,
      oil_preheater_key_value,
      oil_priming_pump_key_value,
      operating_state_key_value,
      preheater_control_key_value,
      starter_relay_key_value,
      cold_idle_status_key_value,
      electrical_sys_key_value,
      emergency_shutdown_key_value,
      shutoff_valve_test_key_value,
      fuel_pump_primer_key_value,
      time_remaining_key_value,
    };

    EOI_.header.stamp = MSG->header.stamp;
    EOI_.header.frame_id = frame_id_;
    EOI_.status = {EOI_status_};

    pub_EOI_->publish(EOI_);
  }
}

void GenericCanDriverNode::recvET1(const can_msgs::msg::Frame::SharedPtr MSG)
{
  NewEagle::DbcMessage * message = dbw_dbc_db.GetMessage("ET1"); // 2566844158
  // if minimum datalength is met
  if (MSG->dlc >= message->GetDlc())
  {
    message->SetFrame(MSG);

    engine_coolant_temperature_.header.stamp = MSG->header.stamp;
    engine_coolant_temperature_.header.frame_id = frame_id_;
    engine_coolant_temperature_.temperature =
      message->GetSignal("EngineCoolantTemperature")->GetResult();
    
    pub_engine_coolant_temp_->publish(engine_coolant_temperature_);

    const double coolant_temp = message->GetSignal("EngineCoolantTemperature")->GetResult();
    diagnostic_msgs::msg::KeyValue coolant_temp_key_value;
    coolant_temp_key_value.key = "Engine Coolant Temperature [degC]";
    coolant_temp_key_value.value = std::to_string(coolant_temp);

    const double fuel_temp_1 = message->GetSignal("EngineFuel1Temperature1")->GetResult();
    diagnostic_msgs::msg::KeyValue fuel_temp_1_key_value;
    fuel_temp_1_key_value.key = "Engine Fuel Temperature 1 [degC]";
    fuel_temp_1_key_value.value = std::to_string(fuel_temp_1);

    const double intercooler_temp = message->GetSignal("EngineIntercoolerTemperature")->GetResult();
    diagnostic_msgs::msg::KeyValue intercooler_temp_key_value;
    intercooler_temp_key_value.key = "Engine Intercooler Temperature [degC]";
    intercooler_temp_key_value.value = std::to_string(intercooler_temp);

    const double oil_temp_1 = message->GetSignal("EngineOilTemperature1")->GetResult();
    diagnostic_msgs::msg::KeyValue oil_temp_1_key_value;
    oil_temp_1_key_value.key = "Engine Oil Temperature 1 [degC]";
    oil_temp_1_key_value.value = std::to_string(oil_temp_1);

    const double charge_air_opening = message->GetSignal("EngnChrgArClrThrmsttOpnng")->GetResult();
    diagnostic_msgs::msg::KeyValue charge_air_opening_key_value;
    charge_air_opening_key_value.key = "Engine Charge Air Cooler Thermostat Opening [%]";
    charge_air_opening_key_value.value = std::to_string(charge_air_opening);

    const double turbo_oil_temp = message->GetSignal("EngnTrhrgr1OlTmprtr")->GetResult();
    diagnostic_msgs::msg::KeyValue turbo_oil_temp_key_value;
    turbo_oil_temp_key_value.key = "Engine Turbocharger 1 Oil Temperature [degC]";
    turbo_oil_temp_key_value.value = std::to_string(turbo_oil_temp);

    ET1_status_.level = 0;
    ET1_status_.name = "ET1";
    ET1_status_.message = "Engine Temperature 1";
    ET1_status_.hardware_id = "Source Address: " + std::to_string(MSG->id & 0x000000FFu);
    ET1_status_.values = {
      coolant_temp_key_value,
      fuel_temp_1_key_value,
      intercooler_temp_key_value,
      oil_temp_1_key_value,
      charge_air_opening_key_value,
      turbo_oil_temp_key_value,
    };

    ET1_.header.stamp = MSG->header.stamp;
    ET1_.header.frame_id = frame_id_;
    ET1_.status = {ET1_status_};

    pub_ET1_->publish(ET1_);   
  }
}

void GenericCanDriverNode::recvETC2(const can_msgs::msg::Frame::SharedPtr MSG)
{
  NewEagle::DbcMessage * message = dbw_dbc_db.GetMessage("ETC2"); // 2565867006
  // if minimum datalength is met
  if (MSG->dlc >= message->GetDlc())
  {
    message->SetFrame(MSG);

    const double gear_ratio = message->GetSignal("TransmissionActualGearRatio")->GetResult();
    diagnostic_msgs::msg::KeyValue gear_ratio_key_value_;
    gear_ratio_key_value_.key = "Transmission Actual Gear Ratio";
    gear_ratio_key_value_.value = std::to_string(gear_ratio);

    const double current_gear = message->GetSignal("TransmissionCurrentGear")->GetResult();
    diagnostic_msgs::msg::KeyValue current_gear_key_value_;
    current_gear_key_value_.key = "Transmission Current Gear";
    current_gear_key_value_.value = std::to_string(current_gear);
    
    const double current_range = message->GetSignal("TransmissionCurrentRange")->GetResult();
    diagnostic_msgs::msg::KeyValue current_range_key_value_;
    current_range_key_value_.key = "Transmission Current Range";
    current_range_key_value_.value = std::to_string(current_range);
    
    const double requested_range = message->GetSignal("TransmissionRequestedRange")->GetResult();
    diagnostic_msgs::msg::KeyValue requested_range_key_value_;
    requested_range_key_value_.key = "Transmission Requested Range";
    requested_range_key_value_.value = std::to_string(requested_range);
    
    const double selected_gear = message->GetSignal("TransmissionSelectedGear")->GetResult();
    diagnostic_msgs::msg::KeyValue selected_gear_key_value_;
    selected_gear_key_value_.key = "Transmission Selected Gear";
    selected_gear_key_value_.value = std::to_string(selected_gear);
    
    ETC2_status_.level = 0;
    ETC2_status_.name = "ETC2";
    ETC2_status_.message = "Electronic Transmission Controller 2";
    ETC2_status_.hardware_id = "Source Address: " + std::to_string(MSG->id & 0x000000FFu);
    ETC2_status_.values = {
      gear_ratio_key_value_,
      current_gear_key_value_,
      current_range_key_value_,
      requested_range_key_value_,
      selected_gear_key_value_
    };

    ETC2_.header.stamp = MSG->header.stamp;
    ETC2_.header.frame_id = frame_id_;
    ETC2_.status = {ETC2_status_};

    pub_ETC2_->publish(ETC2_);
  }
}

void GenericCanDriverNode::recvLBC(const can_msgs::msg::Frame::SharedPtr MSG)
{
  NewEagle::DbcMessage * message = dbw_dbc_db.GetMessage("LBC"); // 2364539902
  // if minimum datalength is met
  if (MSG->dlc >= message->GetDlc())
  {
    message->SetFrame(MSG);
    
    const double blade_control_mode = message->GetSignal("BladeControlMode")->GetResult();
    diagnostic_msgs::msg::KeyValue blade_control_mode_key_value;
    blade_control_mode_key_value.key = "Blade Control Mode";
    if(blade_control_mode == 0.)
    {
      blade_control_mode_key_value.value = "Manual Mode";
    }
    if(blade_control_mode == 1.)
    {
      blade_control_mode_key_value.value = "Automatic Mode";
    }
    if(blade_control_mode == 2.)
    {
      blade_control_mode_key_value.value = "Inactive Automatic Mode";
    }
    else
    {
      blade_control_mode_key_value.value = "UNDEFINED: " + std::to_string(blade_control_mode);
    }

    const double blade_control_mode_left = message->GetSignal("BladeControlModeLeft")->GetResult();
    diagnostic_msgs::msg::KeyValue blade_control_mode_left_key_value;
    blade_control_mode_left_key_value.key = "Blade Control Mode Left";
    if(blade_control_mode_left == 0.)
    {
      blade_control_mode_left_key_value.value = "Manual Mode";
    }
    if(blade_control_mode_left == 1.)
    {
      blade_control_mode_left_key_value.value = "Automatic Mode";
    }
    if(blade_control_mode_left == 2.)
    {
      blade_control_mode_left_key_value.value = "Inactive Automatic Mode";
    }
    else
    {
      blade_control_mode_left_key_value.value = "UNDEFINED: " + std::to_string(blade_control_mode_left);
    }

    const double blade_control_mode_right = message->GetSignal("BladeControlModeRight")->GetResult();
    diagnostic_msgs::msg::KeyValue blade_control_mode_right_key_value;
    blade_control_mode_right_key_value.key = "Blade Control Mode Right";
    if(blade_control_mode_right == 0.)
    {
      blade_control_mode_right_key_value.value = "Manual Mode";
    }
    if(blade_control_mode_right == 1.)
    {
      blade_control_mode_right_key_value.value = "Automatic Mode";
    }
    if(blade_control_mode_right == 2.)
    {
      blade_control_mode_right_key_value.value = "Inactive Automatic Mode";
    }
    else
    {
      blade_control_mode_right_key_value.value = "UNDEFINED: " + std::to_string(blade_control_mode_right);
    }

    const double blade_control_duration = message->GetSignal("BladeDurationAndDirection")->GetResult();
    diagnostic_msgs::msg::KeyValue blade_control_duration_key_value;
    blade_control_duration_key_value.key = "Blade Control Direction and Duration";    
    blade_control_duration_key_value.value = std::to_string(blade_control_duration);

    const double land_leveling_status = message->GetSignal("LandLevelingSystemEnableStatus")->GetResult();
    diagnostic_msgs::msg::KeyValue land_leveling_status_key_value;
    land_leveling_status_key_value.key = "Land Leveling System Enable Status";    
    land_leveling_status_key_value.value = std::to_string(land_leveling_status);
 
    LBC_status_.level = 0;
    LBC_status_.name = "CTL Blade";
    LBC_status_.message = "Blade Lift Control";
    LBC_status_.hardware_id = "Source Address: " + std::to_string(MSG->id & 0x000000FFu);
    LBC_status_.values = {
      blade_control_mode_key_value,
      blade_control_mode_left_key_value,
      blade_control_mode_right_key_value,
      blade_control_duration_key_value,
      land_leveling_status_key_value,
    };

    LBC_.header.stamp = MSG->header.stamp;
    LBC_.header.frame_id = frame_id_;
    LBC_.status = {LBC_status_};

    pub_LBC_->publish(LBC_);
  }
}

void GenericCanDriverNode::recvTRF2(const can_msgs::msg::Frame::SharedPtr MSG)
{
  NewEagle::DbcMessage * message = dbw_dbc_db.GetMessage("TRF2"); // 2566755838
  // if minimum datalength is met
  if (MSG->dlc >= message->GetDlc())
  {
    message->SetFrame(MSG);

    // pub just the temperature in a temperature message
    transmission_oil_temperature_.header.stamp = MSG->header.stamp;
    transmission_oil_temperature_.header.frame_id = frame_id_;
    transmission_oil_temperature_.temperature = 
      message->GetSignal("TransmissionOilTemperature2")->GetResult();
    pub_transmission_oil_temp_->publish(transmission_oil_temperature_);

    torque_converter_oil_temp_.header.stamp = MSG->header.stamp;
    torque_converter_oil_temp_.header.frame_id = frame_id_;
    torque_converter_oil_temp_.temperature = 
      message->GetSignal("TrnsmssnTrqCnvrtrOlOtltTmprtr")->GetResult();
    pub_torque_converter_oil_temp_->publish(torque_converter_oil_temp_);
    // pub the entire diagnostic message
    
    const double oil_level = message->GetSignal("TransmissionOilLevel2HighLow")->GetResult();
    diagnostic_msgs::msg::KeyValue oil_level_key_value;
    oil_level_key_value.key = "Transmission Oil Level 2 High / Low [l]";
    oil_level_key_value.value = std::to_string(oil_level);
    
    const double oil_level_switch = message->GetSignal("TransmissionOilLevelSwitch")->GetResult();
    diagnostic_msgs::msg::KeyValue oil_level_switch_key_value;
    oil_level_switch_key_value.key = "Transmission Oil Level Switch";
    oil_level_switch_key_value.value = std::to_string(oil_level_switch);
    
    const double oil_life = message->GetSignal("TransmissionOilLifeRemaining")->GetResult();
    diagnostic_msgs::msg::KeyValue oil_life_key_value;
    oil_life_key_value.key = "Transmission Oil Life Remaining [%]";
    oil_life_key_value.value = std::to_string(oil_life);
    
    const double oil_temp = message->GetSignal("TransmissionOilTemperature2")->GetResult();
    diagnostic_msgs::msg::KeyValue oil_temp_key_value;
    oil_temp_key_value.key = "Transmission Oil Temperature 2";
    oil_temp_key_value.value = std::to_string(oil_temp);
    
    const double overheat = message->GetSignal("TransmissionOverheatIndicator")->GetResult();
    diagnostic_msgs::msg::KeyValue overheat_key_value;
    overheat_key_value.key = "Transmission Overheat Indicator";
    overheat_key_value.value = std::to_string(overheat);
    
    const double restriction_switch = message->GetSignal("TrnsmssnOlFltrRstrtnSwth")->GetResult();
    diagnostic_msgs::msg::KeyValue restriction_switch_key_value;
    restriction_switch_key_value.key = "Transmission Oil Filter Restriction Switch";
    restriction_switch_key_value.value = std::to_string(restriction_switch);
    
    const double oil_level_countdown = message->GetSignal("TrnsmssnOlLvl2CntdwnTmr")->GetResult();
    diagnostic_msgs::msg::KeyValue oil_level_countdown_key_value;
    oil_level_countdown_key_value.key = "Transmission Oil Level 2 Countdown Timer";
    oil_level_countdown_key_value.value = std::to_string(oil_level_countdown);
    
    const double oil_measurement = message->GetSignal("TrnsmssnOlLvl2MsrmntStts")->GetResult();
    diagnostic_msgs::msg::KeyValue oil_measurement_key_value;
    oil_measurement_key_value.key = "Transmission Oil Level 2 Measurement Status";
    oil_measurement_key_value.value = std::to_string(oil_measurement);
    
    const double torque_oil_temp = message->GetSignal("TrnsmssnTrqCnvrtrOlOtltTmprtr")->GetResult();
    diagnostic_msgs::msg::KeyValue torque_oil_temp_key_value;
    torque_oil_temp_key_value.key = "Transmission Torque Converter Oil Outlet Temperature [degC]";
    torque_oil_temp_key_value.value = std::to_string(torque_oil_temp);

    TRF2_status_.level = 0;
    TRF2_status_.name = "TRF2";
    TRF2_status_.message = "Transmission Fluids 2";
    TRF2_status_.hardware_id = "Source Address: " + std::to_string(MSG->id & 0x000000FFu);
    TRF2_status_.values = {
      oil_level_key_value,
      oil_level_switch_key_value,
      oil_life_key_value,
      oil_temp_key_value,
      overheat_key_value,
      restriction_switch_key_value,
      oil_level_countdown_key_value,
      oil_measurement_key_value,
      torque_oil_temp_key_value,
    };

    TRF2_.header.stamp = MSG->header.stamp;
    TRF2_.header.frame_id = frame_id_;
    TRF2_.status = {TRF2_status_};

    pub_TRF2_->publish(TRF2_);    
  }
}

void GenericCanDriverNode::recvVEP1(const can_msgs::msg::Frame::SharedPtr MSG)
{
  NewEagle::DbcMessage * message = dbw_dbc_db.GetMessage("VEP1"); // 2566809854
  // if minimum datalength is met
  if (MSG->dlc >= message->GetDlc())
  {
    message->SetFrame(MSG);

    const double alternator_current = message->GetSignal("AlternatorCurrent")->GetResult();
    diagnostic_msgs::msg::KeyValue alternator_current_key_value;
    alternator_current_key_value.key = "Alternator Current [A]";
    alternator_current_key_value.value = std::to_string(alternator_current);
    
    const double battery_potential = message->GetSignal("BatteryPotentialPowerInput1")->GetResult();
    diagnostic_msgs::msg::KeyValue battery_potential_key_value;
    battery_potential_key_value.key = "Battery Potential / Power Input 1 [V]";
    battery_potential_key_value.value = std::to_string(battery_potential);
    
    const double system_voltage = message->GetSignal("ChargingSystemPotentialVoltage")->GetResult();
    diagnostic_msgs::msg::KeyValue system_voltage_key_value;
    system_voltage_key_value.key = "Charging System Potential (Voltage) [V]";
    system_voltage_key_value.value = std::to_string(system_voltage);
    
    const double key_voltage = message->GetSignal("KeySwitchBatteryPotential")->GetResult();
    diagnostic_msgs::msg::KeyValue key_voltage_key_value;
    key_voltage_key_value.key = "Key Switch Battery Potential [V]";
    key_voltage_key_value.value = std::to_string(key_voltage);
    
    const double SLI_current = message->GetSignal("SliBattery1NetCurrent")->GetResult();
    diagnostic_msgs::msg::KeyValue SLI_current_key_value;
    SLI_current_key_value.key = "SLI Battery 1 Net Current [A]";
    SLI_current_key_value.value = std::to_string(SLI_current);
    
    VEP1_status_.level = 0;
    VEP1_status_.name = "VEP1";
    VEP1_status_.message = "Vehicle Electrical Power 1";
    VEP1_status_.hardware_id = "Source Address: " + std::to_string(MSG->id & 0x000000FFu);
    VEP1_status_.values = {
      alternator_current_key_value,
      battery_potential_key_value,
      system_voltage_key_value,
      key_voltage_key_value,
      SLI_current_key_value
    };

    VEP1_.header.stamp = MSG->header.stamp;
    VEP1_.header.frame_id = frame_id_;
    VEP1_.status = {VEP1_status_};

    pub_VEP1_->publish(VEP1_);   
  }
}

void GenericCanDriverNode::recvVF(const can_msgs::msg::Frame::SharedPtr MSG)
{
  NewEagle::DbcMessage * message = dbw_dbc_db.GetMessage("VF"); // 2566809854
  // if minimum datalength is met
  if (MSG->dlc >= message->GetDlc())
  {
    message->SetFrame(MSG);
    // publish just the temp in a temp message
    hydraulic_oil_temperature_.header.stamp = MSG->header.stamp;
    hydraulic_oil_temperature_.header.frame_id = frame_id_;
    hydraulic_oil_temperature_.temperature =
      message->GetSignal("HydraulicTemperature")->GetResult();
    pub_hydraulic_temp_->publish(hydraulic_oil_temperature_);

    const double oil_filter_restriction = message->GetSignal("HdrlOlFltrRstrtnSwth")->GetResult();
    diagnostic_msgs::msg::KeyValue oil_filter_restriction_key_value;
    oil_filter_restriction_key_value.key = "Hydraulic Oil Filter Restriction Switch";
    oil_filter_restriction_key_value.value = std::to_string(oil_filter_restriction);

    const double oil_level = message->GetSignal("HydraulicOilLevel")->GetResult();
    diagnostic_msgs::msg::KeyValue oil_level_key_value;
    oil_level_key_value.key = "Hydraulic Oil Level [%]";
    oil_level_key_value.value = std::to_string(oil_level);

    const double hydraulic_temp = message->GetSignal("HydraulicTemperature")->GetResult();
    diagnostic_msgs::msg::KeyValue hydraulic_temp_key_value;
    hydraulic_temp_key_value.key = "Hydraulic Fluid Temperature [degC]";
    hydraulic_temp_key_value.value = std::to_string(hydraulic_temp);

    const double winch_oil_p_switch = message->GetSignal("WinchOilPressureSwitch")->GetResult();
    diagnostic_msgs::msg::KeyValue winch_oil_p_switch_key_value;
    winch_oil_p_switch_key_value.key = "Winch Oil Pressure Switch";
    winch_oil_p_switch_key_value.value = std::to_string(winch_oil_p_switch);

    VF_status_.level = 0;
    VF_status_.name = "VF";
    VF_status_.message = "Vehicle Fluids";
    VF_status_.hardware_id = "Source Address: " + std::to_string(MSG->id & 0x000000FFu);
    VF_status_.values = {
      oil_filter_restriction_key_value,
      oil_level_key_value,
      hydraulic_temp_key_value,
      winch_oil_p_switch_key_value
    };

    VF_.header.stamp = MSG->header.stamp;
    VF_.header.frame_id = frame_id_;
    VF_.status = {VF_status_};

    pub_VF_->publish(VF_);   
  }
}

}  // namespace generic_can_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(generic_can_driver::GenericCanDriverNode)
