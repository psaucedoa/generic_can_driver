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

#ifndef GENERIC_CAN_DRIVER__GENERIC_CAN_DRIVER_NODE_HPP_
#define GENERIC_CAN_DRIVER__GENERIC_CAN_DRIVER_NODE_HPP_

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
#include "generic_can_driver/visibility_control.hpp"

using LifecycleNodeInterface = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
namespace rlc = rclcpp_lifecycle;

namespace generic_can_driver
{

class GenericCanDriverNode : public rlc::LifecycleNode
{
public:
  explicit GenericCanDriverNode(const rclcpp::NodeOptions & OPTIONS);

  ~GenericCanDriverNode();

  // Incoming CAN Frame IDs in hex
  // These are named after standard J1939 can ID message names
  // Searcheable in the j1939 standard
  enum CANIDs
  {
    // make sure to make the last byte 00 ALWAYS
    ID_AMB      = 0x00FEF500u,
    ID_AT1T1I1  = 0x00FE5600u,
    ID_AUXIO1   = 0x00FED900u,
    ID_AUXIO2   = 0x00A70000u,
    ID_AUXIO6   = 0x009D0000u,
    ID_BJM1     = 0x00FDD600u,
    ID_BJM2     = 0x00FDD800u,
    ID_CCVS1    = 0x00FEF100u,
    ID_CCVS5    = 0x00FBF300u,
    ID_DD1      = 0x00FEFC00u,
    ID_DM1      = 0x00FECA00u,
    ID_DPFC1    = 0x00FD7C00u,
    ID_EEC1     = 0x00F00400u,
    ID_EEC2     = 0x00F00300u,
    ID_EEC3     = 0x00FEDF00u,
    ID_EJM1     = 0x00FDD700u,
    ID_EJM2     = 0x00FDD900u,
    ID_EJM3     = 0x00FDDB00u,
    ID_EJM4     = 0x00FD2F00u,
    ID_EJM5     = 0x00FD2D00u,
    ID_EJM6     = 0x00FD2B00u,
    ID_EOI      = 0x00FD9200u,
    ID_ET1      = 0x00FEEE00u,
    ID_ETC2     = 0x00F00500u,
    ID_HOURS    = 0x00FEE500u,
    ID_HEART    = 0x009D2700u,
    ID_LBC      = 0x00FE7200u,
    ID_LFE1     = 0x00FEF200u,
    ID_OI       = 0x00FEFF00u,
    ID_SHUTDN   = 0x00FEE400u,
    ID_TPCM_A3  = 0x00ECA300u,  // gotta think about filtering sources at some point
    ID_TPCM_FF  = 0x00ECFF00u,
    ID_TPDT_A3  = 0x00EBA300u,
    ID_TPDT_FF  = 0x00EBFF00u,
    ID_TRF2     = 0x00FD9500u,
    ID_VF       = 0x00FE6800u,
    ID_VEP1     = 0x00FEF700u,
  };

  LifecycleNodeInterface::CallbackReturn on_configure(const rlc::State & state) override;
  LifecycleNodeInterface::CallbackReturn on_activate(const rlc::State & state) override;
  LifecycleNodeInterface::CallbackReturn on_deactivate(const rlc::State & state) override;
  LifecycleNodeInterface::CallbackReturn on_cleanup(const rlc::State & state) override;
  LifecycleNodeInterface::CallbackReturn on_shutdown(const rlc::State & state) override;

  void componentTimerCallback();

  /** 
   * @brief Generic function that receives incoming ros2 CAN frames and sends it to the correct
   *         receiving function. Such as -> catches incoming imu msg, sends to rxIMURpt()
   * @param[in] MSG The incoming CAN frame
  */
  void rxFrame(const can_msgs::msg::Frame::SharedPtr MSG);

  /** 
   * @brief Generic function that publishes ros2 CAN frames
   *
  */
  void txFrame(const can_msgs::msg::Frame MSG);

  /**
   * @brief changes the source address of the device given its name and desired source address
  */
  void txRename(const std::array<uint8_t, 8UL> name, const uint8_t new_source_address);

  /** 
   * @brief Convert a Microstrain IMU report received over CAN into a ROS message.
   * @param[in] MSG The message received over CAN.
  */
  void rxGenericRpt(const can_msgs::msg::Frame::SharedPtr MSG);

  /** 
   * @brief Handles diagnostics
   *
   */
  void ros2DiagnosticHandler();

  /**
   * @brief
  */
  void heartbeatTimerCallback();

  void createDataArray(
    const std::vector<uint16_t> data_in, 
    const std::vector<uint16_t> data_lengths, 
    std::array<uint8_t, 8UL> &data_out);



  /**
   * @brief Parses ambient temp and pressure
  */
  void recvAMB(const can_msgs::msg::Frame::SharedPtr MSG);
  
  /**
   * @brief Parses Aftertreatment 1 Diesel Exhaust Fluid Tank 1 information
  */
  void recvAT1T1I1(const can_msgs::msg::Frame::SharedPtr MSG);  // NAPA BLUE DEF!!!

  /**
   * @brief Parses Basic joystick message 1
  */
  void recvBJM1(const can_msgs::msg::Frame::SharedPtr MSG);

  /**
   * @brief Parses Basic joystick message 2
  */
  void recvBJM2(const can_msgs::msg::Frame::SharedPtr MSG);

  /**
   * @brief Parses Cruise Control / Vehicle Speed 1 message
  */
  void recvCCVS1(const can_msgs::msg::Frame::SharedPtr MSG);  // idk parking brake and vehicle speed?

  /**
   * @brief Parses Cruise Control / Vehicle Speed 5 message (or Track speed, directional vehicle speed) 
  */
  void recvCCVS5(const can_msgs::msg::Frame::SharedPtr MSG);
  
  /**
   * @brief Parses Dash Display 1 message
  */
  void recvDD1(const can_msgs::msg::Frame::SharedPtr MSG);

  /**
   * @brief Parses Electronic Engine Controller 1 message (Engine speed, torque)
  */
  void recvEEC1(const can_msgs::msg::Frame::SharedPtr MSG);
  
  /**
   * @brief Parses Electronic Engine Controller 2 message
  */
  void recvEEC2(const can_msgs::msg::Frame::SharedPtr MSG);
  
  /**
   * @brief Parses Electronic Engine Controller 3 message
  */
  void recvEEC3(const can_msgs::msg::Frame::SharedPtr MSG);

  /**
   * @brief Parses Engine Operating Information messages
  */
  void recvEOI(const can_msgs::msg::Frame::SharedPtr MSG);
  
  /**
   * @brief Engine Temperature 1
   * Signal List:
   *  Engine Coolant Temperature [degC]
   *  Engine Fuel Temperature 1 [degC]
   *  Engine Intercooler Temperature  [degC]
   *  Engine Oil Temperature 1 [degC]
   *  Engine CHarge Air Cooler Thermostat Opening [%]
   *  Engine Turbocharger 1 Oil Temperature [degC]
  */
  void recvET1(const can_msgs::msg::Frame::SharedPtr MSG);

  /**
   * @brief Parses Electronic Transmission Controller 2 message
  */
  void recvETC2(const can_msgs::msg::Frame::SharedPtr MSG);

  /**
   * @brief Parses Laser Leveling System Blade Control message
  */
  void recvLBC(const can_msgs::msg::Frame::SharedPtr MSG);

  /**
   * @brief Parse Transmission Fluids 2 message
   * Signal List:
   *  Transmission Oil Level 2 High / Low [l]
   *  Transmission Oil Level Switch
   *  Transmission Oil Life Remaining [%]
   *  Transmission Oil Temperature 2
   *  Transmission Overheat Indicator
   *  Transmission Oil Filter Restriction Switch
   *  Transmission Oil Level 2 Countdown Timer
   *  Transmission Oil Level 2 Measurement Status
   *  Transmission Torque Converter Oil Outlet Temperature [degC]
  */
  void recvTRF2(const can_msgs::msg::Frame::SharedPtr MSG);

  /**
   * @brief Parses Vehicle Electrical Power 1 message
  */
 void recvVEP1(const can_msgs::msg::Frame::SharedPtr MSG);

  /**
   * @brief Parses Vehicle Fluids message
   * Signal List: 
   *  Hydraulic oil filter restriction switch
   *  Hydraulic oil level [%]
   *  Hydraulic fluid temperature [degC]
   *  Winch oil pressure switch
  */
  void recvVF(const can_msgs::msg::Frame::SharedPtr MSG);

  // can vars
  sensor_msgs::msg::Temperature engine_coolant_temperature_;
  sensor_msgs::msg::Temperature hydraulic_oil_temperature_;
  sensor_msgs::msg::Temperature transmission_oil_temperature_;
  sensor_msgs::msg::Temperature torque_converter_oil_temp_;
  sensor_msgs::msg::Joy xbox_;
  geometry_msgs::msg::TwistStamped vehicle_speed_;
  geometry_msgs::msg::TwistStamped directional_vehicle_speed_;

  diagnostic_msgs::msg::DiagnosticArray AMB_;
  diagnostic_msgs::msg::DiagnosticStatus AMB_status_;

  diagnostic_msgs::msg::DiagnosticArray AT1T1I1_;
  diagnostic_msgs::msg::DiagnosticStatus AT1T1I1_status_;

  diagnostic_msgs::msg::DiagnosticArray BJM1_;
  diagnostic_msgs::msg::DiagnosticStatus BJM1_status_;

  diagnostic_msgs::msg::DiagnosticArray BJM2_;
  diagnostic_msgs::msg::DiagnosticStatus BJM2_status_;

  diagnostic_msgs::msg::DiagnosticArray CCVS1_;
  diagnostic_msgs::msg::DiagnosticStatus CCVS1_status_;

  diagnostic_msgs::msg::DiagnosticArray CCVS5_;
  diagnostic_msgs::msg::DiagnosticStatus CCVS5_status_;

  diagnostic_msgs::msg::DiagnosticArray DD1_;
  diagnostic_msgs::msg::DiagnosticStatus DD1_status_;

  diagnostic_msgs::msg::DiagnosticArray EEC1_;
  diagnostic_msgs::msg::DiagnosticStatus EEC1_status_;

  diagnostic_msgs::msg::DiagnosticArray EEC2_;
  diagnostic_msgs::msg::DiagnosticStatus EEC2_status_;

  diagnostic_msgs::msg::DiagnosticArray EEC3_;
  diagnostic_msgs::msg::DiagnosticStatus EEC3_status_;

  diagnostic_msgs::msg::DiagnosticArray ET1_;
  diagnostic_msgs::msg::DiagnosticStatus ET1_status_;

  diagnostic_msgs::msg::DiagnosticArray ETC2_;
  diagnostic_msgs::msg::DiagnosticStatus ETC2_status_;

  diagnostic_msgs::msg::DiagnosticArray EOI_;
  diagnostic_msgs::msg::DiagnosticStatus EOI_status_;

  diagnostic_msgs::msg::DiagnosticArray LBC_;
  diagnostic_msgs::msg::DiagnosticStatus LBC_status_;

  diagnostic_msgs::msg::DiagnosticArray TRF2_;
  diagnostic_msgs::msg::DiagnosticStatus TRF2_status_;

  diagnostic_msgs::msg::DiagnosticArray VEP1_;
  diagnostic_msgs::msg::DiagnosticStatus VEP1_status_;

  diagnostic_msgs::msg::DiagnosticArray VF_;
  diagnostic_msgs::msg::DiagnosticStatus VF_status_;

  // publishers
  std::shared_ptr<rlc::LifecyclePublisher<can_msgs::msg::Frame>> pub_can_;
  std::shared_ptr<rlc::LifecyclePublisher<sensor_msgs::msg::JointState>> pub_joint_;
  std::shared_ptr<rlc::LifecyclePublisher<sensor_msgs::msg::NavSatFix>> pub_nav_sat_fix_;
  
  // CAN publishers
  std::shared_ptr<rlc::LifecyclePublisher<sensor_msgs::msg::Temperature>> pub_hydraulic_temp_;
  std::shared_ptr<rlc::LifecyclePublisher<sensor_msgs::msg::Temperature>> pub_transmission_oil_temp_;
  std::shared_ptr<rlc::LifecyclePublisher<sensor_msgs::msg::Temperature>> pub_torque_converter_oil_temp_;
  std::shared_ptr<rlc::LifecyclePublisher<sensor_msgs::msg::Temperature>> pub_engine_coolant_temp_;
  std::shared_ptr<rlc::LifecyclePublisher<geometry_msgs::msg::TwistStamped>> pub_track_speed_;
  std::shared_ptr<rlc::LifecyclePublisher<geometry_msgs::msg::TwistStamped>> pub_vehicle_speed_;
  std::shared_ptr<rlc::LifecyclePublisher<geometry_msgs::msg::TwistStamped>> pub_directional_vehicle_speed_;
  std::shared_ptr<rlc::LifecyclePublisher<sensor_msgs::msg::Joy>> pub_xbox_;
  std::shared_ptr<rlc::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>> pub_AMB_;
  std::shared_ptr<rlc::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>> pub_AT1T1I1_;
  std::shared_ptr<rlc::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>> pub_BJM1_;
  std::shared_ptr<rlc::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>> pub_BJM2_;
  std::shared_ptr<rlc::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>> pub_CCVS1_;
  std::shared_ptr<rlc::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>> pub_CCVS5_;
  std::shared_ptr<rlc::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>> pub_DD1_;
  std::shared_ptr<rlc::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>> pub_EEC1_;
  std::shared_ptr<rlc::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>> pub_EEC2_;
  std::shared_ptr<rlc::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>> pub_EEC3_;
  std::shared_ptr<rlc::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>> pub_EOI_;
  std::shared_ptr<rlc::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>> pub_ET1_;
  std::shared_ptr<rlc::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>> pub_ETC2_;
  std::shared_ptr<rlc::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>> pub_LBC_;
  std::shared_ptr<rlc::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>> pub_TRF2_;
  std::shared_ptr<rlc::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>> pub_VEP1_;
  std::shared_ptr<rlc::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>> pub_VF_;
  
  // subscribers
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr sub_can_;

  // Timer
  rclcpp::TimerBase::SharedPtr component_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  // params
  std::string dbw_dbc_file_;  // set in launch file. Files such as MV5.dbc
  std::string frame_id_;      // such as: base, etc.
  std::string sensor_name_;   // such as: /Imu/microstrain/joint_1_L, or w/e
  uint8_t device_ID_;         // such as 226
  NewEagle::Dbc dbw_dbc_db;   // new eagle dbc database

  std::string sub_topic_can_;  // such as "/from_can_bus"
  std::string pub_topic_can_;  // such as "/to_can_bus"

  std::string pub_topic_hydraulic_temp_           = "/temperature/GENERIC/hydraulic_oil";  // publishes
  std::string pub_topic_transmission_oil_temp_    = "/temperature/GENERIC/transmission_oil";
  std::string pub_topic_torque_converter_oil_temp_= "/temperature/GENERIC/transmission_torque_converter_oil";
  std::string pub_topic_engine_coolant_temp_      = "/temperature/GENERIC/engine_coolant";  // publishes
  std::string pub_topic_track_speed_              = "/twist/GENERIC/track_speed";  // not sure, says it's used but not captured in log
  std::string pub_topic_vehicle_speed_            = "/twist/GENERIC/vehicle_speed";  // publishes, but data in log is stuck at FFFF
  std::string pub_topic_directional_vehicle_speed_= "/twist/GENERIC/directional_vehicle_speed";  // publishes, but data in log is stuck at FFFF
  std::string pub_topic_xbox_   = "/joy";  // 
  std::string pub_topic_AMB_    = "/diagnostics/GENERIC/ambient_pressure_temperature";  // 
  std::string pub_topic_AT1T1I1_= "/diagnostics/GENERIC/aftertreatment_1_diesel_exhaust_fluid_tank_information_1";  // publishes (did not see value chang e but like, it's DEF)
  std::string pub_topic_BJM1_   = "/diagnostics/GENERIC/basic_joystick_message_1";  // 
  std::string pub_topic_BJM2_   = "/diagnostics/GENERIC/basic_joystick_message_2";  // 
  std::string pub_topic_CCVS1_  = "/diagnostics/GENERIC/parking_brake";  // change name
  std::string pub_topic_CCVS5_  = "/diagnostics/GENERIC/directional_vehicle_speed";  // change name
  std::string pub_topic_DD1_    = "/diagnostics/GENERIC/dash_display_1";
  std::string pub_topic_EEC1_   = "/diagnostics/GENERIC/electronic_engine_controller_1";  // publishes (some not available)
  std::string pub_topic_EEC2_   = "/diagnostics/GENERIC/electronic_engine_controller_2";  // publishes (some not available)
  std::string pub_topic_EEC3_   = "/diagnostics/GENERIC/electronic_engine_controller_3";
  std::string pub_topic_EOI_    = "/diagnostics/GENERIC/engine_operating_information";
  std::string pub_topic_ET1_    = "/diagnostics/GENERIC/engine_temperature_1";
  std::string pub_topic_ETC2_   = "/diagnostics/GENERIC/transmission_gear";  // not sure, says it's used but not captured in log
  std::string pub_topic_LBC_    = "/diagnostics/GENERIC/lift_blade_control";  // 
  std::string pub_topic_TRF2_   = "/diagnostics/GENERIC/transmission_fluids_2";
  std::string pub_topic_VEP1_   = "/diagnostics/GENERIC/vehicle_electrical_power_1";
  std::string pub_topic_VF_     = "/diagnostics/GENERIC/vehicle_fluids";


  bool set_new_source_address_;
  uint8_t new_source_address_;
  std::array<uint8_t, 8UL> device_name_;

  bool heartbeat_flag_;
};

}  // namespace generic_can_driver

#endif  // GENERIC_CAN_DRIVER__GENERIC_CAN_DRIVER_NODE_HPP_
