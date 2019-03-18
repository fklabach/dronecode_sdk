#pragma once

#include <cstdint>
#include "plugin_impl_base.h"
#include "mavlink_include.h"
#include "system.h"
#include "plugins/manualcontrol/manualcontrol.h"

namespace dronecode_sdk {

class ManualControlImpl : public PluginImplBase
{
public:
  ManualControlImpl(System &system);
  ~ManualControlImpl();

  void init() override;
  void deinit() override;
  
  void enable() override;
  void disable() override;

  ManualControl::Result set_manual_control(ManualControl::Data const &manualCtrl);

  ManualControl::Result set_local_position_target_ned(ManualControl::PositionTargetLocalNED const& setpoint);

  void arm(const ManualControl::result_callback_t &callback);

  void disarm(const ManualControl::result_callback_t &callback);

  /**
   * @brief enter altitude control mode
   * @param callback  result callback
   */
  void enter_altctl_async(const ManualControl::result_callback_t &callback);

  /**
   * @brief enter position control mode
   * @param callback  result callback
   */
  void enter_posctl_async(const ManualControl::result_callback_t &callback);

  /**
   * @brief enter offboard mode
   * @param callback  result callback
   */
  void enter_offboard_async(const ManualControl::result_callback_t &callback);

  /**
   * @brief enter stabilized mode
   * @param callback  result callback
   */
  void enter_stabilized_async(const ManualControl::result_callback_t &callback);

private:

  static constexpr uint8_t VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED = 1;

  static int const ManualControlRange;

  /**
   * @brief send manual control message usung manual control data
   * @details this method has no way of reporting errors, we don't get any
   * ack for MANUAL_CONTROL message
   * @param manualCtrl  manual control data
   */
  void sendManualControl(ManualControl::Data const &manualCtrl);

  /**
   * @brief check all values in manual control data for valid ranges
   * @param manualCtrl  manual control data
   * @return  true when all ranges valid
   */
  static bool rangesValid(ManualControl::Data const &manualCtrl);

  /**
   * @brief check mavlink manual control val range
   * @param val   value to check
   */
  static bool valValid(int const val);


  static ManualControl::Result manualctl_result_from_command_result(MAVLinkCommands::Result result);

  void command_result_callback(MAVLinkCommands::Result command_result,
                               const ManualControl::result_callback_t &callback);

};

} // namespace dronecode_sdk
