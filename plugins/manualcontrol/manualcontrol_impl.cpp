#include <iostream>
#include "global_include.h"
#include "manualcontrol_impl.h"
#include "dronecode_sdk_impl.h"
#include "px4_custom_mode.h"

namespace dronecode_sdk {

int const ManualControlImpl::ManualControlRange = 1000;

ManualControlImpl::ManualControlImpl(System &system) :
  PluginImplBase(system)
{
    _parent->register_plugin(this);
}

ManualControlImpl::~ManualControlImpl()
{
    _parent->unregister_plugin(this);
}

void ManualControlImpl::init()
{
  /// TODO set right autopilit param for manual control
}

void ManualControlImpl::deinit()
{
}

void ManualControlImpl::enable()
{
}

void ManualControlImpl::disable()
{
}

void ManualControlImpl::arm(const ManualControl::result_callback_t &callback)
{
  MAVLinkCommands::CommandLong command{};

  command.command = MAV_CMD_COMPONENT_ARM_DISARM;
  command.params.param1 = 1.0f; // arm
  command.target_component_id = _parent->get_autopilot_id();

  _parent->send_command_async(command,
                              std::bind(&ManualControlImpl::command_result_callback, this, std::placeholders::_1, callback));
}


void ManualControlImpl::disarm(const ManualControl::result_callback_t &callback)
{
  MAVLinkCommands::CommandLong command{};

  command.command = MAV_CMD_COMPONENT_ARM_DISARM;
  command.params.param1 = 0.0f; // disarm
  command.target_component_id = _parent->get_autopilot_id();

  _parent->send_command_async(command,
                              std::bind(&ManualControlImpl::command_result_callback, this, std::placeholders::_1, callback));
}

void ManualControlImpl::enter_altctl_async(const ManualControl::result_callback_t &callback)
{
  _parent->set_flight_mode_async(
      SystemImpl::FlightMode::ALTCTL,
      std::bind(&ManualControlImpl::command_result_callback, this, std::placeholders::_1, callback));
}

void ManualControlImpl::enter_posctl_async(const ManualControl::result_callback_t &callback)
{
    _parent->set_flight_mode_async(
        SystemImpl::FlightMode::POSCTL,
        std::bind(&ManualControlImpl::command_result_callback, this, std::placeholders::_1, callback));
}

void ManualControlImpl::enter_offboard_async(const ManualControl::result_callback_t &callback)
{
    _parent->set_flight_mode_async(
        SystemImpl::FlightMode::OFFBOARD,
        std::bind(&ManualControlImpl::command_result_callback, this, std::placeholders::_1, callback));
}

void ManualControlImpl::enter_stabilized_async(const ManualControl::result_callback_t &callback)
{
    _parent->set_flight_mode_async(
        SystemImpl::FlightMode::STABILIZED,
        std::bind(&ManualControlImpl::command_result_callback, this, std::placeholders::_1, callback));
}

ManualControl::Result ManualControlImpl::set_local_position_target_ned(ManualControl::PositionTargetLocalNED const& setpoint)
{
  mavlink_message_t message;
  mavlink_msg_set_position_target_local_ned_pack(
      GCSClient::system_id,
      GCSClient::component_id,
      &message,
      static_cast<uint32_t>(_parent->get_time().elapsed_s() * 1e3),
      _parent->get_system_id(),
      _parent->get_autopilot_id(),
      setpoint.coordFrame,
      setpoint.typeMask,
      setpoint.x,
      setpoint.y,
      setpoint.z,
      setpoint.vx,
      setpoint.vy,
      setpoint.vz,
      setpoint.afx,
      setpoint.afy,
      setpoint.afz,
      setpoint.yaw,
      setpoint.yawRate);
  _parent->send_message(message);

  return(ManualControl::Result::SUCCESS);
}

ManualControl::Result ManualControlImpl::set_manual_control(ManualControl::Data const &manualCtrl)
{
  /*
  LogInfo() << "sending manual control "
            << " x: " << manualCtrl.x
            << ", y: " << manualCtrl.y
            << ", z: " << manualCtrl.z
            << ", r: " << manualCtrl.r
            << ", buttons: " << manualCtrl.buttons;
            */

  if(!rangesValid(manualCtrl))
  {
    return(ManualControl::Result::WRONG_RANGE);
  }

  sendManualControl(manualCtrl);

  return(ManualControl::Result::SUCCESS);
}

void ManualControlImpl::sendManualControl(ManualControl::Data const &manualCtrl)
{
  mavlink_message_t message;
  mavlink_manual_control_t manualControl;
  // set manual control struct
  manualControl.target = _parent->get_system_id();
  manualControl.x = manualCtrl.x;
  manualControl.y = manualCtrl.y;
  manualControl.z = manualCtrl.z;
  manualControl.r = manualCtrl.r;
  manualControl.buttons = manualCtrl.buttons;

  mavlink_msg_manual_control_encode(GCSClient::system_id,
                                    GCSClient::component_id,
                                    &message,
                                    &manualControl);

  _parent->send_message(message);
}

bool ManualControlImpl::rangesValid(ManualControl::Data const &manualCtrl)
{
  // check ranges
  return( valValid(manualCtrl.x) &&
          valValid(manualCtrl.y) &&
          valValid(manualCtrl.z) &&
          valValid(manualCtrl.r) );
}

bool ManualControlImpl::valValid(int const val)
{
  return((val >= (-ManualControlRange)) && (val <= ManualControlRange));
}


ManualControl::Result
ManualControlImpl::manualctl_result_from_command_result(MAVLinkCommands::Result result)
{
    switch (result) {
        case MAVLinkCommands::Result::SUCCESS:
            return ManualControl::Result::SUCCESS;
        case MAVLinkCommands::Result::NO_SYSTEM:
            return ManualControl::Result::NO_SYSTEM;
        case MAVLinkCommands::Result::CONNECTION_ERROR:
            return ManualControl::Result::CONNECTION_ERROR;
        case MAVLinkCommands::Result::BUSY:
            return ManualControl::Result::BUSY;
        case MAVLinkCommands::Result::COMMAND_DENIED:
            return ManualControl::Result::COMMAND_DENIED;
        case MAVLinkCommands::Result::TIMEOUT:
            return ManualControl::Result::TIMEOUT;
        default:
            return ManualControl::Result::UNKNOWN;
    }
}

void ManualControlImpl::command_result_callback(MAVLinkCommands::Result command_result,
                                         const ManualControl::result_callback_t &callback)
{
    ManualControl::Result manualctl_result = manualctl_result_from_command_result(command_result);

    if (callback) {
        callback(manualctl_result);
    }
}


} // namespace dronecode_sdk
