#include "plugins/manualcontrol/manualcontrol.h"
#include "manualcontrol_impl.h"

namespace dronecode_sdk {

ManualControl::ManualControl(System &system)
  : _impl{new ManualControlImpl(system)}
{
}

ManualControl::~ManualControl()
{
}

ManualControl::Result ManualControl::set_manual_control(Data const &manualCtrl)
{
  return( _impl->set_manual_control(manualCtrl) );
}

ManualControl::Result ManualControl::set_local_position_target_ned(PositionTargetLocalNED const& setpoint)
{
  return( _impl->set_local_position_target_ned(setpoint) );
}

void ManualControl::arm(result_callback_t callback)
{
  _impl->arm(callback);
}

void ManualControl::disarm(result_callback_t callback)
{
  _impl->disarm(callback);
}

void ManualControl::enter_altctl_async(result_callback_t callback)
{
  _impl->enter_altctl_async(callback);
}

void ManualControl::enter_posctl_async(result_callback_t callback)
{
  _impl->enter_posctl_async(callback);
}

void ManualControl::enter_offboard_async(result_callback_t callback)
{
  _impl->enter_offboard_async(callback);
}

void ManualControl::enter_stabilized_async(result_callback_t callback)
{
  _impl->enter_stabilized_async(callback);
}

const char *ManualControl::result_str(Result result)
{
  switch (result) {
  case Result::SUCCESS:
    return "Success";
  case Result::NO_SYSTEM:
    return "No system";
  case Result::CONNECTION_ERROR:
    return "Connection error";
  case Result::BUSY:
    return "Busy";
  case Result::COMMAND_DENIED:
    return "Command denied";
  case Result::TIMEOUT:
    return "Timeout";
  case Result::WRONG_RANGE:
    return "Wrong Manual Control Range";
  case Result::UNKNOWN:
  default:
    return "Unknown";
  }
}

} // namespace dronecode_sdk
