#pragma once

#include <functional>
#include <memory>
#include "plugin_base.h"

namespace dronecode_sdk {

class ManualControlImpl;
class System;

class ManualControl : public PluginBase
{
public:

  explicit ManualControl(System &system);

  ~ManualControl();

  enum class Result {
    SUCCESS = 0, /**< @brief %Request succeeded. */
    NO_SYSTEM, /**< @brief No system connected. */
    CONNECTION_ERROR, /**< @brief %Connection error. */
    BUSY, /**< @brief Vehicle busy. */
    COMMAND_DENIED, /**< @brief Command denied. */
    TIMEOUT, /**< @brief %Request timeout. */
    WRONG_RANGE,
    UNKNOWN /**< @brief Unknown error. */
  };

  /**
   * @brief Returns a human-readable English string for an Action::Result.
   *
   * @param result The enum value for which a human readable string is required.
   * @return Human readable string for the Action::Result.
   */
  static const char *result_str(Result result);

  /**
   * @brief Callback type for offboard requests.
   */
  typedef std::function<void(Result)> result_callback_t;

  struct Data
  {
    int x;
    int y;
    int z;
    int r;
    size_t buttons;
  };

  struct PositionTargetLocalNED {
    uint8_t coordFrame = 1;
    uint16_t typeMask = 0x00;
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;
    float vx = 0.0;
    float vy = 0.0;
    float vz = 0.0;
    float afx = 0.0;
    float afy = 0.0;
    float afz = 0.0;
    float yaw = 0.0;
    float yawRate = 0.0;
  };

  /**
   * @brief set manual control data
   * @details call is always asynchronous, MANUAL_CONTROL message has no ACK
   * @param manualCtrl
   * @return
   */
  Result set_manual_control(Data const &manualCtrl);

  Result set_local_position_target_ned(PositionTargetLocalNED const& setpoint);

  void arm(result_callback_t callback);

  void disarm(result_callback_t callback);

  /**
   * @brief enter altitude control mode
   * @param callback  result callback
   */
  void enter_altctl_async(result_callback_t callback);

  /**
   * @brief enter position control mode
   * @param callback  result callback
   */
  void enter_posctl_async(result_callback_t callback);

  /**
   * @brief enter offboard mode
   * @param callback  result callback
   */
  void enter_offboard_async(result_callback_t callback);

  /**
   * @brief enter stabilized mode
   * @param callback  result callback
   */
  void enter_stabilized_async(result_callback_t callback);

  // Non-copyable
  ManualControl(const ManualControl &) = delete;
  const ManualControl &operator=(const ManualControl &) = delete;

private:
  // Underlying implementation, set at instantiation
  std::unique_ptr<ManualControlImpl> _impl;
};

} // namespace dronecode_sdk
