// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// THIS FILE WAS AUTO-GENERATED BY ./wpilibc/generate_pwm_motor_controllers.py. DO NOT MODIFY

#pragma once

#include "frc/motorcontrol/PWMMotorController.h"

namespace frc {

/**
 * REV Robotics SPARKMini Motor Controller with PWM control.
 *
 * Note that the SPARKMini uses the following bounds for PWM values. These
 * values should work reasonably well for most controllers, but if users
 * experience issues such as asymmetric behavior around the deadband or
 * inability to saturate the controller in either direction, calibration is
 * recommended. The calibration procedure can be found in the SPARKMini User
 * Manual available from REV Robotics.
 *
 * \li 2.500ms = full "forward"
 * \li 1.510ms = the "high end" of the deadband range
 * \li 1.500ms = center of the deadband range (off)
 * \li 1.490ms = the "low end" of the deadband range
 * \li 0.500ms = full "reverse"
 */
class SparkMini : public PWMMotorController {
 public:
  /**
   * Constructor for a SPARKMini connected via PWM.
   *
   * @param channel The PWM channel that the SPARKMini is attached to. 0-9 are
   *                on-board, 10-19 are on the MXP port
   */
  explicit SparkMini(int channel);

  SparkMini(SparkMini&&) = default;
  SparkMini& operator=(SparkMini&&) = default;
};

}  // namespace frc
