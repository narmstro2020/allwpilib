// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// THIS FILE WAS AUTO-GENERATED BY ./wpilibc/generate_pwm_motor_controllers.py. DO NOT MODIFY

#include "frc/motorcontrol/SparkMini.h"

#include <hal/UsageReporting.h>

using namespace frc;

SparkMini::SparkMini(int channel) : PWMMotorController("SparkMini", channel) {
  SetBounds(2.5_ms, 1.51_ms, 1.5_ms, 1.49_ms, 0.5_ms);
  m_pwm.SetOutputPeriod(PWM::kOutputPeriod_5Ms);
  SetSpeed(0.0);

  HAL_ReportUsage("IO", GetChannel(), "RevSPARK");
}
