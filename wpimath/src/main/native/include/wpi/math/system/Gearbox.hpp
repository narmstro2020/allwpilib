// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <stdexcept>

#include "wpi/math/system/DCMotor.hpp"
#include "wpi/units/angular_velocity.hpp"
#include "wpi/units/base.hpp"
#include "wpi/units/current.hpp"
#include "wpi/units/torque.hpp"
#include "wpi/units/voltage.hpp"
#include "wpi/util/SymbolExports.hpp"

namespace wpi::math {

/**
 * Holds a number of identical DCMotors driving a common output shaft through a
 * gear reduction.
 *
 * DCMotor describes the motor alone. This class adds the two things that
 * describe how those motors are installed: how many of them there are, and the
 * reduction between them and the mechanism they drive.
 *
 * All of this class's calculations are performed at the *output* of the
 * gearbox. Velocities and torques are those of the driven shaft, not of the
 * individual motors, and currents are the total drawn by all the motors
 * together.
 */
class WPILIB_DLLEXPORT Gearbox {
 public:
  /// The type of motor in the gearbox.
  DCMotor motor;

  /// The number of motors in the gearbox.
  int numMotors;

  /// The gear reduction from the motors to the output, as input over output.
  double reduction;

  /**
   * Constructs a gearbox.
   *
   * @param motor The type of motor in the gearbox.
   * @param numMotors The number of motors in the gearbox.
   * @param reduction The gear reduction from the motors to the output, as input
   *     over output. Values greater than one are reductions: the output turns
   *     slower than the motors, with more torque.
   * @throws std::domain_error if numMotors <= 0 or reduction <= 0.
   */
  constexpr Gearbox(DCMotor motor, int numMotors, double reduction = 1.0)
      : motor(motor), numMotors(numMotors), reduction(reduction) {
    if (numMotors <= 0) {
      throw std::domain_error("numMotors must be greater than zero.");
    }
    if (reduction <= 0.0) {
      throw std::domain_error("reduction must be greater than zero.");
    }
  }

  /**
   * Returns a copy of this gearbox with an additional reduction applied.
   *
   * @param reduction The reduction to apply on top of this gearbox's existing
   *     reduction.
   */
  constexpr Gearbox WithReduction(double reduction) const {
    return Gearbox{motor, numMotors, this->reduction * reduction};
  }

  /**
   * Returns the torque the gearbox produces at its output when stalled.
   */
  constexpr wpi::units::newton_meter_t StallTorque() const {
    return motor.stallTorque * numMotors * reduction;
  }

  /**
   * Returns the total current the gearbox draws when stalled.
   */
  constexpr wpi::units::ampere_t StallCurrent() const {
    return motor.stallCurrent * numMotors;
  }

  /**
   * Returns the total current the gearbox draws under no load.
   */
  constexpr wpi::units::ampere_t FreeCurrent() const {
    return motor.freeCurrent * numMotors;
  }

  /**
   * Returns the angular velocity of the gearbox's output under no load.
   */
  constexpr wpi::units::radians_per_second_t FreeSpeed() const {
    return motor.freeSpeed / reduction;
  }

  /**
   * Returns the total current drawn by the gearbox with a given output velocity
   * and input voltage.
   *
   * @param velocity The angular velocity of the output.
   * @param inputVoltage The voltage being applied to the motors.
   */
  constexpr wpi::units::ampere_t Current(
      wpi::units::radians_per_second_t velocity,
      wpi::units::volt_t inputVoltage) const {
    return numMotors * motor.Current(velocity * reduction, inputVoltage);
  }

  /**
   * Returns the total current drawn by the gearbox for a given output torque.
   *
   * @param torque The torque produced at the output.
   */
  constexpr wpi::units::ampere_t Current(
      wpi::units::newton_meter_t torque) const {
    return torque / (reduction * motor.Kt);
  }

  /**
   * Returns the torque produced at the gearbox's output for a given total
   * current.
   *
   * @param current The total current drawn by the motors.
   */
  constexpr wpi::units::newton_meter_t Torque(
      wpi::units::ampere_t current) const {
    return current * motor.Kt * reduction;
  }

  /**
   * Returns the voltage provided to the motors for a given output torque and
   * angular velocity.
   *
   * @param torque The torque produced at the output.
   * @param velocity The angular velocity of the output.
   */
  constexpr wpi::units::volt_t Voltage(
      wpi::units::newton_meter_t torque,
      wpi::units::radians_per_second_t velocity) const {
    return motor.Voltage(torque / (numMotors * reduction),
                         velocity * reduction);
  }

  /**
   * Returns the angular velocity produced at the gearbox's output at a given
   * output torque and input voltage.
   *
   * @param torque The torque produced at the output.
   * @param inputVoltage The input voltage provided to the motors.
   */
  constexpr wpi::units::radians_per_second_t Velocity(
      wpi::units::newton_meter_t torque,
      wpi::units::volt_t inputVoltage) const {
    return motor.Velocity(torque / (numMotors * reduction), inputVoltage) /
           reduction;
  }
};

}  // namespace wpi::math

#include "wpi/math/system/proto/GearboxProto.hpp"
#include "wpi/math/system/struct/GearboxStruct.hpp"
