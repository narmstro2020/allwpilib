// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "wpi/math/system/Gearbox.hpp"
#include "wpi/math/system/LinearSystem.hpp"
#include "wpi/simulation/LinearSystemSim.hpp"
#include "wpi/units/angle.hpp"
#include "wpi/units/angular_acceleration.hpp"
#include "wpi/units/angular_velocity.hpp"
#include "wpi/units/moment_of_inertia.hpp"
#include "wpi/units/torque.hpp"

namespace wpi::sim {
/**
 * Represents a simulated gearbox driving a rotational mechanism.
 *
 * Positions, velocities and torques are those of the gearbox's output shaft;
 * current draw is the total across all of its motors.
 */
class GearboxSim : public LinearSystemSim<2, 1, 2> {
 public:
  /**
   * Creates a simulated gearbox.
   *
   * @param plant The linear system representing the mechanism. This system can
   *     be created with
   *     wpi::math::Models::SingleJointedArmFromPhysicalConstants() or
   *     wpi::math::Models::SingleJointedArmFromSysId(). It must have been built
   *     with the same reduction as the gearbox.
   * @param gearbox The gearbox driving the mechanism.
   * @param measurementStdDevs The standard deviation of the measurement noise.
   */
  GearboxSim(const wpi::math::LinearSystem<2, 1, 2>& plant,
             const wpi::math::Gearbox& gearbox,
             const std::array<double, 2>& measurementStdDevs = {0.0, 0.0});

  using LinearSystemSim::SetState;

  /**
   * Sets the state of the gearbox.
   *
   * @param angularPosition The new position
   * @param angularVelocity The new velocity
   */
  void SetState(wpi::units::radian_t angularPosition,
                wpi::units::radians_per_second_t angularVelocity);

  /**
   * Sets the gearbox's angular position.
   *
   * @param angularPosition The new position in radians.
   */
  void SetAngle(wpi::units::radian_t angularPosition);

  /**
   * Sets the gearbox's angular velocity.
   *
   * @param angularVelocity The new velocity in radians per second.
   */
  void SetAngularVelocity(wpi::units::radians_per_second_t angularVelocity);

  /**
   * Returns the gearbox's angular position.
   *
   * @return The gearbox's position.
   */
  wpi::units::radian_t GetAngularPosition() const;

  /**
   * Returns the gearbox's angular velocity.
   *
   * @return The gearbox's velocity.
   */
  wpi::units::radians_per_second_t GetAngularVelocity() const;

  /**
   * Returns the gearbox's angular acceleration.
   *
   * @return The gearbox's acceleration.
   */
  wpi::units::radians_per_second_squared_t GetAngularAcceleration() const;

  /**
   * Returns the torque at the gearbox's output.
   *
   * @return The output torque.
   */
  wpi::units::newton_meter_t GetTorque() const;

  /**
   * Returns the total current drawn by the gearbox's motors.
   *
   * @return The gearbox's current draw.
   */
  wpi::units::ampere_t GetCurrentDraw() const;

  /**
   * Gets the input voltage for the gearbox.
   *
   * @return The gearbox's input voltage.
   */
  wpi::units::volt_t GetInputVoltage() const;

  /**
   * Sets the input voltage for the gearbox.
   *
   * @param voltage The input voltage.
   */
  void SetInputVoltage(wpi::units::volt_t voltage);

  /**
   * Returns the gearbox.
   */
  const wpi::math::Gearbox& GetGearbox() const;

  /**
   * Returns the moment of inertia of the driven mechanism.
   */
  wpi::units::kilogram_square_meter_t GetJ() const;

 private:
  wpi::math::Gearbox m_gearbox;
  wpi::units::kilogram_square_meter_t m_j;
};
}  // namespace wpi::sim
