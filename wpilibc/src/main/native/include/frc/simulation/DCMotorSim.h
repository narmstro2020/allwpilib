// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/moment_of_inertia.h>
#include <units/torque.h>
#include <wpi/MathExtras.h>

#include "frc/RobotController.h"
#include "frc/simulation/LinearSystemSim.h"
#include "frc/system/plant/DCMotor.h"
#include "frc/system/plant/LinearSystemId.h"

namespace frc::sim {
/**
 * Represents a simulated DC motor mechanism.
 */
template <typename Input>
  requires(units::current_unit<Input> || units::voltage_unit<Input>)
class DCMotorSim : public LinearSystemSim<2, 1, 2> {
 public:
  using Velocity_t = units::unit_t<
      units::compound_unit<units::radian, units::inverse<units::seconds>>>;

  using Acceleration_t = units::unit_t<units::compound_unit<
      units::compound_unit<units::radian, units::inverse<units::seconds>>,
      units::inverse<units::seconds>>>;

  using Input_t = units::unit_t<Input>;

  /**
   * Creates a simulated DC motor mechanism.
   *
   * @param gearbox The gearbox attached to the system.
   * @param J the moment of inertia J of the DC motor.
   * @param gearing Gear ratio from motor to output.
   * @throws std::domain_error if J <= 0 or gearing <= 0.
   * @param measurementStdDevs The standard deviation of the measurement noise.
   */
  DCMotorSim(const DCMotor& gearbox, units::kilogram_square_meter_t J,
             double gearing,
             const std::array<double, 2>& measurementStdDevs = {0.0, 0.0})
      : LinearSystemSim<2, 1, 2>(
            LinearSystemId::DCMotorSystem<Input>(gearbox, J, gearing),
            measurementStdDevs),
        m_gearbox(gearbox),
        m_gearing(gearing),
        m_j(J) {}

  /**
   * Creates a simulated DC motor mechanism.
   *
   * @param gearbox The gearbox attached to the system.
   * @param kV The velocity gain, in volts/(unit/sec).
   * @param kA The acceleration gain, in volts/(unit/sec²).
   * @throws std::domain_error if kV < 0 or kA <= 0.
   * @param measurementStdDevs The standard deviation of the measurement noise.
   */
  DCMotorSim(const DCMotor& gearbox, decltype(1_V / 1_rad_per_s) kV,
             decltype(1_V / 1_rad_per_s_sq) kA,
             const std::array<double, 2>& measurementStdDevs = {0.0, 0.0})
      : LinearSystemSim<2, 1, 2>(
            LinearSystemId::IdentifyPositionSystem<units::radian, Input>(kV,
                                                                         kA),
            measurementStdDevs),
        m_gearbox(gearbox),
        m_gearing(-gearbox.Kv.value() * m_plant.A(1, 1) / m_plant.B(1, 0)),
        m_j(m_gearing * gearbox.Kt.value() /
            (gearbox.R.value() * m_plant.B(1, 0))) {
    // See wpimath/algorithms.md#DC_motor_sim for derivation
  }

  using LinearSystemSim::SetInput;
  using LinearSystemSim::SetState;

  /**
   * Sets the state of the DC motor.
   *
   * @param angularPosition The new position
   * @param angularVelocity The new velocity
   */
  void SetState(units::radian_t angularPosition,
                units::radians_per_second_t angularVelocity) {
    SetState(Vectord<2>{angularPosition, angularVelocity});
  }

  /**
   * Sets the DC motor's angular position.
   *
   * @param angularPosition The new position in radians.
   */
  void SetAngle(units::radian_t angularPosition) {
    SetState(angularPosition, GetAngularVelocity());
  }

  /**
   * Sets the DC motor's angular velocity.
   *
   * @param angularVelocity The new velocity in radians per second.
   */
  void SetAngularVelocity(units::radians_per_second_t angularVelocity) {
    SetState(GetAngularPosition(), angularVelocity);
  }

  /**
   * Returns the DC motor position.
   *
   * @return The DC motor position.
   */
  units::radian_t GetAngularPosition() const {
    return units::radian_t{GetOutput(0)};
  }

  /**
   * Returns the DC motor velocity.
   *
   * @return The DC motor velocity.
   */
  units::radians_per_second_t GetAngularVelocity() const {
    return units::radians_per_second_t{GetOutput(1)};
  }

  /**
   * Returns the DC motor acceleration.
   *
   * @return The DC motor acceleration
   */
  units::radians_per_second_squared_t GetAngularAcceleration() const {
    return units::radians_per_second_squared_t{
        (m_plant.A() * m_x + m_plant.B() * m_u)(1, 0)};
  }

  /**
   * Returns the DC motor torque.
   *
   * @return The DC motor torque
   */
  units::newton_meter_t GetTorque() const {
    return units::newton_meter_t{GetAngularAcceleration().value() *
                                 m_j.value()};
  }

  /**
   * Returns the DC motor current draw.
   *
   * @return The DC motor current draw.
   */
  units::ampere_t GetCurrent() const {
    // I = V / R - omega / (Kv * R)
    // Reductions are greater than 1, so a reduction of 10:1 would mean the
    // motor is spinning 10x faster than the output.
    return m_gearbox.Current(units::radians_per_second_t{m_x(1)} * m_gearing,
                             units::volt_t{m_u(0)}) *
           wpi::sgn(m_u(0));
  }

  /**
   * Gets the input voltage for the DC motor.
   *
   * @return The DC motor input voltage.
   */
  units::volt_t GetVoltage() const { return units::volt_t{GetInput(0)}; }

  /**
   * Sets the input for the DC motor.
   *
   * @param input The input.
   */
  void SetInput(Input_t input) {
    SetInput(Vectord<1>{input.value()});

    if constexpr (units::voltage_unit<Input>) {
      ClampInput(frc::RobotController::GetBatteryVoltage().value());
    } else if constexpr (units::current_unit<Input>) {
      ClampInput(m_gearbox.stallCurrent.value());
    }
  }

  /**
   * Returns the gearbox.
   */
  const DCMotor& GetGearbox() const { return m_gearbox; }

  /**
   * Returns the gearing;
   */
  double GetGearing() const { return m_gearing; }

  /**
   * Returns the moment of inertia
   */
  units::kilogram_square_meter_t GetJ() const { return m_j; }

 private:
  const DCMotor& m_gearbox;
  const double m_gearing;
  const units::kilogram_square_meter_t m_j;
};
}  // namespace frc::sim
