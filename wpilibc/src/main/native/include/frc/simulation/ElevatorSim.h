// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <array>

#include <units/length.h>
#include <units/mass.h>
#include <units/velocity.h>
#include <wpi/MathExtras.h>

#include "frc/RobotController.h"
#include "frc/simulation/LinearSystemSim.h"
#include "frc/system/plant/DCMotor.h"
#include "frc/system/plant/LinearSystemId.h"

namespace frc::sim {
/**
 * Represents a simulated elevator mechanism.
 */
class ElevatorSim : public LinearSystemSim<2, 1, 2> {
 public:
  using Velocity_t = units::unit_t<
      units::compound_unit<units::meter, units::inverse<units::seconds>>>;

  using Acceleration_t = units::unit_t<units::compound_unit<
      units::compound_unit<units::meter, units::inverse<units::seconds>>,
      units::inverse<units::seconds>>>;

  /**
   * Constructs a simulated elevator mechanism.
   *
   * @param gearbox            The type of and number of motors in your
   *                           elevator gearbox.
   * @param carriageMass       The mass of the elevator carriage.
   * @param drumRadius         The radius of the drum that your cable is
   *                           wrapped around.
   * @param gearing            The gearing of the elevator (numbers greater
   *                           than 1 represent reductions).
   * @param minHeight          The minimum allowed height of the elevator.
   * @param maxHeight          The maximum allowed height of the elevator.
   * @param simulateGravity    Whether gravity should be simulated or not.
   * @param startingHeight     The starting height of the elevator.
   * @param measurementStdDevs The standard deviation of the measurements.
   * @throws std::domain_error if carriageMass <= 0, drumRadius <= 0 or gearing
   * <= 0.
   */
  ElevatorSim(const DCMotor& gearbox, units::kilogram_t carriageMass,
              units::meter_t drumRadius, double gearing,
              units::meter_t minHeight, units::meter_t maxHeight,
              bool simulateGravity, units::meter_t startingHeight,
              const std::array<double, 2>& measurementStdDevs = {0.0, 0.0});

  /**
   * Constructs a simulated elevator mechanism.
   *
   * @param gearbox            The type of and number of motors in your
   *                           elevator gearbox.
   * @param kV                 The velocity gain.
   * @param kA                 The acceleration gain.
   * @param gearing            The gearing of the elevator (numbers greater
   *                           than 1 represent reductions).
   * @param minHeight          The minimum allowed height of the elevator.
   * @param maxHeight          The maximum allowed height of the elevator.
   * @param simulateGravity    Whether gravity should be simulated or not.
   * @param startingHeight     The starting height of the elevator.
   * @param measurementStdDevs The standard deviation of the measurements.
   */
  ElevatorSim(const DCMotor& gearbox, decltype(1_V / 1_mps) kV,
              decltype(1_V / 1_mps_sq) kA, double gearing,
              units::meter_t minHeight, units::meter_t maxHeight,
              bool simulateGravity, units::meter_t startingHeight,
              const std::array<double, 2>& measurementStdDevs = {0.0, 0.0});

  /**
   * Constructs a simulated elevator mechanism.
   *
   * @param gearbox            The type of and number of motors in your
   *                           elevator gearbox.
   * @param plant              The linear system representing the DC motor. This
   * system can be created with LinearSystemId::DCMotorSystem(). If
   * LinearSystemId::DCMotorSystem(kV, kA) is used, the distance unit must be
   * radians.
   * @param gearing            The gearing of the elevator (numbers greater
   *                           than 1 represent reductions).
   * @param minHeight          The minimum allowed height of the elevator.
   * @param maxHeight          The maximum allowed height of the elevator.
   * @param simulateGravity    Whether gravity should be simulated or not.
   * @param startingHeight     The starting height of the elevator.
   * @param measurementStdDevs The standard deviation of the measurements.
   */
  ElevatorSim(const DCMotor& gearbox, const LinearSystem<2, 1, 2>& plant,
              double gearing, units::meter_t minHeight,
              units::meter_t maxHeight, bool simulateGravity,
              units::meter_t startingHeight,
              const std::array<double, 2>& measurementStdDevs = {0.0, 0.0});

  using LinearSystemSim::SetState;

  /**
   * Sets the elevator's state. The new position will be limited between the
   * minimum and maximum allowed heights.
   * @param position The new position
   * @param velocity The new velocity
   */
  void SetState(units::meter_t position, units::meters_per_second_t velocity) {
    SetState(
        Vectord<2>{std::clamp(position, m_minHeight, m_maxHeight), velocity});
  }

  /**
   * Sets the elevator's position. The new position will be limited between the
   * minimum and maximum allowed heights.
   *
   * @param position The new position
   */
  void SetPosition(units::meter_t position) {
    SetState(position, GetVelocity());
  }

  /**
   * Sets the elevator's velocity.
   *
   * @param velocity The new velocity
   */
  void SetVelocity(units::meters_per_second_t velocity) {
    SetState(GetPosition(), velocity);
  }
  /**
   * Returns whether the elevator would hit the lower limit.
   *
   * @param elevatorHeight The elevator height.
   * @return Whether the elevator would hit the lower limit.
   */
  bool WouldHitLowerLimit(units::meter_t elevatorHeight) const {
    return elevatorHeight <= m_minHeight;
  }
  /**
   * Returns whether the elevator would hit the upper limit.
   *
   * @param elevatorHeight The elevator height.
   * @return Whether the elevator would hit the upper limit.
   */
  bool WouldHitUpperLimit(units::meter_t elevatorHeight) const {
    return elevatorHeight >= m_maxHeight;
  }
  /**
   * Returns whether the elevator has hit the lower limit.
   *
   * @return Whether the elevator has hit the lower limit.
   */
  bool HasHitLowerLimit() const {
    return WouldHitLowerLimit(units::meter_t{m_y(0)});
  }
  /**
   * Returns whether the elevator has hit the upper limit.
   *
   * @return Whether the elevator has hit the upper limit.
   */
  bool HasHitUpperLimit() const {
    return WouldHitUpperLimit(units::meter_t{m_y(0)});
  }
  /**
   * Returns the position of the elevator.
   *
   * @return The position of the elevator.
   */
  units::meter_t GetPosition() const { return units::meter_t{m_y(0)}; }
  /**
   * Returns the velocity of the elevator.
   *
   * @return The velocity of the elevator.
   */
  units::meters_per_second_t GetVelocity() const {
    return units::meters_per_second_t{m_x(1)};
  }

  /**
   * Returns the acceleration of the elevator.
   *
   * @return The acceleration of the elevator.
   */
  units::meters_per_second_squared_t GetAngularAcceleration() const {
    return units::meters_per_second_squared_t{
        (m_plant.A() * m_x + m_plant.B() * m_u)(1, 0)};
  }

  /**
   * Returns the elevator current draw.
   *
   * @return The elevator current draw.
   */
  units::ampere_t GetCurrentDraw() const {
    // I = V / R - omega / (Kv * R)
    // Reductions are greater than 1, so a reduction of 10:1 would mean the
    // motor is spinning 10x faster than the output.

    double kA = 1.0 / m_plant.B(1, 0);
    using Kv_t = units::unit_t<units::compound_unit<
        units::volt, units::inverse<units::meters_per_second>>>;
    Kv_t Kv = Kv_t{kA * m_plant.A(1, 1)};
    units::meters_per_second_t velocity{m_x(1)};
    units::radians_per_second_t motorVelocity = velocity * Kv * m_gearbox.Kv;

    // Perform calculation and return.
    return m_gearbox.Current(motorVelocity, units::volt_t{m_u(0)}) *
           wpi::sgn(m_u(0));
  }
  /**
   * Sets the input voltage for the elevator.
   *
   * @param voltage The input voltage.
   */
  void SetInputVoltage(units::volt_t voltage) {
    SetInput(Vectord<1>{voltage.value()});
    ClampInput(frc::RobotController::GetBatteryVoltage().value());
  }
  /**
   * Returns the gearbox.
   */
  const DCMotor& GetGearbox() const { return m_gearbox; }

  /**
   * Returns the gearing.
   */
  double GetGearing() const { return m_gearing; }

  /**
   * Returns the carriage mass.
   */
  units::kilogram_t GetCarriageMass() const { return m_carriageMass; }

  /**
   * Returns the drum radius.
   */
  units::meter_t GetDrumRadius() const { return m_drumRadius; }

 protected:
  /**
   * Updates the state estimate of the elevator.
   *
   * @param currentXhat The current state estimate.
   * @param u           The system inputs (voltage).
   * @param dt          The time difference between controller updates.
   */
  Vectord<2> UpdateX(const Vectord<2>& currentXhat, const Vectord<1>& u,
                     units::second_t dt) override;

 private:
  const DCMotor& m_gearbox;
  const double m_gearing;
  const units::meter_t m_drumRadius;
  const units::kilogram_t m_carriageMass;
  const units::meter_t m_minHeight;
  const units::meter_t m_maxHeight;
  const bool m_simulateGravity;
};
}  // namespace frc::sim
