// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/moment_of_inertia.h>

#include <wpi/MathExtras.h>

#include "frc/simulation/AffineSystemSim.h"
#include "frc/system/AffineSystem.h"
#include "frc/system/LinearSystem.h"
#include "frc/system/plant/DCMotor.h"

namespace frc::sim {
/**
 * Represents a simulated flywheel mechanism.
 */
class FlywheelSim : public AffineSystemSim<1, 1, 1> {
 public:
  /**
   * Creates a simulated flywheel mechanism.
   *
   * @param linearSystemComponent             The linear system component of the affine system for the flywheel. This
   *                           system can be created with
   *                           LinearSystemId::FlywheelSystem() or
   * LinearSystemId::IdentifyVelocitySystem().
   * @param kS                 The minimum voltage needed to move the flywheel. 
   * @param gearbox            The type of and number of motors in the flywheel
   *                           gearbox.
   * @param measurementStdDevs The standard deviation of the measurement noise.
   */
  FlywheelSim(const LinearSystem<1, 1, 1>& linearSystemComponent, 
              units::volt_t kS,
              const DCMotor& gearbox,
              const std::array<double, 1>& measurementStdDevs = {0.0});

  using AffineSystemSim::SetState;
  using AffineSystemSim::Update;
  using AffineSystemSim::Setc;

  /**
   * Sets the flywheel's state.
   *
   * @param velocity The new velocity
   */
  void SetState(units::radians_per_second_t velocity);

  /**
   * Returns the flywheel velocity.
   *
   * @return The flywheel velocity.
   */
  units::radians_per_second_t GetAngularVelocity() const;

  /**
   * Returns the flywheel current draw.
   *
   * @return The flywheel current draw.
   */
  units::ampere_t GetCurrentDraw() const;

  /**
   * Gets the input voltage for the flywheel.
   *
   * @return The flywheel input voltage.
   */
  units::volt_t GetInputVoltage() const;

  /**
   * Sets the input voltage for the flywheel.
   *
   * @param voltage The input voltage.
   */
  void SetInputVoltage(units::volt_t voltage);

  /**
   * Returns the gearbox.
   */
  DCMotor Gearbox() const { return m_gearbox; }

  /**
   * Returns the gearing;
   */
  double Gearing() const { return m_gearing; }

  /**
   * Returns the moment of inertia
   */
  units::kilogram_square_meter_t J() const { return m_j; }

  /**
   * Returns the voltage needed to overcome static friction.
   *
   */
  units::volt_t Ks() const {return m_kS;}

  /**
   * Updates the simulation.
   *
   * @param dt The time between updates.
   */
  void Update(units::second_t dt) {
    AffineSystemSim::Setc(0, (-m_kS.value() / m_kA.value()) * wpi::sgn(m_x(0, 0)));
    AffineSystemSim::Update(dt);
  }


 private:
  DCMotor m_gearbox;
  units::volt_t m_kS;
  decltype(1_V / 1_rad_per_s_sq) m_kA;
  double m_gearing;
  units::kilogram_square_meter_t m_j;
};
}  // namespace frc::sim
