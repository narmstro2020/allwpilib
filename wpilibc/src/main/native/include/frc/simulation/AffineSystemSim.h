// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <array>

#include <units/current.h>
#include <units/time.h>

#include "frc/EigenCore.h"
#include "frc/RobotController.h"
#include "frc/StateSpaceUtil.h"
#include "frc/system/AffineSystem.h"

namespace frc::sim {
/**
 * This class helps simulate affine systems. To use this class, do the following
 * in the simulationPeriodic() method.
 *
 * Call the SetInput() method with the inputs to your system (generally
 * voltage). Call the Update() method to update the simulation. Set simulated
 * sensor readings with the simulated positions in the GetOutput() method.
 *
 * @tparam States  Number of states of the system.
 * @tparam Inputs  Number of inputs to the system.
 * @tparam Outputs Number of outputs of the system.
 */
template <int States, int Inputs, int Outputs>
class AffineSystemSim {
 public:
  /**
   * Creates a simulated generic affine system.
   *
   * @param affineSystem             The system to simulate.
   * @param measurementStdDevs The standard deviations of the measurements.
   */
  explicit AffineSystemSim(
      const AffineSystem<States, Inputs, Outputs>& affineSystem,
      const std::array<double, Outputs>& measurementStdDevs = {})
      : m_affineSystem(affineSystem), m_measurementStdDevs(measurementStdDevs) {
    m_x = Vectord<States>::Zero();
    m_y = Vectord<Outputs>::Zero();
    m_u = Vectord<Inputs>::Zero();
    m_c = Vectord<States>::Zero();
  }

  virtual ~AffineSystemSim() = default;

  /**
   * Updates the simulation.
   *
   * @param dt The time between updates.
   */
  void Update(units::second_t dt) {
    // Update x. By default, this is the affine system dynamics x_k+1 = Ax_k +
    // Bu_k + c
    m_x = UpdateX(m_x, m_u, dt);

    // y = Cx + Du
    m_y = m_affineSystem.CalculateY(m_x, m_u);

    // Add noise. If the user did not pass a noise vector to the
    // constructor, then this method will not do anything because
    // the standard deviations default to zero.
    m_y += frc::MakeWhiteNoiseVector<Outputs>(m_measurementStdDevs);
  }

  /**
   * Returns the current output of the system.
   *
   * @return The current output of the system.
   */
  const Vectord<Outputs>& GetOutput() const { return m_y; }

  /**
   * Returns an element of the current output of the system.
   *
   * @param row The row to return.
   * @return An element of the current output of the system.
   */
  double GetOutput(int row) const { return m_y(row); }

  /**
   * Sets the system inputs (usually voltages).
   *
   * @param u The system inputs.
   */
  void SetInput(const Vectord<Inputs>& u) { m_u = u; }

  /**
   * Sets the system inputs.
   *
   * @param row   The row in the input matrix to set.
   * @param value The value to set the row to.
   */
  void SetInput(int row, double value) { m_u(row, 0) = value; }

  /**
   * Returns the current input of the system.
   *
   * @return The current input of the system.
   */
  const Vectord<Inputs>& GetInput() const { return m_u; }

  /**
   * Returns an element of the current input of the system.
   *
   * @param row The row to return.
   * @return An element of the current input of the system.
   */
  double GetInput(int row) const { return m_u(row); }

  /**
   * Sets the constant vector.
   *
   * @param c The constant vector.
   */
  void Setc(const Vectord<States>& c) { m_c = c; }

  /**
   * Sets an element of the constant vector.
   *
   * @param row The row in the constant vector to set.
   * @param value The value to set the row to.
   */
  void Setc(int row, double value) { m_c(row, 0) = value; }

  /**
   * Returns the current constant vector of the system.
   *
   * @return The current constant vector of the system.
   */
  const Vectord<States>& Getc() const { return m_c; }

  /**
   * Returns an element of the current constant vector of the system.
   *
   * @param row The row to return.
   * @return An element of the current constant vector of the system.
   */
  double Getc(int row) const { return m_c(row); }

  /**
   * Sets the system state.
   *
   * @param state The new state.
   */
  void SetState(const Vectord<States>& state) { m_x = state; }

 protected:
  /**
   * Updates the state estimate of the system.
   *
   * @param currentXhat The current state estimate.
   * @param u           The system inputs (usually voltage).
   * @param dt          The time difference between controller updates.
   */
  virtual Vectord<States> UpdateX(const Vectord<States>& currentXhat,
                                  const Vectord<Inputs>& u,
                                  units::second_t dt) {
    return m_affineSystem.CalculateX(currentXhat, u, m_c, dt);
  }

  /**
   * Clamp the input vector such that no element exceeds the given voltage. If
   * any does, the relative magnitudes of the input will be maintained.
   *
   * @param maxInput The maximum magnitude of the input vector after clamping.
   */
  void ClampInput(double maxInput) {
    m_u = frc::DesaturateInputVector<Inputs>(m_u, maxInput);
  }

  /// The system that represents the affine system.
  AffineSystem<States, Inputs, Outputs> m_affineSystem;

  /// State vector.
  Vectord<States> m_x;

  /// Input vector.
  Vectord<Inputs> m_u;

  /// Output vector.
  Vectord<Outputs> m_y;

  /// Affine constant vector.
  Vectord<States> m_c;

  /// The standard deviations of measurements, used for adding noise to the
  /// measurements.
  std::array<double, Outputs> m_measurementStdDevs;
};
}  // namespace frc::sim
