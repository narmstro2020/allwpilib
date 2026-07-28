// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.wpilib.simulation;

import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.system.Gearbox;
import org.wpilib.math.system.LinearSystem;
import org.wpilib.math.system.Models;
import org.wpilib.system.RobotController;

/**
 * Represents a simulated flywheel mechanism.
 *
 * <p>Velocities and torques are those of the flywheel itself; current draw is the total across all
 * of the gearbox's motors.
 */
public class FlywheelSim extends LinearSystemSim<N1, N1, N1> {
  // Gearbox for the flywheel.
  private final Gearbox m_gearbox;

  // The moment of inertia for the flywheel mechanism.
  private final double m_j;

  /**
   * Creates a simulated flywheel mechanism.
   *
   * @param plant The linear system that represents the flywheel. Use either {@link
   *     Models#flywheelFromPhysicalConstants(Gearbox, double)} if using physical constants or
   *     {@link Models#flywheelFromSysId(double, double)} if using system characterization. It must
   *     have been built with the same reduction as {@code gearbox}.
   * @param gearbox The gearbox driving the flywheel.
   * @param measurementStdDevs The standard deviations of the measurements. Can be omitted if no
   *     noise is desired. If present must have 1 element for velocity.
   */
  public FlywheelSim(
      LinearSystem<N1, N1, N1> plant, Gearbox gearbox, double... measurementStdDevs) {
    super(plant, measurementStdDevs);
    m_gearbox = gearbox;

    // By theorem 6.10.1 of https://file.tavsys.net/control/controls-engineering-in-frc.pdf,
    // the flywheel state-space model is:
    //
    //   dx/dt = -G²Kₜ/(KᵥRJ)x + (GKₜ)/(RJ)u
    //   A = -G²Kₜ/(KᵥRJ)
    //   B = GKₜ/(RJ)
    //
    // Solve for J.
    //
    //   B = GKₜ/(RJ)
    //   J = GKₜ/(RB)
    //
    // Kₜ here is that of the gearbox as a whole, so it scales with the number of
    // motors. G is taken from the gearbox, so the plant must have been built with
    // the same reduction.
    m_j =
        gearbox.reduction
            * gearbox.numMotors
            * gearbox.motor.Kt
            / (gearbox.motor.R * plant.getB(0, 0));
  }

  /**
   * Sets the flywheel's angular velocity.
   *
   * @param velocity The new velocity in radians per second.
   */
  public void setAngularVelocity(double velocity) {
    setState(VecBuilder.fill(velocity));
  }

  /**
   * Returns the moment of inertia.
   *
   * @return The flywheel's moment of inertia in kg-m².
   */
  public double getJ() {
    return m_j;
  }

  /**
   * Returns the gearbox for the flywheel.
   *
   * @return The flywheel's gearbox.
   */
  public Gearbox getGearbox() {
    return m_gearbox;
  }

  /**
   * Returns the flywheel's velocity.
   *
   * @return The flywheel's velocity in rad/s.
   */
  public double getAngularVelocity() {
    return getOutput(0);
  }

  /**
   * Returns the flywheel's acceleration.
   *
   * @return The flywheel's acceleration in rad/s².
   */
  public double getAngularAcceleration() {
    var acceleration = (m_plant.getA().times(m_x)).plus(m_plant.getB().times(m_u));
    return acceleration.get(0, 0);
  }

  /**
   * Returns the flywheel's torque.
   *
   * @return The flywheel's torque in Newton-meters.
   */
  public double getTorque() {
    return getAngularAcceleration() * m_j;
  }

  /**
   * Returns the total current drawn by the gearbox's motors.
   *
   * @return The flywheel's current draw in amps.
   */
  public double getCurrentDraw() {
    // Gearbox.getCurrent() takes the velocity of the output and applies the reduction
    // internally to get the motor velocity, and returns the total across all the motors.
    return m_gearbox.getCurrent(m_x.get(0, 0), m_u.get(0, 0)) * Math.signum(m_u.get(0, 0));
  }

  /**
   * Gets the input voltage for the flywheel.
   *
   * @return The flywheel's input voltage.
   */
  public double getInputVoltage() {
    return getInput(0);
  }

  /**
   * Sets the input voltage for the flywheel.
   *
   * @param volts The input voltage.
   */
  public void setInputVoltage(double volts) {
    setInput(volts);
    clampInput(RobotController.getBatteryVoltage());
  }
}
