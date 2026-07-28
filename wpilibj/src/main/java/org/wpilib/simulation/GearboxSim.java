// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.wpilib.simulation;

import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N2;
import org.wpilib.math.system.Gearbox;
import org.wpilib.math.system.LinearSystem;
import org.wpilib.system.RobotController;

/**
 * Represents a simulated gearbox driving a rotational mechanism.
 *
 * <p>Positions, velocities and torques are those of the gearbox's output shaft; current draw is the
 * total across all of its motors.
 */
public class GearboxSim extends LinearSystemSim<N2, N1, N2> {
  // Gearbox for the sim.
  private final Gearbox m_gearbox;

  // The moment of inertia of the driven mechanism in kg-m².
  private final double m_j;

  /**
   * Creates a simulated gearbox.
   *
   * @param plant The linear system representing the mechanism. This system can be created with
   *     {@link org.wpilib.math.system.Models#singleJointedArmFromPhysicalConstants(Gearbox,
   *     double)} or {@link org.wpilib.math.system.Models#singleJointedArmFromSysId(double,
   *     double)}. It must have been built with the same reduction as {@code gearbox}.
   * @param gearbox The gearbox driving the mechanism.
   * @param measurementStdDevs The standard deviations of the measurements. Can be omitted if no
   *     noise is desired. If present must have 2 elements. The first element is for position. The
   *     second element is for velocity.
   */
  public GearboxSim(LinearSystem<N2, N1, N2> plant, Gearbox gearbox, double... measurementStdDevs) {
    super(plant, measurementStdDevs);
    m_gearbox = gearbox;

    // By theorem 6.10.1 of https://file.tavsys.net/control/controls-engineering-in-frc.pdf,
    // the DC motor state-space model is:
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
            / (gearbox.motor.R * plant.getB(1, 0));
  }

  /**
   * Sets the state of the gearbox.
   *
   * @param angularPosition The new position in radians.
   * @param angularVelocity The new velocity in radians per second.
   */
  public void setState(double angularPosition, double angularVelocity) {
    setState(VecBuilder.fill(angularPosition, angularVelocity));
  }

  /**
   * Sets the gearbox's angular position.
   *
   * @param angularPosition The new position in radians.
   */
  public void setAngle(double angularPosition) {
    setState(angularPosition, getAngularVelocity());
  }

  /**
   * Sets the gearbox's angular velocity.
   *
   * @param angularVelocity The new velocity in radians per second.
   */
  public void setAngularVelocity(double angularVelocity) {
    setState(getAngularPosition(), angularVelocity);
  }

  /**
   * Returns the moment of inertia of the driven mechanism.
   *
   * @return The mechanism's moment of inertia in kg-m².
   */
  public double getJ() {
    return m_j;
  }

  /**
   * Returns the gearbox.
   *
   * @return The gearbox.
   */
  public Gearbox getGearbox() {
    return m_gearbox;
  }

  /**
   * Returns the gearbox's angular position.
   *
   * @return The gearbox's position in radians.
   */
  public double getAngularPosition() {
    return getOutput(0);
  }

  /**
   * Returns the gearbox's angular velocity.
   *
   * @return The gearbox's velocity in radians per second.
   */
  public double getAngularVelocity() {
    return getOutput(1);
  }

  /**
   * Returns the gearbox's angular acceleration.
   *
   * @return The gearbox's acceleration in rad/s².
   */
  public double getAngularAcceleration() {
    var acceleration = (m_plant.getA().times(m_x)).plus(m_plant.getB().times(m_u));
    return acceleration.get(1, 0);
  }

  /**
   * Returns the torque at the gearbox's output.
   *
   * @return The output torque in Newton-meters.
   */
  public double getTorque() {
    return getAngularAcceleration() * m_j;
  }

  /**
   * Returns the total current drawn by the gearbox's motors.
   *
   * @return The gearbox's current draw in amps.
   */
  public double getCurrentDraw() {
    // Gearbox.getCurrent() takes the velocity of the output and applies the reduction
    // internally to get the motor velocity, and returns the total across all the motors.
    return m_gearbox.getCurrent(m_x.get(1, 0), m_u.get(0, 0)) * Math.signum(m_u.get(0, 0));
  }

  /**
   * Gets the input voltage for the gearbox.
   *
   * @return The gearbox's input voltage.
   */
  public double getInputVoltage() {
    return getInput(0);
  }

  /**
   * Sets the input voltage for the gearbox.
   *
   * @param volts The input voltage.
   */
  public void setInputVoltage(double volts) {
    setInput(volts);
    clampInput(RobotController.getBatteryVoltage());
  }
}
