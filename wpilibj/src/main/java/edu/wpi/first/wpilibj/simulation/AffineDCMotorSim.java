// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.wpilibj.simulation;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.AffineSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/** Represents a simulated DC motor mechanism. */
public class AffineDCMotorSim extends AffineSystemSim<N2, N1, N2> {
  // Gearbox for the DC motor.
  private final DCMotor m_gearbox;

  // The gearing from the motors to the output.
  private final double m_gearing;

  // The static friction gain.
  private final Matrix<N2, N1> m_kS;

  /**
   * Creates a simulated DC motor mechanism.
   *
   * @param plant   The affine system representing the DC motor.
   * @param kS      The static friction gain.
   * @param gearbox The type of and number of motors in the DC motor gearbox.
   * @param gearing The gearing of the DC motor (numbers greater than 1 represent
   *                reductions).
   */
  public AffineDCMotorSim(AffineSystem<N2, N1, N2> plant, double kS, DCMotor gearbox, double gearing) {
    super(plant);
    m_kS = new Matrix<>(Nat.N2(), Nat.N1(), new double[] { 0, -kS / plant.getB().get(1, 0) });
    m_gearbox = gearbox;
    m_gearing = gearing;
  }

  /**
   * Creates a simulated DC motor mechanism.
   *
   * @param plant              The affine system representing the DC motor. This
   *                           system can be created with
   * @param gearbox            The type of and number of motors in the DC motor
   *                           gearbox.
   * @param gearing            The gearing of the DC motor (numbers greater than 1
   *                           represent reductions).
   * @param measurementStdDevs The standard deviations of the measurements.
   */
  public AffineDCMotorSim(
      AffineSystem<N2, N1, N2> plant,
      double kS,
      DCMotor gearbox,
      double gearing,
      Matrix<N2, N1> measurementStdDevs) {
    super(plant, measurementStdDevs);
    m_kS = new Matrix<>(Nat.N2(), Nat.N1(), new double[] { 0, -kS / plant.getB().get(1, 0) });
    m_gearbox = gearbox;
    m_gearing = gearing;
  }

  /**
   * Sets the state of the DC motor.
   *
   * @param angularPositionRad       The new position in radians.
   * @param angularVelocityRadPerSec The new velocity in radians per second.
   */
  public void setState(double angularPositionRad, double angularVelocityRadPerSec) {
    setState(VecBuilder.fill(angularPositionRad, angularVelocityRadPerSec));
  }

  /**
   * Returns the DC motor position.
   *
   * @return The DC motor position.
   */
  public double getAngularPositionRad() {
    return getOutput(0);
  }

  /**
   * Returns the DC motor position in rotations.
   *
   * @return The DC motor position in rotations.
   */
  public double getAngularPositionRotations() {
    return Units.radiansToRotations(getAngularPositionRad());
  }

  /**
   * Returns the DC motor velocity.
   *
   * @return The DC motor velocity.
   */
  public double getAngularVelocityRadPerSec() {
    return getOutput(1);
  }

  /**
   * Returns the DC motor velocity in RPM.
   *
   * @return The DC motor velocity in RPM.
   */
  public double getAngularVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(getAngularVelocityRadPerSec());
  }

  /**
   * Returns the DC motor current draw.
   *
   * @return The DC motor current draw.
   */
  @Override
  public double getCurrentDrawAmps() {
    // I = V / R - omega / (Kv * R)
    // Reductions are output over input, so a reduction of 2:1 means the motor is
    // spinning
    // 2x faster than the output
    return m_gearbox.getCurrent(getAngularVelocityRadPerSec() * m_gearing, m_u.get(0, 0))
        * Math.signum(m_u.get(0, 0));
  }

  /**
   * Sets the input voltage for the DC motor.
   *
   * @param volts The input voltage.
   */
  public void setInputVoltage(double volts) {
    setInput(volts);
  }

  /**
   * Updates the simulation.
   *
   * @param dtSeconds The time between updates.
   */
  public void update(double dtSeconds) {
    if (m_x.get(1, 0) > 0.0) {
      super.update(m_kS, dtSeconds);
    } else if (m_x.get(1, 0) < 0.0) {
      super.update(m_kS.times(-1.0), dtSeconds);
    } else {
      super.update(m_kS.times(0.0), dtSeconds);
    }
  }
}
