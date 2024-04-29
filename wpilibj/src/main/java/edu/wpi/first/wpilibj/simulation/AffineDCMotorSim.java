// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.wpilibj.simulation;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.AffineSystem;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;

/** Represents a simulated DC motor mechanism. */
public class AffineDCMotorSim {
  /** The plant that represents the positive velocity affine system. */
  protected final AffineSystem<N2, N1, N2> positive_velocity_plant;

  /** The plant that represents the negative velocity affine system. */
  protected final AffineSystem<N2, N1, N2> negative_velocity_plant;

  /** The plant that represents the negative velocity affine system. */
  protected final AffineSystem<N2, N1, N2> zero_velocity_plant;

  /** State vector. */
  protected Matrix<N2, N1> m_x;

  /** Input vector. */
  protected Matrix<N1, N1> m_u;

  /** Output vector. */
  protected Matrix<N2, N1> m_y;

  /**
   * The standard deviations of measurements, used for adding noise to the
   * measurements.
   */
  protected final Matrix<N2, N1> m_measurementStdDevs;

  // Gearbox for the DC motor.
  private final DCMotor m_gearbox;

  // The gearing from the motors to the output.
  private final double m_gearing;

  /**
   * Creates a simulated DC motor mechanism.
   *
   * @param plant   The linear system representing the DC motor. This system can
   *                be created with
   *                {@link edu.wpi.first.math.system.plant.LinearSystemId#createDCMotorSystem(DCMotor, double,
   *                double)}.
   * @param gearbox The type of and number of motors in the DC motor gearbox.
   * @param gearing The gearing of the DC motor (numbers greater than 1 represent
   *                reductions).
   */
  public AffineDCMotorSim(LinearSystem<N2, N1, N2> plant, double kS, DCMotor gearbox, double gearing) {
    this(
        plant,
        gearbox,
        gearing,
        kS,
        null);
  }

  /**
   * Creates a simulated DC motor mechanism.
   *
   * @param plant              The linear system representing the DC motor. This
   *                           system can be created with
   * @param gearbox            The type of and number of motors in the DC motor
   *                           gearbox.
   * @param gearing            The gearing of the DC motor (numbers greater than 1
   *                           represent reductions).
   * @param measurementStdDevs The standard deviations of the measurements.
   */
  public AffineDCMotorSim(
      LinearSystem<N2, N1, N2> plant,
      DCMotor gearbox,
      double gearing,
      double kS,
      Matrix<N2, N1> measurementStdDevs) {
    positive_velocity_plant = new AffineSystem<>(
        plant.getA(),
        plant.getB(),
        plant.getC(),
        plant.getD(),
        new Matrix<>(Nat.N2(), Nat.N1(), new double[] { 0.0, -kS / plant.getB(1, 0) }));
    negative_velocity_plant = new AffineSystem<>(
        plant.getA(),
        plant.getB(),
        plant.getC(),
        plant.getD(),
        new Matrix<>(Nat.N2(), Nat.N1(), new double[] { 0.0, kS / plant.getB(1, 0) }));
    zero_velocity_plant = new AffineSystem<>(
        plant.getA(),
        plant.getB(),
        plant.getC(),
        plant.getD(),
        new Matrix<>(Nat.N2(), Nat.N1(), new double[] { 0.0, 0.0 }));
    m_gearbox = gearbox;
    m_gearing = gearing;
    m_measurementStdDevs = measurementStdDevs;

    m_x = new Matrix<>(new SimpleMatrix(plant.getA().getNumRows(), 1));
    m_u = new Matrix<>(new SimpleMatrix(plant.getB().getNumCols(), 1));
    m_y = new Matrix<>(new SimpleMatrix(plant.getC().getNumRows(), 1));

  }

  /**
   * Creates a simulated DC motor mechanism.
   *
   * @param gearbox          The type of and number of motors in the DC motor
   *                         gearbox.
   * @param gearing          The gearing of the DC motor (numbers greater than 1
   *                         represent reductions).
   * @param jKgMetersSquared The moment of inertia of the DC motor. If this is
   *                         unknown, use the
   *                         {@link #AffineDCMotorSim(LinearSystem, DCMotor, double, Matrix)}
   *                         constructor.
   */
  public AffineDCMotorSim(DCMotor gearbox, double gearing, double staticFriction, double jKgMetersSquared) {
    this(
        gearbox,
        gearing,
        staticFriction,
        jKgMetersSquared,
        null);
  }

  /**
   * Creates a simulated DC motor mechanism.
   *
   * @param gearbox            The type of and number of motors in the DC motor
   *                           gearbox.
   * @param gearing            The gearing of the DC motor (numbers greater than 1
   *                           represent reductions).
   * @param jKgMetersSquared   The moment of inertia of the DC motor. If this is
   *                           unknown, use the
   *                           {@link #AffineDCMotorSim(LinearSystem, DCMotor, double, Matrix)}
   *                           constructor.
   * @param measurementStdDevs The standard deviations of the measurements.
   */
  public AffineDCMotorSim(
      DCMotor gearbox, double gearing, double staticFriction, double jKgMetersSquared,
      Matrix<N2, N1> measurementStdDevs) {
    var plant = LinearSystemId.createDCMotorSystem(gearbox, jKgMetersSquared, gearing);
    positive_velocity_plant = new AffineSystem<>(
        plant.getA(),
        plant.getB(),
        plant.getC(),
        plant.getD(),
        new Matrix<>(Nat.N2(), Nat.N1(), new double[] { 0.0, -staticFriction / plant.getB(1, 0) }));
    negative_velocity_plant = new AffineSystem<>(
        plant.getA(),
        plant.getB(),
        plant.getC(),
        plant.getD(),
        new Matrix<>(Nat.N2(), Nat.N1(), new double[] { 0.0, staticFriction / plant.getB(1, 0) }));
    zero_velocity_plant = new AffineSystem<>(
        plant.getA(),
        plant.getB(),
        plant.getC(),
        plant.getD(),
        new Matrix<>(Nat.N2(), Nat.N1(), new double[] { 0.0, 0.0 }));
    m_gearbox = gearbox;
    m_gearing = gearing;
    m_measurementStdDevs = measurementStdDevs;

    m_x = new Matrix<>(new SimpleMatrix(plant.getA().getNumRows(), 1));
    m_u = new Matrix<>(new SimpleMatrix(plant.getB().getNumCols(), 1));
    m_y = new Matrix<>(new SimpleMatrix(plant.getC().getNumRows(), 1));
  }

  /**
   * Sets the state of the DC motor.
   *
   * @param angularPositionRad       The new position in radians.
   * @param angularVelocityRadPerSec The new velocity in radians per second.
   */
  public void setState(double angularPositionRad, double angularVelocityRadPerSec) {
    m_x = VecBuilder.fill(angularPositionRad, angularVelocityRadPerSec);
  }

  /**
   * Returns the DC motor position.
   *
   * @return The DC motor position.
   */
  public double getAngularPositionRad() {
    return m_y.get(0, 0);
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
    return m_y.get(1, 0);
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
  public double getCurrentDrawAmps() {
    // I = V / R - omega / (Kv * R)
    // Reductions are output over input, so a reduction of 2:1 means the motor is
    // spinning
    // 2x faster than the output
    return m_gearbox.getCurrent(m_x.get(1, 0) * m_gearing, m_u.get(0, 0))
        * Math.signum(m_u.get(0, 0));
  }

  /**
   * Sets the input voltage for the DC motor.
   *
   * @param volts The input voltage.
   */
  public void setInputVoltage(double volts) {
    m_u = new Matrix<>(Nat.N1(), Nat.N1(), new double[] { volts });
    clampInput();

  }

  /**
   * Clamp the input vector such that no element exceeds the given voltage. If any
   * does, the
   * relative magnitudes of the input will be maintained.
   *
   * @param u The input vector.
   * @return The normalized input.
   */
  protected void clampInput() {
    m_u = StateSpaceUtil.desaturateInputVector(m_u, RobotController.getBatteryVoltage());
  }

  /**
   * Updates the simulation.
   *
   * @param dtSeconds The time between updates.
   */
  public void update(double dtSeconds) {
    // Update X. By default, this is the affine system dynamics X = Ax + Bu + c
    if (m_x.get(1, 0) < 0.0) {
      m_x = negative_velocity_plant.calculateX(m_x, m_u, dtSeconds);
    } else if (m_x.get(1, 0) > 0.0) {
      m_x = positive_velocity_plant.calculateX(m_x, m_u, dtSeconds);
    } else {
      m_x = zero_velocity_plant.calculateX(m_x, m_u, dtSeconds);
    }

    // y = cx + du
    m_y = zero_velocity_plant.calculateY(m_x, m_u);

    // Add measurement noise.
    if (m_measurementStdDevs != null) {
      m_y = m_y.plus(StateSpaceUtil.makeWhiteNoiseVector(m_measurementStdDevs));
    }
  }
}
