// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.wpilibj.simulation;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearAcceleration;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.wpilibj.RobotController;

/** Represents a simulated elevator mechanism. */
public class ElevatorSim extends LinearSystemSim<N2, N1, N2> {
  // Gearbox for the elevator.
  private final DCMotor m_gearbox;

  // The radius of the drum that the elevator spool is wrapped around.
  private final double m_drumRadius;

  // The gearing from the motors to the output.
  private final double m_gearing;

  // The effective mass for the Elevator mechanism.
  private final double m_massKg;

  // The min allowable height for the elevator.
  private final double m_minHeight;

  // The max allowable height for the elevator.
  private final double m_maxHeight;

  // Whether the simulator should simulate gravity.
  private final boolean m_simulateGravity;

  // The gravitational acceleration.
  private final double m_g;

  // The position of the system.
  private final MutDistance m_position = Meters.mutable(0.0);

  // The velocity of the system.
  private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0.0);

  // The acceleration of the system.
  private final MutLinearAcceleration m_acceleration = MetersPerSecondPerSecond.mutable(0.0);

  /**
   * Creates a simulated elevator mechanism.
   *
   * @param kV The velocity gain.
   * @param kA The acceleration gain.
   * @param kG The gravity gain.
   * @param gearbox The type of and number of motors in the elevator gearbox.
   * @param drumRadiusMeters The radius of the drum that the elevator spool is wrapped around.
   * @param minHeightMeters The min allowable height of the elevator.
   * @param maxHeightMeters The max allowable height of the elevator.
   * @param startingHeightMeters The starting height of the elevator.
   * @param measurementStdDevs The standard deviations of the measurements. Can be omitted if no
   *     noise is desired. If present must have 1 element for position.
   */
  public ElevatorSim(
      double kV,
      double kA,
      double kG,
      DCMotor gearbox,
      double drumRadiusMeters,
      double minHeightMeters,
      double maxHeightMeters,
      double startingHeightMeters,
      double... measurementStdDevs) {
    super(LinearSystemId.identifyPositionSystem(kV, kA), measurementStdDevs);
    m_gearbox = gearbox;
    m_drumRadius = drumRadiusMeters;

    // By theorem 6.9.1 of
    // https://file.tavsys.net/control/controls-engineering-in-frc.pdf,
    // the Elevator state-space model is:
    //
    // dx/dt = -G²Kₜ/(Rr²mKᵥ)x + (GKₜ)/(Rrm)u
    // A = -G²Kₜ/(Rr²mKᵥ)
    // B = (GKₜ)/(Rrm)
    //
    // Solve for G.
    //
    // A/B = -G/rKᵥ
    // G = -KᵥrA/B
    //
    // Solve for m.
    //
    // B = (GKₜ)/(Rrm)
    // m = (GKₜ)/(RrB)
    // -kG / kA = g
    m_gearing = -gearbox.KvRadPerSecPerVolt * m_plant.getA(1, 1) / m_plant.getB(1, 0);
    m_massKg = m_gearing * gearbox.KtNMPerAmp / gearbox.rOhms / m_drumRadius / m_plant.getB(1, 0);
    m_g = -kG / kA;
    m_minHeight = minHeightMeters;
    m_maxHeight = maxHeightMeters;
    m_simulateGravity = true;
    setState(startingHeightMeters, 0);
  }

  /**
   * Creates a simulated elevator mechanism.
   *
   * @param gearbox The type of and number of motors in the elevator gearbox.
   * @param gearing The gearing of the elevator (numbers greater than 1 represent reductions).
   * @param carriageMassKg The mass of the elevator carriage.
   * @param drumRadiusMeters The radius of the drum that the elevator spool is wrapped around.
   * @param minHeightMeters The min allowable height of the elevator.
   * @param maxHeightMeters The max allowable height of the elevator.
   * @param simulateGravity Whether gravity should be simulated or not.
   * @param startingHeightMeters The starting height of the elevator.
   * @param measurementStdDevs The standard deviations of the measurements. Can be omitted if no
   *     noise is desired. If present must have 1 element for position.
   */
  public ElevatorSim(
      DCMotor gearbox,
      double gearing,
      double carriageMassKg,
      double drumRadiusMeters,
      double minHeightMeters,
      double maxHeightMeters,
      boolean simulateGravity,
      double startingHeightMeters,
      double... measurementStdDevs) {
    super(
        LinearSystemId.createElevatorSystem(gearbox, carriageMassKg, drumRadiusMeters, gearing),
        measurementStdDevs);
    m_gearbox = gearbox;
    m_gearing = gearing;
    m_drumRadius = drumRadiusMeters;
    m_massKg = carriageMassKg;
    m_minHeight = minHeightMeters;
    m_maxHeight = maxHeightMeters;
    m_simulateGravity = simulateGravity;
    m_g = -9.8;
    setState(startingHeightMeters, 0);
  }

  /**
   * Sets the elevator's state. The new position will be limited between the minimum and maximum
   * allowed heights.
   *
   * @param positionMeters The new position in meters.
   * @param velocityMetersPerSecond New velocity in meters per second.
   */
  public final ElevatorSim setState(double positionMeters, double velocityMetersPerSecond) {
    setState(
        VecBuilder.fill(
            MathUtil.clamp(positionMeters, m_minHeight, m_maxHeight), velocityMetersPerSecond));
    return this;
  }

  /**
   * Sets the elevator's position.
   *
   * @param positionMeters The new position in meters.
   * @return this simulated elevator with applied position.
   */
  public ElevatorSim setPosition(double positionMeters) {
    return setState(positionMeters, getVelocityMetersPerSecond());
  }

  /**
   * Sets the elevator's velocity.
   *
   * @param velocityMetersPerSecond The new velocity in meters per second.
   * @return this simulated elevator with applied velocity.
   */
  public ElevatorSim setVelocity(double velocityMetersPerSecond) {
    return setState(getPositionMeters(), velocityMetersPerSecond);
  }

  /**
   * Returns whether the elevator would hit the lower limit.
   *
   * @param elevatorHeightMeters The elevator height.
   * @return Whether the elevator would hit the lower limit.
   */
  public boolean wouldHitLowerLimit(double elevatorHeightMeters) {
    return elevatorHeightMeters <= this.m_minHeight;
  }

  /**
   * Returns whether the elevator would hit the upper limit.
   *
   * @param elevatorHeightMeters The elevator height.
   * @return Whether the elevator would hit the upper limit.
   */
  public boolean wouldHitUpperLimit(double elevatorHeightMeters) {
    return elevatorHeightMeters >= this.m_maxHeight;
  }

  /**
   * Returns whether the elevator has hit the lower limit.
   *
   * @return Whether the elevator has hit the lower limit.
   */
  public boolean hasHitLowerLimit() {
    return wouldHitLowerLimit(getPositionMeters());
  }

  /**
   * Returns whether the elevator has hit the upper limit.
   *
   * @return Whether the elevator has hit the upper limit.
   */
  public boolean hasHitUpperLimit() {
    return wouldHitUpperLimit(getPositionMeters());
  }

  /**
   * Returns the gear ratio of the elevator.
   *
   * @return the elevator's gear ratio.
   */
  public double getGearing() {
    return m_gearing;
  }

  /**
   * Returns the mass of the elevator.
   *
   * @return the elevator's mass in Kilograms.
   */
  public double getMassKg() {
    return m_massKg;
  }

  /**
   * Returns the drum radius for the elevator.
   *
   * @return The elevator's drum radius in meters.
   */
  public double getDrumRadiusMeters() {
    return m_drumRadius;
  }

  /**
   * Returns the gearbox for the elevator.
   *
   * @return The elevator's gearbox.
   */
  public DCMotor getGearBox() {
    return m_gearbox;
  }

  /**
   * Returns the position of the elevator in meters.
   *
   * @return The position of the elevator in meters.
   */
  public double getPositionMeters() {
    return getOutput(0);
  }

  /**
   * Returns the position of the elevator.
   *
   * @return The elevator's position
   */
  public Distance getPosition() {
    m_position.mut_setMagnitude(getPositionMeters());
    return m_position;
  }

  /**
   * Returns the velocity of the elevator in meters per second.
   *
   * @return The velocity of the elevator in meters per second.
   */
  public double getVelocityMetersPerSecond() {
    return getOutput(1);
  }

  /**
   * Returns the velocity of the elevator.
   *
   * @return The velocity of the elevator.
   */
  public LinearVelocity getVelocity() {
    m_velocity.mut_setMagnitude(getVelocityMetersPerSecond());
    return m_velocity;
  }

  /**
   * Returns the elevator's acceleration io meters per second squared.
   *
   * @return The elevator's acceleration io meters per second squared.
   */
  public double getAccelerationMetersPerSecondSquared() {
    var acceleration = (m_plant.getA().times(m_x)).plus(m_plant.getB().times(m_u));
    return acceleration.get(0, 0);
  }

  /**
   * Returns the elevator's acceleration.
   *
   * @return The elevator's acceleration.
   */
  public LinearAcceleration getAcceleration() {
    m_acceleration.mut_setMagnitude(getAccelerationMetersPerSecondSquared());
    return m_acceleration;
  }

  /**
   * Returns the force acting on the elevator in Newtons.
   *
   * @return The DC motor's torque in Newton.
   */
  public double getForceNewtons() {
    return getAccelerationMetersPerSecondSquared() * m_massKg;
  }

  /**
   * Returns the elevator current draw.
   *
   * @return The elevator current draw.
   */
  public double getCurrentDrawAmps() {
    // I = V / R - omega / (Kv * R)
    // Reductions are greater than 1, so a reduction of 10:1 would mean the motor is
    // spinning 10x faster than the output
    // v = r w, so w = v/r
    return m_gearbox.getCurrent(m_x.get(1, 0) * m_gearing / m_drumRadius, m_u.get(0, 0))
        * Math.signum(m_u.get(0, 0));
  }

  /**
   * Gets the input voltage for the elevator.
   *
   * @return The elevator's input voltage.
   */
  public double getInputVoltage() {
    return getInput(0);
  }

  /**
   * Sets the input voltage for the elevator.
   *
   * @param volts The input voltage.
   */
  public void setInputVoltage(double volts) {
    setInput(volts);
    clampInput(RobotController.getBatteryVoltage());
  }

  /**
   * Updates the state of the elevator.
   *
   * @param currentXhat The current state estimate.
   * @param u The system inputs (voltage).
   * @param dtSeconds The time difference between controller updates.
   */
  @Override
  protected Matrix<N2, N1> updateX(Matrix<N2, N1> currentXhat, Matrix<N1, N1> u, double dtSeconds) {
    // Calculate updated x-hat from Runge-Kutta.
    var updatedXhat =
        NumericalIntegration.rkdp(
            (x, _u) -> {
              Matrix<N2, N1> xdot = m_plant.getA().times(x).plus(m_plant.getB().times(_u));
              if (m_simulateGravity) {
                xdot = xdot.plus(VecBuilder.fill(0, m_g));
              }
              return xdot;
            },
            currentXhat,
            u,
            dtSeconds);

    // We check for collisions after updating x-hat.
    if (wouldHitLowerLimit(updatedXhat.get(0, 0))) {
      return VecBuilder.fill(m_minHeight, 0);
    }
    if (wouldHitUpperLimit(updatedXhat.get(0, 0))) {
      return VecBuilder.fill(m_maxHeight, 0);
    }
    return updatedXhat;
  }
}
