// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.wpilib.math.system;

import org.wpilib.math.system.proto.GearboxProto;
import org.wpilib.math.system.struct.GearboxStruct;
import org.wpilib.util.protobuf.ProtobufSerializable;
import org.wpilib.util.struct.StructSerializable;

/**
 * Holds a number of identical {@link DCMotor}s driving a common output shaft through a gear
 * reduction.
 *
 * <p>{@link DCMotor} describes the motor alone. This class adds the two things that describe how
 * those motors are installed: how many of them there are, and the reduction between them and the
 * mechanism they drive.
 *
 * <p>All of this class's calculations are performed at the <i>output</i> of the gearbox. Velocities
 * and torques are those of the driven shaft, not of the individual motors, and currents are the
 * total drawn by all the motors together.
 */
public class Gearbox implements ProtobufSerializable, StructSerializable {
  /** The type of motor in the gearbox. */
  public final DCMotor motor;

  /** The number of motors in the gearbox. */
  public final int numMotors;

  /** The gear reduction from the motors to the output, as input over output. */
  public final double reduction;

  /** Gearbox protobuf for serialization. */
  public static final GearboxProto proto = new GearboxProto();

  /** Gearbox struct for serialization. */
  public static final GearboxStruct struct = new GearboxStruct();

  /**
   * Constructs a gearbox.
   *
   * @param motor The type of motor in the gearbox.
   * @param numMotors The number of motors in the gearbox.
   * @param reduction The gear reduction from the motors to the output, as input over output. Values
   *     greater than one are reductions: the output turns slower than the motors, with more torque.
   * @throws IllegalArgumentException if numMotors &lt;= 0 or reduction &lt;= 0.
   */
  public Gearbox(DCMotor motor, int numMotors, double reduction) {
    if (numMotors <= 0) {
      throw new IllegalArgumentException("numMotors must be greater than zero.");
    }
    if (reduction <= 0.0) {
      throw new IllegalArgumentException("reduction must be greater than zero.");
    }

    this.motor = motor;
    this.numMotors = numMotors;
    this.reduction = reduction;
  }

  /**
   * Constructs a direct-drive gearbox, with a reduction of one.
   *
   * @param motor The type of motor in the gearbox.
   * @param numMotors The number of motors in the gearbox.
   * @throws IllegalArgumentException if numMotors &lt;= 0.
   */
  public Gearbox(DCMotor motor, int numMotors) {
    this(motor, numMotors, 1.0);
  }

  /**
   * Returns a copy of this gearbox with an additional reduction applied.
   *
   * @param reduction The reduction to apply on top of this gearbox's existing reduction.
   * @return A gearbox with the combined reduction.
   * @throws IllegalArgumentException if reduction &lt;= 0.
   */
  public Gearbox withReduction(double reduction) {
    return new Gearbox(motor, numMotors, this.reduction * reduction);
  }

  /**
   * Returns the torque the gearbox produces at its output when stalled.
   *
   * @return The stall torque in Newton-meters.
   */
  public double getStallTorque() {
    return motor.stallTorque * numMotors * reduction;
  }

  /**
   * Returns the total current the gearbox draws when stalled.
   *
   * @return The stall current in amps.
   */
  public double getStallCurrent() {
    return motor.stallCurrent * numMotors;
  }

  /**
   * Returns the total current the gearbox draws under no load.
   *
   * @return The free current in amps.
   */
  public double getFreeCurrent() {
    return motor.freeCurrent * numMotors;
  }

  /**
   * Returns the angular velocity of the gearbox's output under no load.
   *
   * @return The free speed in radians per second.
   */
  public double getFreeSpeed() {
    return motor.freeSpeed / reduction;
  }

  /**
   * Calculate the total current drawn by the gearbox with a given output velocity and input
   * voltage.
   *
   * @param velocity The angular velocity of the output in radians per second.
   * @param voltageInput The voltage being applied to the motors.
   * @return The estimated total current in amps.
   */
  public double getCurrent(double velocity, double voltageInput) {
    return numMotors * motor.getCurrent(velocity * reduction, voltageInput);
  }

  /**
   * Calculate the total current drawn by the gearbox for a given output torque.
   *
   * @param torque The torque produced at the output in Newton-meters.
   * @return The total current drawn by the motors in amps.
   */
  public double getCurrent(double torque) {
    return torque / (reduction * motor.Kt);
  }

  /**
   * Calculate the torque produced at the gearbox's output for a given total current.
   *
   * @param current The total current drawn by the motors in amps.
   * @return The torque output in Newton-meters.
   */
  public double getTorque(double current) {
    return current * motor.Kt * reduction;
  }

  /**
   * Calculate the voltage provided to the motors for a given output torque and angular velocity.
   *
   * @param torque The torque produced at the output in Newton-meters.
   * @param velocity The angular velocity of the output in radians per second.
   * @return The voltage of the motors.
   */
  public double getVoltage(double torque, double velocity) {
    return motor.getVoltage(torque / (numMotors * reduction), velocity * reduction);
  }

  /**
   * Calculates the angular velocity produced at the gearbox's output at a given output torque and
   * input voltage.
   *
   * @param torque The torque produced at the output in Newton-meters.
   * @param voltageInput The voltage applied to the motors.
   * @return The angular velocity of the output in radians per second.
   */
  public double getVelocity(double torque, double voltageInput) {
    return motor.getVelocity(torque / (numMotors * reduction), voltageInput) / reduction;
  }
}
