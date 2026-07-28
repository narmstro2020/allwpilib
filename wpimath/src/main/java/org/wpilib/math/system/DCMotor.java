// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.wpilib.math.system;

import org.wpilib.math.system.proto.DCMotorProto;
import org.wpilib.math.system.struct.DCMotorStruct;
import org.wpilib.math.util.Units;
import org.wpilib.util.protobuf.ProtobufSerializable;
import org.wpilib.util.struct.StructSerializable;

/**
 * Holds the constants for a single DC motor.
 *
 * <p>This describes the motor itself and nothing about how it is installed. To model a number of
 * these motors driving a mechanism through a gear reduction, wrap this in a {@link Gearbox}.
 */
public class DCMotor implements ProtobufSerializable, StructSerializable {
  /** Voltage at which the motor constants were measured. */
  public final double nominalVoltage;

  /** Torque when stalled in Newton-meters. */
  public final double stallTorque;

  /** Current draw when stalled in amps. */
  public final double stallCurrent;

  /** Current draw under no load in amps. */
  public final double freeCurrent;

  /** Angular velocity under no load in radians per second. */
  public final double freeSpeed;

  /** Motor internal resistance in Ohms. */
  public final double R;

  /** Motor velocity constant in (rad/s)/V. */
  public final double Kv;

  /** Motor torque constant in Newton-meters per amp. */
  public final double Kt;

  /** DCMotor protobuf for serialization. */
  public static final DCMotorProto proto = new DCMotorProto();

  /** DCMotor struct for serialization. */
  public static final DCMotorStruct struct = new DCMotorStruct();

  /** A CIM motor. */
  public static final DCMotor kCIM =
      new DCMotor(12, 2.42, 133, 2.7, Units.rotationsPerMinuteToRadiansPerSecond(5310));

  /** A MiniCIM motor. */
  public static final DCMotor kMiniCIM =
      new DCMotor(12, 1.41, 89, 3, Units.rotationsPerMinuteToRadiansPerSecond(5840));

  /** A Bag motor. */
  public static final DCMotor kBag =
      new DCMotor(12, 0.43, 53, 1.8, Units.rotationsPerMinuteToRadiansPerSecond(13180));

  /** A 775Pro motor. */
  public static final DCMotor kVex775Pro =
      new DCMotor(12, 0.71, 134, 0.7, Units.rotationsPerMinuteToRadiansPerSecond(18730));

  /** An Andymark RS775-125 motor. */
  public static final DCMotor kAndymarkRs775_125 =
      new DCMotor(12, 0.28, 18, 1.6, Units.rotationsPerMinuteToRadiansPerSecond(5800.0));

  /** A Banebots RS775 motor. */
  public static final DCMotor kBanebotsRs775 =
      new DCMotor(12, 0.72, 97, 2.7, Units.rotationsPerMinuteToRadiansPerSecond(13050.0));

  /** An Andymark 9015 motor. */
  public static final DCMotor kAndymark9015 =
      new DCMotor(12, 0.36, 71, 3.7, Units.rotationsPerMinuteToRadiansPerSecond(14270.0));

  /** A Banebots RS 550 motor. */
  public static final DCMotor kBanebotsRs550 =
      new DCMotor(12, 0.38, 84, 0.4, Units.rotationsPerMinuteToRadiansPerSecond(19000.0));

  /** A NEO motor. */
  public static final DCMotor kNEO =
      new DCMotor(12, 2.6, 105, 1.8, Units.rotationsPerMinuteToRadiansPerSecond(5676));

  /** A NEO 550 motor. */
  public static final DCMotor kNeo550 =
      new DCMotor(12, 0.97, 100, 1.4, Units.rotationsPerMinuteToRadiansPerSecond(11000.0));

  /** A Falcon 500 motor. */
  public static final DCMotor kFalcon500 =
      new DCMotor(12, 4.69, 257, 1.5, Units.rotationsPerMinuteToRadiansPerSecond(6380.0));

  /**
   * A Falcon 500 motor with FOC (Field-Oriented Control) enabled.
   *
   * @see <a
   *     href="https://store.ctr-electronics.com/falcon-500-powered-by-talon-fx/">https://store.ctr-electronics.com/falcon-500-powered-by-talon-fx/</a>
   */
  public static final DCMotor kFalcon500Foc =
      new DCMotor(12, 5.84, 304, 1.5, Units.rotationsPerMinuteToRadiansPerSecond(6080.0));

  /**
   * A Romi/TI_RSLK MAX motor.
   *
   * @see <a
   *     href="https://www.pololu.com/product/1520/specs">https://www.pololu.com/product/1520/specs</a>
   */
  public static final DCMotor kRomiBuiltIn =
      new DCMotor(4.5, 0.1765, 1.25, 0.13, Units.rotationsPerMinuteToRadiansPerSecond(150.0));

  /**
   * A Kraken X60 brushless motor.
   *
   * @see <a
   *     href="https://store.ctr-electronics.com/announcing-kraken-x60/">https://store.ctr-electronics.com/announcing-kraken-x60/</a>
   */
  public static final DCMotor kKrakenX60 =
      new DCMotor(12, 7.09, 366, 2, Units.rotationsPerMinuteToRadiansPerSecond(6000));

  /**
   * A Kraken X60 brushless motor with FOC (Field-Oriented Control) enabled.
   *
   * @see <a
   *     href="https://store.ctr-electronics.com/announcing-kraken-x60/">https://store.ctr-electronics.com/announcing-kraken-x60/</a>
   */
  public static final DCMotor kKrakenX60Foc =
      new DCMotor(12, 9.37, 483, 2, Units.rotationsPerMinuteToRadiansPerSecond(5800));

  /**
   * A Kraken X44 brushless motor.
   *
   * @see <a
   *     href="https://motors.ctr-electronics.com/dyno/dynometer-testing/">https://motors.ctr-electronics.com/dyno/dynometer-testing/</a>
   */
  public static final DCMotor kKrakenX44 =
      new DCMotor(12, 4.11, 279, 2, Units.rotationsPerMinuteToRadiansPerSecond(7758));

  /**
   * A Kraken X44 brushless motor with FOC (Field-Oriented Control) enabled.
   *
   * @see <a
   *     href="https://motors.ctr-electronics.com/dyno/dynometer-testing/">https://motors.ctr-electronics.com/dyno/dynometer-testing/</a>
   */
  public static final DCMotor kKrakenX44Foc =
      new DCMotor(12, 5.01, 329, 2, Units.rotationsPerMinuteToRadiansPerSecond(7368));

  /**
   * A Minion brushless motor.
   *
   * @see <a
   *     href="https://motors.ctr-electronics.com/dyno/dynometer-testing/">https://motors.ctr-electronics.com/dyno/dynometer-testing/</a>
   */
  public static final DCMotor kMinion =
      new DCMotor(12, 3.17, 211, 2, Units.rotationsPerMinuteToRadiansPerSecond(7704));

  /**
   * A Neo Vortex brushless motor.
   *
   * @see <a
   *     href="https://www.revrobotics.com/next-generation-spark-neo/">https://www.revrobotics.com/next-generation-spark-neo/</a>
   */
  public static final DCMotor kNeoVortex =
      new DCMotor(12, 3.60, 211, 3.6, Units.rotationsPerMinuteToRadiansPerSecond(6784));

  /**
   * Constructs a DC motor.
   *
   * @param nominalVoltage Voltage at which the motor constants were measured.
   * @param stallTorque Torque when stalled.
   * @param stallCurrent Current draw when stalled.
   * @param freeCurrent Current draw under no load.
   * @param freeSpeed Angular velocity under no load.
   */
  public DCMotor(
      double nominalVoltage,
      double stallTorque,
      double stallCurrent,
      double freeCurrent,
      double freeSpeed) {
    this.nominalVoltage = nominalVoltage;
    this.stallTorque = stallTorque;
    this.stallCurrent = stallCurrent;
    this.freeCurrent = freeCurrent;
    this.freeSpeed = freeSpeed;

    this.R = nominalVoltage / stallCurrent;
    this.Kv = freeSpeed / (nominalVoltage - R * freeCurrent);
    this.Kt = stallTorque / stallCurrent;
  }

  /**
   * Calculate current drawn by the motor with given velocity and input voltage.
   *
   * @param velocity The current angular velocity of the motor in radians per second.
   * @param voltageInput The voltage being applied to the motor.
   * @return The estimated current in amps.
   */
  public double getCurrent(double velocity, double voltageInput) {
    return -1.0 / Kv / R * velocity + 1.0 / R * voltageInput;
  }

  /**
   * Calculate current drawn by the motor for a given torque.
   *
   * @param torque The torque produced by the motor in Newton-meters.
   * @return The current drawn by the motor in amps.
   */
  public double getCurrent(double torque) {
    return torque / Kt;
  }

  /**
   * Calculate torque produced by the motor with a given current.
   *
   * @param current The current drawn by the motor in amps.
   * @return The torque output in Newton-meters.
   */
  public double getTorque(double current) {
    return current * Kt;
  }

  /**
   * Calculate the voltage provided to the motor for a given torque and angular velocity.
   *
   * @param torque The torque produced by the motor in Newton-meters.
   * @param velocity The current angular velocity of the motor in radians per second.
   * @return The voltage of the motor.
   */
  public double getVoltage(double torque, double velocity) {
    return 1.0 / Kv * velocity + 1.0 / Kt * R * torque;
  }

  /**
   * Calculates the angular velocity produced by the motor at a given torque and input voltage.
   *
   * @param torque The torque produced by the motor in Newton-meters.
   * @param voltageInput The voltage applied to the motor.
   * @return The angular velocity of the motor in radians per second.
   */
  public double getVelocity(double torque, double voltageInput) {
    return voltageInput * Kv - 1.0 / Kt * torque * R * Kv;
  }
}
