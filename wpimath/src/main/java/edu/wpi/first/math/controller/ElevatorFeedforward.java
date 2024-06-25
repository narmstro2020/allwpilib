// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math.controller;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.proto.ElevatorFeedforwardProto;
import edu.wpi.first.math.controller.struct.ElevatorFeedforwardStruct;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.protobuf.ProtobufSerializable;
import edu.wpi.first.util.struct.StructSerializable;

/**
 * A helper class that computes feedforward outputs for a simple elevator (modeled as a motor acting
 * against the force of gravity).
 */
public class ElevatorFeedforward implements ProtobufSerializable, StructSerializable {
  /** The static gain. */
  public final double ks;

  /** The gravity gain. */
  public final double kg;

  /** The velocity gain. */
  public final double kv;

  /** The acceleration gain. */
  public final double ka;

  /** The period. */
  private double m_dt;

  /** ElevatorFeedforward protobuf for serialization. */
  public static final ElevatorFeedforwardProto proto = new ElevatorFeedforwardProto();

  /** ElevatorFeedforward struct for serialization. */
  public static final ElevatorFeedforwardStruct struct = new ElevatorFeedforwardStruct();

  /**
   * Creates a new ElevatorFeedforward with the specified gains. Units of the gain values will
   * dictate units of the computed feedforward.
   *
   * @param ks The static gain.
   * @param kg The gravity gain.
   * @param kv The velocity gain.
   * @param ka The acceleration gain.
   * @param dtSeconds The period in seconds.
   * @throws IllegalArgumentException for kv &lt; zero.
   * @throws IllegalArgumentException for ka &lt; zero.
   * @throws IllegalArgumentException for period &le; zero.
   */
  public ElevatorFeedforward(double ks, double kg, double kv, double ka, double dtSeconds) {
    this.ks = ks;
    this.kg = kg;
    this.kv = kv;
    this.ka = ka;
    if (kv < 0.0) {
      throw new IllegalArgumentException("kv must be a non-negative number, got " + kv + "!");
    }
    if (ka < 0.0) {
      throw new IllegalArgumentException("ka must be a non-negative number, got " + ka + "!");
    }
    if (dtSeconds <= 0.0) {
      throw new IllegalArgumentException(
          "period must be a positive number, got " + dtSeconds + "!");
    }
    m_dt = dtSeconds;
  }

  /**
   * Creates a new ElevatorFeedforward with the specified gains. The period is defaulted to 20 ms.
   * Units of the gain values will dictate units of the computed feedforward.
   *
   * @param ks The static gain.
   * @param kg The gravity gain.
   * @param kv The velocity gain.
   * @param ka The acceleration gain.
   * @throws IllegalArgumentException for kv &lt; zero.
   * @throws IllegalArgumentException for ka &lt; zero.
   */
  public ElevatorFeedforward(double ks, double kg, double kv, double ka) {
    this(ks, kg, kv, ka, 0.020);
  }

  /**
   * Creates a new ElevatorFeedforward with the specified gains. Acceleration gain is defaulted to
   * zero. The period is defaulted to 20 ms. Units of the gain values will dictate units of the
   * computed feedforward.
   *
   * @param ks The static gain.
   * @param kg The gravity gain.
   * @param kv The velocity gain.
   * @throws IllegalArgumentException for kv &lt; zero.
   */
  public ElevatorFeedforward(double ks, double kg, double kv) {
    this(ks, kg, kv, 0);
  }

  /**
   * Calculates the feedforward from the gains and setpoints.
   *
   * @param velocity The velocity setpoint.
   * @param acceleration The acceleration setpoint.
   * @return The computed feedforward.
   */
  @SuppressWarnings("removal")
  @Deprecated(forRemoval = true, since = "2025")
  public double calculate(double velocity, double acceleration) {
    return ks * Math.signum(velocity) + kg + kv * velocity + ka * acceleration;
  }

  /**
   * Calculates the feedforward from the gains and velocity setpoint (acceleration is assumed to be
   * zero).
   *
   * @param velocity The velocity setpoint.
   * @return The computed feedforward.
   */
  @SuppressWarnings("removal")
  @Deprecated(forRemoval = true, since = "2025")
  public double calculate(double velocity) {
    return calculate(velocity, 0);
  }

  /**
   * Calculates the feedforward from the gains and setpoints assuming discrete control when the
   * setpoint does not change.
   *
   * @param <U> The velocity parameter either as distance or angle.
   * @param setpoint The velocity setpoint.
   * @return The computed feedforward.
   */
  public <U extends Unit<U>> Measure<Voltage> calculate(Measure<Velocity<U>> setpoint) {
    return calculate(setpoint, setpoint);
  }

  /**
   * Calculates the feedforward from the gains and setpoints.
   *
   * <p>Note this method is inaccurate when the velocity crosses 0.
   *
   * @param <U> The velocity parameter either as distance or angle.
   * @param currentVelocity The current velocity setpoint.
   * @param nextVelocity The next velocity setpoint.
   * @return The computed feedforward.
   */
  public <U extends Unit<U>> Measure<Voltage> calculate(
      Measure<Velocity<U>> currentVelocity, Measure<Velocity<U>> nextVelocity) {
    if (ka == 0.0) {
      // Given the following discrete feedforward model
      //
      //   uₖ = B_d⁺(rₖ₊₁ − A_d rₖ)
      //
      // where
      //
      //   A_d = eᴬᵀ
      //   B_d = A⁻¹(eᴬᵀ - I)B
      //   A = −kᵥ/kₐ
      //   B = 1/kₐ
      //
      // We want the feedforward model when kₐ = 0.
      //
      // Simplify A.
      //
      //   A = −kᵥ/kₐ
      //
      // As kₐ approaches zero, A approaches -∞.
      //
      //   A = −∞
      //
      // Simplify A_d.
      //
      //   A_d = eᴬᵀ
      //   A_d = exp(−∞)
      //   A_d = 0
      //
      // Simplify B_d.
      //
      //   B_d = A⁻¹(eᴬᵀ - I)B
      //   B_d = A⁻¹((0) - I)B
      //   B_d = A⁻¹(-I)B
      //   B_d = -A⁻¹B
      //   B_d = -(−kᵥ/kₐ)⁻¹(1/kₐ)
      //   B_d = (kᵥ/kₐ)⁻¹(1/kₐ)
      //   B_d = kₐ/kᵥ(1/kₐ)
      //   B_d = 1/kᵥ
      //
      // Substitute these into the feedforward equation.
      //
      //   uₖ = B_d⁺(rₖ₊₁ − A_d rₖ)
      //   uₖ = (1/kᵥ)⁺(rₖ₊₁ − (0) rₖ)
      //   uₖ = kᵥrₖ₊₁
      return Volts.of(
          kg + ks * Math.signum(nextVelocity.magnitude()) + kv * nextVelocity.magnitude());
    } else {
      //   uₖ = B_d⁺(rₖ₊₁ − A_d rₖ)
      //
      // where
      //
      //   A_d = eᴬᵀ
      //   B_d = A⁻¹(eᴬᵀ - I)B
      //   A = −kᵥ/kₐ
      //   B = 1/kₐ
      double A = -kv / ka;
      double B = 1.0 / ka;
      double A_d = Math.exp(A * m_dt);
      double B_d = 1.0 / A * (A_d - 1.0) * B;
      return Volts.of(
          kg
              + ks * Math.signum(currentVelocity.magnitude())
              + 1.0 / B_d * (nextVelocity.magnitude() - A_d * currentVelocity.magnitude()));
    }
  }

  // Rearranging the main equation from the calculate() method yields the
  // formulas for the methods below:

  /**
   * Calculates the maximum achievable velocity given a maximum voltage supply and an acceleration.
   * Useful for ensuring that velocity and acceleration constraints for a trapezoidal profile are
   * simultaneously achievable - enter the acceleration constraint, and this will give you a
   * simultaneously-achievable velocity constraint.
   *
   * @param maxVoltage The maximum voltage that can be supplied to the elevator.
   * @param acceleration The acceleration of the elevator.
   * @return The maximum possible velocity at the given acceleration.
   */
  public double maxAchievableVelocity(double maxVoltage, double acceleration) {
    // Assume max velocity is positive
    return (maxVoltage - ks - kg - acceleration * ka) / kv;
  }

  /**
   * Calculates the minimum achievable velocity given a maximum voltage supply and an acceleration.
   * Useful for ensuring that velocity and acceleration constraints for a trapezoidal profile are
   * simultaneously achievable - enter the acceleration constraint, and this will give you a
   * simultaneously-achievable velocity constraint.
   *
   * @param maxVoltage The maximum voltage that can be supplied to the elevator.
   * @param acceleration The acceleration of the elevator.
   * @return The minimum possible velocity at the given acceleration.
   */
  public double minAchievableVelocity(double maxVoltage, double acceleration) {
    // Assume min velocity is negative, ks flips sign
    return (-maxVoltage + ks - kg - acceleration * ka) / kv;
  }

  /**
   * Calculates the maximum achievable acceleration given a maximum voltage supply and a velocity.
   * Useful for ensuring that velocity and acceleration constraints for a trapezoidal profile are
   * simultaneously achievable - enter the velocity constraint, and this will give you a
   * simultaneously-achievable acceleration constraint.
   *
   * @param maxVoltage The maximum voltage that can be supplied to the elevator.
   * @param velocity The velocity of the elevator.
   * @return The maximum possible acceleration at the given velocity.
   */
  public double maxAchievableAcceleration(double maxVoltage, double velocity) {
    return (maxVoltage - ks * Math.signum(velocity) - kg - velocity * kv) / ka;
  }

  /**
   * Calculates the minimum achievable acceleration given a maximum voltage supply and a velocity.
   * Useful for ensuring that velocity and acceleration constraints for a trapezoidal profile are
   * simultaneously achievable - enter the velocity constraint, and this will give you a
   * simultaneously-achievable acceleration constraint.
   *
   * @param maxVoltage The maximum voltage that can be supplied to the elevator.
   * @param velocity The velocity of the elevator.
   * @return The minimum possible acceleration at the given velocity.
   */
  public double minAchievableAcceleration(double maxVoltage, double velocity) {
    return maxAchievableAcceleration(-maxVoltage, velocity);
  }
}
