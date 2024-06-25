// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <wpi/MathExtras.h>

#include "frc/EigenCore.h"
#include "frc/controller/LinearPlantInversionFeedforward.h"
#include "frc/system/plant/LinearSystemId.h"
#include "units/length.h"
#include "units/time.h"
#include "units/voltage.h"
#include "wpimath/MathShared.h"

namespace frc {
/**
 * A helper class that computes feedforward outputs for a simple elevator
 * (modeled as a motor acting against the force of gravity).
 */
class ElevatorFeedforward {
 public:
  using Distance = units::meters;
  using Velocity =
      units::compound_unit<Distance, units::inverse<units::seconds>>;
  using Acceleration =
      units::compound_unit<Velocity, units::inverse<units::seconds>>;
  using kv_unit = units::compound_unit<units::volts, units::inverse<Velocity>>;
  using ka_unit =
      units::compound_unit<units::volts, units::inverse<Acceleration>>;

  /**
   * Creates a new ElevatorFeedforward with the specified gains.
   *
   * @param kS The static gain, in volts.
   * @param kG The gravity gain, in volts.
   * @param kV The velocity gain, in volt seconds per distance.
   * @param kA The acceleration gain, in volt seconds² per distance.
   * @param dt The period in seconds.
   */
  constexpr ElevatorFeedforward(
      units::volt_t kS, units::volt_t kG, units::unit_t<kv_unit> kV,
      units::unit_t<ka_unit> kA = units::unit_t<ka_unit>(0),
      units::second_t dt = 20_ms)
      : kS(kS),
        kG(kG),
        kV([&] {
          if (kV.value() < 0) {
            wpi::math::MathSharedStore::ReportError(
                "kV must be a non-negative number, got {}!", kV.value());
            wpi::math::MathSharedStore::ReportWarning("kV defaulted to 0.");
            return units::unit_t<kv_unit>{0};
          } else {
            return kV;
          }
        }()),
        kA([&] {
          if (kA.value() < 0) {
            wpi::math::MathSharedStore::ReportError(
                "kA must be a non-negative number, got {}!", kA.value());
            wpi::math::MathSharedStore::ReportWarning("kA defaulted to 0.");
            return units::unit_t<ka_unit>{0};
          } else {
            return kA;
          }
        }()) {
    if (dt <= 0_ms) {
      wpi::math::MathSharedStore::ReportError(
          "period must be a positive number, got {}!", dt.value());
      m_dt = 20_ms;
      wpi::math::MathSharedStore::ReportWarning("period defaulted to 20 ms.");
    } else {
      m_dt = dt;
    }
  }

  /**
   * Calculates the feedforward from the gains and setpoints.
   *
   * @param velocity     The velocity setpoint, in distance per second.
   * @param acceleration The acceleration setpoint, in distance per second².
   * @return The computed feedforward, in volts.
   * @deprecated Use the current/next velocity overload instead.
   */
  [[deprecated("Use the current/next velocity overload instead.")]]
  constexpr units::volt_t Calculate(
      units::unit_t<Velocity> velocity,
      units::unit_t<Acceleration> acceleration) const {
    return kS * wpi::sgn(velocity) + kG + kV * velocity + kA * acceleration;
  }

  /**
   * Calculates the feedforward from the gains and setpoint.
   * Use this method when the setpoint does not change.
   *
   * @param setpoint The velocity setpoint, in distance per
   *                        second.
   * @return The computed feedforward, in volts.
   */
  constexpr units::volt_t Calculate(units::unit_t<Velocity> setpoint) const {
    return Calculate(setpoint, setpoint);
  }

  /**
   * Calculates the feedforward from the gains and setpoints.
   *
   * @param currentVelocity The current velocity setpoint, in distance per
   *                        second.
   * @param nextVelocity    The next velocity setpoint, in distance per second.
   * @return The computed feedforward, in volts.
   */
  constexpr units::volt_t Calculate(
      units::unit_t<Velocity> currentVelocity,
      units::unit_t<Velocity> nextVelocity) const {
    if (kA == decltype(kA)(0)) {
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
      //   A_d = std::exp(−∞)
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
      return kG + kS * wpi::sgn(nextVelocity) + kV * nextVelocity;
    } else {
      //   uₖ = B_d⁺(rₖ₊₁ − A_d rₖ)
      //
      // where
      //
      //   A_d = eᴬᵀ
      //   B_d = A⁻¹(eᴬᵀ - I)B
      //   A = −kᵥ/kₐ
      //   B = 1/kₐ
      double A = -kV.value() / kA.value();
      double B = 1.0 / kA.value();
      double A_d = gcem::exp(A * m_dt.value());
      double B_d = 1.0 / A * (A_d - 1.0) * B;
      return kG + kS * wpi::sgn(currentVelocity) +
             units::volt_t{
                 1.0 / B_d *
                 (nextVelocity.value() - A_d * currentVelocity.value())};
    }
  }

  // Rearranging the main equation from the calculate() method yields the
  // formulas for the methods below:

  /**
   * Calculates the maximum achievable velocity given a maximum voltage supply
   * and an acceleration.  Useful for ensuring that velocity and
   * acceleration constraints for a trapezoidal profile are simultaneously
   * achievable - enter the acceleration constraint, and this will give you
   * a simultaneously-achievable velocity constraint.
   *
   * @param maxVoltage The maximum voltage that can be supplied to the elevator.
   * @param acceleration The acceleration of the elevator.
   * @return The maximum possible velocity at the given acceleration.
   */
  constexpr units::unit_t<Velocity> MaxAchievableVelocity(
      units::volt_t maxVoltage, units::unit_t<Acceleration> acceleration) {
    // Assume max velocity is positive
    return (maxVoltage - kS - kG - kA * acceleration) / kV;
  }

  /**
   * Calculates the minimum achievable velocity given a maximum voltage supply
   * and an acceleration.  Useful for ensuring that velocity and
   * acceleration constraints for a trapezoidal profile are simultaneously
   * achievable - enter the acceleration constraint, and this will give you
   * a simultaneously-achievable velocity constraint.
   *
   * @param maxVoltage The maximum voltage that can be supplied to the elevator.
   * @param acceleration The acceleration of the elevator.
   * @return The minimum possible velocity at the given acceleration.
   */
  constexpr units::unit_t<Velocity> MinAchievableVelocity(
      units::volt_t maxVoltage, units::unit_t<Acceleration> acceleration) {
    // Assume min velocity is negative, ks flips sign
    return (-maxVoltage + kS - kG - kA * acceleration) / kV;
  }

  /**
   * Calculates the maximum achievable acceleration given a maximum voltage
   * supply and a velocity. Useful for ensuring that velocity and
   * acceleration constraints for a trapezoidal profile are simultaneously
   * achievable - enter the velocity constraint, and this will give you
   * a simultaneously-achievable acceleration constraint.
   *
   * @param maxVoltage The maximum voltage that can be supplied to the elevator.
   * @param velocity The velocity of the elevator.
   * @return The maximum possible acceleration at the given velocity.
   */
  constexpr units::unit_t<Acceleration> MaxAchievableAcceleration(
      units::volt_t maxVoltage, units::unit_t<Velocity> velocity) {
    return (maxVoltage - kS * wpi::sgn(velocity) - kG - kV * velocity) / kA;
  }

  /**
   * Calculates the minimum achievable acceleration given a maximum voltage
   * supply and a velocity. Useful for ensuring that velocity and
   * acceleration constraints for a trapezoidal profile are simultaneously
   * achievable - enter the velocity constraint, and this will give you
   * a simultaneously-achievable acceleration constraint.
   *
   * @param maxVoltage The maximum voltage that can be supplied to the elevator.
   * @param velocity The velocity of the elevator.
   * @return The minimum possible acceleration at the given velocity.
   */
  constexpr units::unit_t<Acceleration> MinAchievableAcceleration(
      units::volt_t maxVoltage, units::unit_t<Velocity> velocity) {
    return MaxAchievableAcceleration(-maxVoltage, velocity);
  }

  /// The static gain.
  const units::volt_t kS;

  /// The gravity gain.
  const units::volt_t kG;

  /// The velocity gain.
  const units::unit_t<kv_unit> kV;

  /// The acceleration gain.
  const units::unit_t<ka_unit> kA;

 private:
  /** The period. */
  units::second_t m_dt;
};
}  // namespace frc

#include "frc/controller/proto/ElevatorFeedforwardProto.h"
#include "frc/controller/struct/ElevatorFeedforwardStruct.h"
