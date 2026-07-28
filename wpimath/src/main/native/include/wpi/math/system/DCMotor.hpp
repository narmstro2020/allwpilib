// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "wpi/units/angular_velocity.hpp"
#include "wpi/units/base.hpp"
#include "wpi/units/current.hpp"
#include "wpi/units/impedance.hpp"
#include "wpi/units/torque.hpp"
#include "wpi/units/voltage.hpp"
#include "wpi/util/SymbolExports.hpp"

namespace wpi::math {

/**
 * Holds the constants for a single DC motor.
 *
 * This describes the motor itself and nothing about how it is installed. To
 * model a number of these motors driving a mechanism through a gear reduction,
 * wrap this in a Gearbox.
 */
class WPILIB_DLLEXPORT DCMotor {
 public:
  using radians_per_second_per_volt_t = wpi::units::unit_t<
      wpi::units::compound_unit<wpi::units::radians_per_second,
                                wpi::units::inverse<wpi::units::volt>>>;
  using newton_meters_per_ampere_t =
      wpi::units::unit_t<wpi::units::compound_unit<
          wpi::units::newton_meters, wpi::units::inverse<wpi::units::ampere>>>;

  /// Voltage at which the motor constants were measured.
  wpi::units::volt_t nominalVoltage;

  /// Torque when stalled.
  wpi::units::newton_meter_t stallTorque;

  /// Current draw when stalled.
  wpi::units::ampere_t stallCurrent;

  /// Current draw under no load.
  wpi::units::ampere_t freeCurrent;

  /// Angular velocity under no load.
  wpi::units::radians_per_second_t freeSpeed;

  /// Motor internal resistance.
  wpi::units::ohm_t R;

  /// Motor velocity constant.
  radians_per_second_per_volt_t Kv;

  /// Motor torque constant.
  newton_meters_per_ampere_t Kt;

  /**
   * Constructs a DC motor.
   *
   * @param nominalVoltage Voltage at which the motor constants were measured.
   * @param stallTorque Torque when stalled.
   * @param stallCurrent Current draw when stalled.
   * @param freeCurrent Current draw under no load.
   * @param freeSpeed Angular velocity under no load.
   */
  constexpr DCMotor(wpi::units::volt_t nominalVoltage,
                    wpi::units::newton_meter_t stallTorque,
                    wpi::units::ampere_t stallCurrent,
                    wpi::units::ampere_t freeCurrent,
                    wpi::units::radians_per_second_t freeSpeed)
      : nominalVoltage(nominalVoltage),
        stallTorque(stallTorque),
        stallCurrent(stallCurrent),
        freeCurrent(freeCurrent),
        freeSpeed(freeSpeed),
        R(nominalVoltage / stallCurrent),
        Kv(freeSpeed / (nominalVoltage - R * freeCurrent)),
        Kt(stallTorque / stallCurrent) {}

  /**
   * Returns current drawn by the motor with given velocity and input voltage.
   *
   * @param velocity The current angular velocity of the motor.
   * @param inputVoltage The voltage being applied to the motor.
   */
  constexpr wpi::units::ampere_t Current(
      wpi::units::radians_per_second_t velocity,
      wpi::units::volt_t inputVoltage) const {
    return -1.0 / Kv / R * velocity + 1.0 / R * inputVoltage;
  }

  /**
   * Returns current drawn by the motor for a given torque.
   *
   * @param torque The torque produced by the motor.
   */
  constexpr wpi::units::ampere_t Current(
      wpi::units::newton_meter_t torque) const {
    return torque / Kt;
  }

  /**
   * Returns torque produced by the motor with a given current.
   *
   * @param current The current drawn by the motor.
   */
  constexpr wpi::units::newton_meter_t Torque(
      wpi::units::ampere_t current) const {
    return current * Kt;
  }

  /**
   * Returns the voltage provided to the motor for a given torque and
   * angular velocity.
   *
   * @param torque The torque produced by the motor.
   * @param velocity The current angular velocity of the motor.
   */
  constexpr wpi::units::volt_t Voltage(
      wpi::units::newton_meter_t torque,
      wpi::units::radians_per_second_t velocity) const {
    return 1.0 / Kv * velocity + 1.0 / Kt * R * torque;
  }

  /**
   * Returns the angular velocity produced by the motor at a given torque and
   * input voltage.
   *
   * @param torque The torque produced by the motor.
   * @param inputVoltage The input voltage provided to the motor.
   */
  constexpr wpi::units::radians_per_second_t Velocity(
      wpi::units::newton_meter_t torque,
      wpi::units::volt_t inputVoltage) const {
    return inputVoltage * Kv - 1.0 / Kt * torque * R * Kv;
  }

  /// A CIM motor.
  static const DCMotor kCIM;

  /// A MiniCIM motor.
  static const DCMotor kMiniCIM;

  /// A Bag motor.
  static const DCMotor kBag;

  /// A Vex 775 Pro motor.
  static const DCMotor kVex775Pro;

  /// An Andymark RS 775-125 motor.
  static const DCMotor kRS775_125;

  /// A Banebots RS 775 motor.
  static const DCMotor kBanebotsRS775;

  /// An Andymark 9015 motor.
  static const DCMotor kAndymark9015;

  /// A Banebots RS 550 motor.
  static const DCMotor kBanebotsRS550;

  /// A NEO brushless motor.
  static const DCMotor kNEO;

  /// A NEO 550 brushless motor.
  static const DCMotor kNEO550;

  /// A Falcon 500 brushless motor.
  static const DCMotor kFalcon500;

  /**
   * A Falcon 500 motor with FOC (Field-Oriented Control) enabled.
   *
   * @see https://store.ctr-electronics.com/falcon-500-powered-by-talon-fx/
   */
  static const DCMotor kFalcon500FOC;

  /**
   * A Romi/TI_RSLK MAX motor.
   *
   * @see https://www.pololu.com/product/1520/specs
   */
  static const DCMotor kRomiBuiltIn;

  /**
   * A Kraken X60 brushless motor.
   *
   * @see https://store.ctr-electronics.com/announcing-kraken-x60/
   */
  static const DCMotor kKrakenX60;

  /**
   * A Kraken X60 brushless motor with FOC (Field-Oriented Control)
   * enabled.
   *
   * @see https://store.ctr-electronics.com/announcing-kraken-x60/
   */
  static const DCMotor kKrakenX60FOC;

  /**
   * A Kraken X44 brushless motor.
   *
   * @see https://motors.ctr-electronics.com/dyno/dynometer-testing/
   */
  static const DCMotor kKrakenX44;

  /**
   * A Kraken X44 brushless motor with FOC (Field-Oriented Control)
   * enabled.
   *
   * @see https://motors.ctr-electronics.com/dyno/dynometer-testing/
   */
  static const DCMotor kKrakenX44FOC;

  /**
   * A Minion brushless motor.
   *
   * @see https://motors.ctr-electronics.com/dyno/dynometer-testing/
   */
  static const DCMotor kMinion;

  /**
   * A Neo Vortex brushless motor.
   *
   * @see https://www.revrobotics.com/next-generation-spark-neo/
   */
  static const DCMotor kNeoVortex;
};

inline constexpr DCMotor DCMotor::kCIM{12_V, 2.42_Nm, 133_A, 2.7_A, 5310_rpm};

inline constexpr DCMotor DCMotor::kMiniCIM{12_V, 1.41_Nm, 89_A, 3_A, 5840_rpm};

inline constexpr DCMotor DCMotor::kBag{12_V, 0.43_Nm, 53_A, 1.8_A, 13180_rpm};

inline constexpr DCMotor DCMotor::kVex775Pro{12_V, 0.71_Nm, 134_A, 0.7_A,
                                             18730_rpm};

inline constexpr DCMotor DCMotor::kRS775_125{12_V, 0.28_Nm, 18_A, 1.6_A,
                                             5800_rpm};

inline constexpr DCMotor DCMotor::kBanebotsRS775{12_V, 0.72_Nm, 97_A, 2.7_A,
                                                 13050_rpm};

inline constexpr DCMotor DCMotor::kAndymark9015{12_V, 0.36_Nm, 71_A, 3.7_A,
                                                14270_rpm};

inline constexpr DCMotor DCMotor::kBanebotsRS550{12_V, 0.38_Nm, 84_A, 0.4_A,
                                                 19000_rpm};

inline constexpr DCMotor DCMotor::kNEO{12_V, 2.6_Nm, 105_A, 1.8_A, 5676_rpm};

inline constexpr DCMotor DCMotor::kNEO550{12_V, 0.97_Nm, 100_A, 1.4_A,
                                          11000_rpm};

inline constexpr DCMotor DCMotor::kFalcon500{12_V, 4.69_Nm, 257_A, 1.5_A,
                                             6380_rpm};

inline constexpr DCMotor DCMotor::kFalcon500FOC{12_V, 5.84_Nm, 304_A, 1.5_A,
                                                6080_rpm};

inline constexpr DCMotor DCMotor::kRomiBuiltIn{4.5_V, 0.1765_Nm, 1.25_A, 0.13_A,
                                               150_rpm};

inline constexpr DCMotor DCMotor::kKrakenX60{12_V, 7.09_Nm, 366_A, 2_A,
                                             6000_rpm};

inline constexpr DCMotor DCMotor::kKrakenX60FOC{12_V, 9.37_Nm, 483_A, 2_A,
                                                5800_rpm};

inline constexpr DCMotor DCMotor::kKrakenX44{12_V, 4.11_Nm, 279_A, 2_A,
                                             7758_rpm};

inline constexpr DCMotor DCMotor::kKrakenX44FOC{12_V, 5.01_Nm, 329_A, 2_A,
                                                7368_rpm};

inline constexpr DCMotor DCMotor::kMinion{12_V, 3.17_Nm, 211_A, 2_A, 7704_rpm};

inline constexpr DCMotor DCMotor::kNeoVortex{12_V, 3.60_Nm, 211_A, 3.615_A,
                                             6784_rpm};

}  // namespace wpi::math

#include "wpi/math/system/proto/DCMotorProto.hpp"
#include "wpi/math/system/struct/DCMotorStruct.hpp"
