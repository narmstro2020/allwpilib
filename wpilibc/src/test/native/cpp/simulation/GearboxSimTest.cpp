// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "wpi/simulation/GearboxSim.hpp"

#include <gtest/gtest.h>

#include "wpi/hardware/motor/PWMVictorSPX.hpp"
#include "wpi/hardware/rotation/Encoder.hpp"
#include "wpi/math/controller/PIDController.hpp"
#include "wpi/math/system/Gearbox.hpp"
#include "wpi/math/system/Models.hpp"
#include "wpi/simulation/BatterySim.hpp"
#include "wpi/simulation/EncoderSim.hpp"
#include "wpi/simulation/RoboRioSim.hpp"
#include "wpi/system/RobotController.hpp"

TEST(GearboxSimTest, VoltageSteadyState) {
  wpi::math::Gearbox gearbox{wpi::math::DCMotor::kNEO, 1, 1.0};
  auto plant = wpi::math::Models::SingleJointedArmFromPhysicalConstants(
      gearbox, wpi::units::kilogram_square_meter_t{0.0005});
  wpi::sim::GearboxSim sim{plant, gearbox};

  wpi::Encoder encoder{0, 1};
  wpi::sim::EncoderSim encoderSim{encoder};
  wpi::PWMVictorSPX motor{0};

  wpi::sim::RoboRioSim::ResetData();
  encoderSim.ResetData();

  // Spin-up
  for (int i = 0; i < 100; i++) {
    // RobotPeriodic runs first
    motor.SetVoltage(12_V);

    // Then, SimulationPeriodic runs
    wpi::sim::RoboRioSim::SetVInVoltage(
        wpi::sim::BatterySim::Calculate({sim.GetCurrentDraw()}));
    sim.SetInputVoltage(motor.GetThrottle() *
                        wpi::RobotController::GetBatteryVoltage());
    sim.Update(20_ms);
    encoderSim.SetRate(sim.GetAngularVelocity().value());
  }

  EXPECT_NEAR((gearbox.motor.Kv * 12_V).value(), encoder.GetRate(), 0.1);

  // Decay
  for (int i = 0; i < 100; i++) {
    // RobotPeriodic runs first
    motor.SetVoltage(0_V);

    // Then, SimulationPeriodic runs
    wpi::sim::RoboRioSim::SetVInVoltage(
        wpi::sim::BatterySim::Calculate({sim.GetCurrentDraw()}));
    sim.SetInputVoltage(motor.GetThrottle() *
                        wpi::RobotController::GetBatteryVoltage());
    sim.Update(20_ms);
    encoderSim.SetRate(sim.GetAngularVelocity().value());
  }

  EXPECT_NEAR(0, encoder.GetRate(), 0.1);
}

TEST(GearboxSimTest, PositionFeedbackControl) {
  wpi::math::Gearbox gearbox{wpi::math::DCMotor::kNEO, 1, 1.0};
  auto plant = wpi::math::Models::SingleJointedArmFromPhysicalConstants(
      gearbox, wpi::units::kilogram_square_meter_t{0.0005});
  wpi::sim::GearboxSim sim{plant, gearbox};

  wpi::math::PIDController controller{0.04, 0.0, 0.001};

  wpi::Encoder encoder{0, 1};
  wpi::sim::EncoderSim encoderSim{encoder};
  wpi::PWMVictorSPX motor{0};

  wpi::sim::RoboRioSim::ResetData();
  encoderSim.ResetData();

  for (int i = 0; i < 140; i++) {
    // RobotPeriodic runs first
    motor.SetThrottle(controller.Calculate(encoder.GetDistance(), 750));

    // Then, SimulationPeriodic runs
    wpi::sim::RoboRioSim::SetVInVoltage(
        wpi::sim::BatterySim::Calculate({sim.GetCurrentDraw()}));
    sim.SetInputVoltage(motor.GetThrottle() *
                        wpi::RobotController::GetBatteryVoltage());
    sim.Update(20_ms);
    encoderSim.SetDistance(sim.GetAngularPosition().value());
    encoderSim.SetRate(sim.GetAngularVelocity().value());
  }

  EXPECT_NEAR(encoder.GetDistance(), 750, 1.0);
  EXPECT_NEAR(encoder.GetRate(), 0, 0.1);
}

TEST(GearboxSimTest, CurrentDrawScalesWithMotorCount) {
  // Current draw is the total across the gearbox's motors, so doubling the
  // motor count while holding the mechanism identical should double the stall
  // current.
  constexpr auto J = wpi::units::kilogram_square_meter_t{0.0005};
  constexpr double kReduction = 3.0;

  wpi::math::Gearbox oneMotor{wpi::math::DCMotor::kNEO, 1, kReduction};
  wpi::math::Gearbox twoMotors{wpi::math::DCMotor::kNEO, 2, kReduction};

  wpi::sim::GearboxSim oneMotorSim{
      wpi::math::Models::SingleJointedArmFromPhysicalConstants(oneMotor, J),
      oneMotor};
  wpi::sim::GearboxSim twoMotorSim{
      wpi::math::Models::SingleJointedArmFromPhysicalConstants(twoMotors, J),
      twoMotors};

  // At rest, current draw is the stall current at the applied voltage.
  oneMotorSim.SetInput(wpi::math::Vectord<1>{12.0});
  twoMotorSim.SetInput(wpi::math::Vectord<1>{12.0});

  EXPECT_NEAR(wpi::math::DCMotor::kNEO.Current(0_rad_per_s, 12_V).value(),
              oneMotorSim.GetCurrentDraw().value(), 1e-9);
  EXPECT_NEAR(2.0 * oneMotorSim.GetCurrentDraw().value(),
              twoMotorSim.GetCurrentDraw().value(), 1e-9);
}
