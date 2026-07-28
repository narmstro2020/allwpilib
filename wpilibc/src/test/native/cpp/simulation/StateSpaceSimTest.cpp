// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>

#include "wpi/hardware/motor/PWMVictorSPX.hpp"
#include "wpi/hardware/rotation/Encoder.hpp"
#include "wpi/math/controller/PIDController.hpp"
#include "wpi/math/controller/SimpleMotorFeedforward.hpp"
#include "wpi/math/system/Models.hpp"
#include "wpi/simulation/BatterySim.hpp"
#include "wpi/simulation/EncoderSim.hpp"
#include "wpi/simulation/FlywheelSim.hpp"
#include "wpi/simulation/RoboRioSim.hpp"
#include "wpi/system/RobotController.hpp"
#include "wpi/units/angular_acceleration.hpp"
#include "wpi/units/angular_velocity.hpp"

TEST(StateSpaceSimTest, FlywheelSim) {
  const wpi::math::LinearSystem<1, 1, 1> plant =
      wpi::math::Models::FlywheelFromSysId(0.02_V / 1_rad_per_s,
                                           0.01_V / 1_rad_per_s_sq);
  wpi::sim::FlywheelSim sim{plant,
                            wpi::math::Gearbox{wpi::math::DCMotor::kNEO, 2}};
  wpi::math::PIDController controller{0.2, 0.0, 0.0};
  wpi::math::SimpleMotorFeedforward<wpi::units::radian> feedforward{
      0_V, 0.02_V / 1_rad_per_s, 0.01_V / 1_rad_per_s_sq};
  wpi::Encoder encoder{0, 1};
  wpi::sim::EncoderSim encoderSim{encoder};
  wpi::PWMVictorSPX motor{0};

  wpi::sim::RoboRioSim::ResetData();
  encoderSim.ResetData();

  for (int i = 0; i < 100; i++) {
    // RobotPeriodic runs first
    auto voltageOut = controller.Calculate(encoder.GetRate(), 200.0);
    motor.SetVoltage(wpi::units::volt_t{voltageOut} +
                     feedforward.Calculate(200_rad_per_s));

    // Then, SimulationPeriodic runs
    wpi::sim::RoboRioSim::SetVInVoltage(
        wpi::sim::BatterySim::Calculate({sim.GetCurrentDraw()}));
    sim.SetInput(wpi::math::Vectord<1>{
        motor.GetThrottle() * wpi::RobotController::GetInputVoltage()});
    sim.Update(20_ms);
    encoderSim.SetRate(sim.GetAngularVelocity().value());
  }

  ASSERT_TRUE(std::abs(200 - encoder.GetRate()) < 0.1);
}

TEST(StateSpaceSimTest, FlywheelSimCurrentDrawScalesWithMotorCount) {
  // Current draw is the total across the gearbox's motors, so doubling the
  // motor count while holding the mechanism identical should double the stall
  // current.
  constexpr auto J = 0.00032_kg_sq_m;
  constexpr double kReduction = 2.0;

  wpi::math::Gearbox oneMotor{wpi::math::DCMotor::kNEO, 1, kReduction};
  wpi::math::Gearbox twoMotors{wpi::math::DCMotor::kNEO, 2, kReduction};

  wpi::sim::FlywheelSim oneMotorSim{
      wpi::math::Models::FlywheelFromPhysicalConstants(oneMotor, J), oneMotor};
  wpi::sim::FlywheelSim twoMotorSim{
      wpi::math::Models::FlywheelFromPhysicalConstants(twoMotors, J),
      twoMotors};

  // At rest, current draw is the stall current at the applied voltage.
  oneMotorSim.SetInput(wpi::math::Vectord<1>{12.0});
  twoMotorSim.SetInput(wpi::math::Vectord<1>{12.0});

  EXPECT_NEAR(wpi::math::DCMotor::kNEO.Current(0_rad_per_s, 12_V).value(),
              oneMotorSim.GetCurrentDraw().value(), 1e-9);
  EXPECT_NEAR(2.0 * oneMotorSim.GetCurrentDraw().value(),
              twoMotorSim.GetCurrentDraw().value(), 1e-9);

  // J is solved back out of the plant, so it should match what it was built
  // with.
  EXPECT_NEAR(J.value(), twoMotorSim.J().value(), 1e-12);
}
