// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "wpi/simulation/FlywheelSim.hpp"

#include "wpi/system/RobotController.hpp"
#include "wpi/util/MathExtras.hpp"

using namespace wpi;
using namespace wpi::sim;

FlywheelSim::FlywheelSim(const wpi::math::LinearSystem<1, 1, 1>& plant,
                         const wpi::math::Gearbox& gearbox,
                         const std::array<double, 1>& measurementStdDevs)
    : LinearSystemSim<1, 1, 1>(plant, measurementStdDevs),
      m_gearbox(gearbox),
      // By theorem 6.10.1 of
      // https://file.tavsys.net/control/controls-engineering-in-frc.pdf, the
      // flywheel state-space model is:
      //
      //   dx/dt = -G²Kₜ/(KᵥRJ)x + (GKₜ)/(RJ)u
      //   A = -G²Kₜ/(KᵥRJ)
      //   B = GKₜ/(RJ)
      //
      // Solve for J.
      //
      //   B = GKₜ/(RJ)
      //   J = GKₜ/(RB)
      //
      // Kₜ here is that of the gearbox as a whole, so it scales with the number
      // of motors. G is taken from the gearbox, so the plant must have been
      // built with the same reduction.
      m_j(gearbox.reduction * gearbox.numMotors * gearbox.motor.Kt.value() /
          (gearbox.motor.R.value() * m_plant.B(0, 0))) {}

void FlywheelSim::SetVelocity(wpi::units::radians_per_second_t velocity) {
  LinearSystemSim::SetState(wpi::math::Vectord<1>{velocity.value()});
}

wpi::units::radians_per_second_t FlywheelSim::GetAngularVelocity() const {
  return wpi::units::radians_per_second_t{GetOutput(0)};
}

wpi::units::radians_per_second_squared_t FlywheelSim::GetAngularAcceleration()
    const {
  return wpi::units::radians_per_second_squared_t{
      (m_plant.A() * m_x + m_plant.B() * m_u)(0, 0)};
}

wpi::units::newton_meter_t FlywheelSim::GetTorque() const {
  return wpi::units::newton_meter_t{GetAngularAcceleration().value() *
                                    m_j.value()};
}

wpi::units::ampere_t FlywheelSim::GetCurrentDraw() const {
  // Gearbox::Current() takes the velocity of the output and applies the
  // reduction internally to get the motor velocity, and returns the total
  // across all the motors.
  return m_gearbox.Current(wpi::units::radians_per_second_t{m_x(0)},
                           wpi::units::volt_t{m_u(0)}) *
         wpi::util::sgn(m_u(0));
}

wpi::units::volt_t FlywheelSim::GetInputVoltage() const {
  return wpi::units::volt_t{GetInput(0)};
}

void FlywheelSim::SetInputVoltage(wpi::units::volt_t voltage) {
  SetInput(wpi::math::Vectord<1>{voltage.value()});
  ClampInput(wpi::RobotController::GetBatteryVoltage().value());
}
