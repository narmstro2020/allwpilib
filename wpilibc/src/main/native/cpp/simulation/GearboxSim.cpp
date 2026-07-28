// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "wpi/simulation/GearboxSim.hpp"

#include "wpi/system/RobotController.hpp"
#include "wpi/util/MathExtras.hpp"

using namespace wpi;
using namespace wpi::sim;

GearboxSim::GearboxSim(const wpi::math::LinearSystem<2, 1, 2>& plant,
                       const wpi::math::Gearbox& gearbox,
                       const std::array<double, 2>& measurementStdDevs)
    : LinearSystemSim<2, 1, 2>(plant, measurementStdDevs),
      m_gearbox(gearbox),
      // By theorem 6.10.1 of
      // https://file.tavsys.net/control/controls-engineering-in-frc.pdf, the
      // DC motor state-space model is:
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
          (gearbox.motor.R.value() * m_plant.B(1, 0))) {}

void GearboxSim::SetState(wpi::units::radian_t angularPosition,
                          wpi::units::radians_per_second_t angularVelocity) {
  SetState(wpi::math::Vectord<2>{angularPosition, angularVelocity});
}

void GearboxSim::SetAngle(wpi::units::radian_t angularPosition) {
  SetState(angularPosition, GetAngularVelocity());
}

void GearboxSim::SetAngularVelocity(
    wpi::units::radians_per_second_t angularVelocity) {
  SetState(GetAngularPosition(), angularVelocity);
}

wpi::units::radian_t GearboxSim::GetAngularPosition() const {
  return wpi::units::radian_t{GetOutput(0)};
}

wpi::units::radians_per_second_t GearboxSim::GetAngularVelocity() const {
  return wpi::units::radians_per_second_t{GetOutput(1)};
}

wpi::units::radians_per_second_squared_t GearboxSim::GetAngularAcceleration()
    const {
  return wpi::units::radians_per_second_squared_t{
      (m_plant.A() * m_x + m_plant.B() * m_u)(1, 0)};
}

wpi::units::newton_meter_t GearboxSim::GetTorque() const {
  return wpi::units::newton_meter_t{GetAngularAcceleration().value() *
                                    m_j.value()};
}

wpi::units::ampere_t GearboxSim::GetCurrentDraw() const {
  // Gearbox::Current() takes the velocity of the output and applies the
  // reduction internally to get the motor velocity, and returns the total
  // across all the motors.
  return m_gearbox.Current(wpi::units::radians_per_second_t{m_x(1)},
                           wpi::units::volt_t{m_u(0)}) *
         wpi::util::sgn(m_u(0));
}

wpi::units::volt_t GearboxSim::GetInputVoltage() const {
  return wpi::units::volt_t{GetInput(0)};
}

void GearboxSim::SetInputVoltage(wpi::units::volt_t voltage) {
  SetInput(wpi::math::Vectord<1>{voltage.value()});
  ClampInput(wpi::RobotController::GetBatteryVoltage().value());
}

const wpi::math::Gearbox& GearboxSim::GetGearbox() const {
  return m_gearbox;
}

wpi::units::kilogram_square_meter_t GearboxSim::GetJ() const {
  return m_j;
}
