// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/simulation/ElevatorSim.h"

#include "frc/system/NumericalIntegration.h"
#include "frc/system/plant/LinearSystemId.h"

using namespace frc;
using namespace frc::sim;

ElevatorSim::ElevatorSim(const DCMotor& gearbox, units::kilogram_t carriageMass,
                         units::meter_t drumRadius, double gearing,
                         units::meter_t minHeight, units::meter_t maxHeight,
                         bool simulateGravity, units::meter_t startingHeight,
                         const std::array<double, 2>& measurementStdDevs)
    : LinearSystemSim(LinearSystemId::ElevatorSystem(gearbox, carriageMass,
                                                     drumRadius, gearing),
                      measurementStdDevs),
      m_gearbox(gearbox),
      m_gearing(gearing),
      m_drumRadius(drumRadius),
      m_carriageMass(carriageMass),

      m_minHeight(minHeight),
      m_maxHeight(maxHeight),
      m_simulateGravity(simulateGravity) {
  SetState(startingHeight, units::meters_per_second_t(0));
}

ElevatorSim::ElevatorSim(const DCMotor& gearbox, decltype(1_V / 1_mps) kV,
                         decltype(1_V / 1_mps_sq) kA, double gearing,
                         units::meter_t minHeight, units::meter_t maxHeight,
                         bool simulateGravity, units::meter_t startingHeight,
                         const std::array<double, 2>& measurementStdDevs)
    : LinearSystemSim(
          LinearSystemId::IdentifyPositionSystem<units::meter, units::volt>(kV,
                                                                            kA),
          measurementStdDevs),
      m_gearbox(gearbox),
      m_gearing(gearing),
      m_drumRadius(-m_plant.B(1, 0) * gearing /
                   (gearbox.Kv.value() * m_plant.A(1, 1))),
      m_carriageMass(
          gearing * gearbox.Kt.value() /
          (gearbox.R.value() * m_drumRadius.value() * m_plant.B(1, 0))),
      m_minHeight(minHeight),
      m_maxHeight(maxHeight),
      m_simulateGravity(simulateGravity) {
  // See wpimath/algorithms.md#Elevator_sim for derivation
  SetState(startingHeight, units::meters_per_second_t(0));
}

ElevatorSim::ElevatorSim(const DCMotor& gearbox,
                         const LinearSystem<2, 1, 2>& plant, double gearing,
                         units::meter_t minHeight, units::meter_t maxHeight,
                         bool simulateGravity, units::meter_t startingHeight,
                         const std::array<double, 2>& measurementStdDevs)
    : LinearSystemSim(plant, measurementStdDevs),
      m_gearbox(gearbox),
      m_gearing(gearing),
      m_drumRadius(-m_plant.B(1, 0) * gearing /
                   (gearbox.Kv.value() * m_plant.A(1, 1))),
      m_carriageMass(
          gearing * gearbox.Kt.value() /
          (gearbox.R.value() * m_drumRadius.value() * m_plant.B(1, 0))),
      m_minHeight(minHeight),
      m_maxHeight(maxHeight),
      m_simulateGravity(simulateGravity) {
  // See wpimath/algorithms.md#Elevator_sim for derivation
  SetState(startingHeight, units::meters_per_second_t(0));
}

Vectord<2> ElevatorSim::UpdateX(const Vectord<2>& currentXhat,
                                const Vectord<1>& u, units::second_t dt) {
  auto updatedXhat = RKDP(
      [&](const Vectord<2>& x, const Vectord<1>& u_) -> Vectord<2> {
        Vectord<2> xdot = m_plant.A() * x + m_plant.B() * u;

        if (m_simulateGravity) {
          xdot += Vectord<2>{0.0, -9.8};
        }
        return xdot;
      },
      currentXhat, u, dt);
  // Check for collision after updating x-hat.
  if (WouldHitLowerLimit(units::meter_t{updatedXhat(0)})) {
    return Vectord<2>{m_minHeight.value(), 0.0};
  }
  if (WouldHitUpperLimit(units::meter_t{updatedXhat(0)})) {
    return Vectord<2>{m_maxHeight.value(), 0.0};
  }
  return updatedXhat;
}
