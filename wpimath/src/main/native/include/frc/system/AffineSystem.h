// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <algorithm>
#include <concepts>
#include <functional>
#include <stdexcept>

#include <wpi/Algorithm.h>
#include <wpi/SmallVector.h>

#include "frc/EigenCore.h"
#include "frc/StateSpaceUtil.h"
#include "frc/system/Discretization.h"
#include "units/time.h"

namespace frc {

/**
 * A plant defined using state-space notation with an affine component.
 *
 * A plant is a mathematical model of a system's dynamics.
 *
 * For more on the underlying math, read
 * https://file.tavsys.net/control/controls-engineering-in-frc.pdf.
 * 
 * The system modeled of the form Ax + Bu + c where c is allowed to be
 * dynamic.
 *
 * @tparam States Number of states.
 * @tparam Inputs Number of inputs.
 * @tparam Outputs Number of outputs.
 */
template <int States, int Inputs, int Outputs>
class AffineSystem {
 public:
  using StateVector = Vectord<States>;
  using InputVector = Vectord<Inputs>;
  using OutputVector = Vectord<Outputs>;
  using ConstantVector = Vectord<Outputs>;

  /**
   * Constructs a new AffineSystem from a LinearSystem component.
   *
   * @param linearSystemComponent The linear system component.
   */
  LinearSystem(const frc::LinearSystem<States, Inputs, Outputs>& linearSystemComponent) {
    m_linearSystemComponent = linearSystemComponent
  }

  LinearSystem(const LinearSystem&) = default;
  LinearSystem& operator=(const LinearSystem&) = default;
  LinearSystem(LinearSystem&&) = default;
  LinearSystem& operator=(LinearSystem&&) = default;

  /**
   * Returns the system matrix A.
   */
  const LinearSystem<States, Inputs, Outputs>& GetLinearSystemComponent() const { return m_linearSystemComponent; }

  /**
   * Computes the new x given the old x and the control input.
   *
   * This is used by state observers directly to run updates based on state
   * estimate.
   *
   * @param x        The current state.
   * @param clampedU The control input.
   * @param c The constants matrix
   * @param dt       Timestep for model update.
   */
  StateVector CalculateX(const StateVector& x, const InputVector& clampedU,
                        const ConstantVector c, units::second_t dt) const {
    Matrixd<States, States> discA;
    Matrixd<States, Inputs> discB;
    DiscretizeAB<States, Inputs>(m_A, m_B, dt, &discA, &discB);

    return discA * x + discB * (clampedU + m_linearSystemComponent.B().Solve(c));
  }

  /**
   * Computes the new y given the control input.
   *
   * This is used by state observers directly to run updates based on state
   * estimate.
   *
   * @param x The current state.
   * @param clampedU The control input.
   */
  OutputVector CalculateY(const StateVector& x,
                          const InputVector& clampedU) const {
    return m_linearSystemComponent.C() * x + m_linearSystemComponent.D() * clampedU;
  }

 private:
  /**
   * The linear system component.
   */
  frc::LinearSystem<States, Inputs, Outputs> m_linearSystemComponent;
};

}  // namespace frc
