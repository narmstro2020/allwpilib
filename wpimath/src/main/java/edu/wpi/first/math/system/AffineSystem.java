// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math.system;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

/**
 * A plant defined using state-space notation with an affine component.
 *
 * <p>A plant is a mathematical model of a system's dynamics.
 *
 * <p>For more on the underlying math, read
 * https://file.tavsys.net/control/controls-engineering-in-frc.pdf.
 *
 * <p>The system modeled is of the form Ax + Bu + c where c is allowed to be dynamic.
 *
 * @param <States> Number of states.
 * @param <Inputs> Number of inputs.
 * @param <Outputs> Number of outputs.
 */
public class AffineSystem<States extends Num, Inputs extends Num, Outputs extends Num> {
  /** Linear system component. */
  private final LinearSystem<States, Inputs, Outputs> m_linearSystemComponent;

  /**
   * Construct a new AffineSystem from a LinearSystem component.
   *
   * @param linearSystemComponent The linear system component.
   */
  public AffineSystem(LinearSystem<States, Inputs, Outputs> linearSystemComponent) {
    m_linearSystemComponent = linearSystemComponent;
  }

  /**
   * Returns the linear system component.
   *
   * @return the linear system component.
   */
  public LinearSystem<States, Inputs, Outputs> getLinearSystemComponent() {
    return m_linearSystemComponent;
  }

  /**
   * Computes the new x given the old x and the control input.
   *
   * <p>This is used by state observers directly to run updates based on state estimate.
   *
   * @param x The current state.
   * @param clampedU The control input.
   * @param c The constant matrix.
   * @param dtSeconds Timestep for model update.
   * @return the updated x.
   */
  public Matrix<States, N1> calculateX(
      Matrix<States, N1> x, Matrix<Inputs, N1> clampedU, Matrix<States, N1> c, double dtSeconds) {
    var discABpair =
        Discretization.discretizeAB(
            m_linearSystemComponent.getA(), m_linearSystemComponent.getB(), dtSeconds);
    var discA = discABpair.getFirst();
    var discB = discABpair.getSecond();
    return discA.times(x).plus(discB.times(clampedU.plus(m_linearSystemComponent.getB().solve(c))));
  }

  /**
   * Computes the new y given the control input.
   *
   * <p>This is used by state observers directly to run updates based on state estimate.
   *
   * @param x The current state.
   * @param clampedU The control input.
   * @return the updated output matrix Y.
   */
  public Matrix<Outputs, N1> calculateY(Matrix<States, N1> x, Matrix<Inputs, N1> clampedU) {
    return m_linearSystemComponent
        .getC()
        .times(x)
        .plus(m_linearSystemComponent.getD().times(clampedU));
  }

  @Override
  public String toString() {
    return String.format(
        "Affine System: A\n%s\n\nB:\n%s\n\nC:\n%s\n\nD:\n%s\n",
        m_linearSystemComponent.getA().toString(),
        m_linearSystemComponent.getB().toString(),
        m_linearSystemComponent.getC().toString(),
        m_linearSystemComponent.getD().toString());
  }
}
