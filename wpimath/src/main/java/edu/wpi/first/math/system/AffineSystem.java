// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math.system;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

/**
 * A plant defined using state-space notation.
 *
 * <p>
 * A plant is a mathematical model of a system's dynamics.
 *
 * <p>
 * For more on the underlying math, read
 * https://file.tavsys.net/control/controls-engineering-in-frc.pdf.
 *
 * @param <States>  Number of states.
 * @param <Inputs>  Number of inputs.
 * @param <Outputs> Number of outputs.
 */
public class AffineSystem<States extends Num, Inputs extends Num, Outputs extends Num>
    extends LinearSystem<States, Inputs, Outputs> {

  /** Affine constant matrix. */
  private final Matrix<States, N1> m_constant;

  /**
   * Construct a new LinearSystem from the four system matrices.
   *
   * @param A        The system matrix A.
   * @param B        The input matrix B.
   * @param C        The output matrix C.
   * @param D        The feedthrough matrix D.
   * @param constant The constant matrix of the affine system.
   * @throws IllegalArgumentException if any matrix element isn't finite.
   */
  public AffineSystem(
      Matrix<States, States> A,
      Matrix<States, Inputs> B,
      Matrix<Outputs, States> C,
      Matrix<Outputs, Inputs> D,
      Matrix<States, N1> constant) {
    super(A, B, C, D);

    for (int row = 0; row < constant.getNumRows(); ++row) {
      for (int col = 0; col < constant.getNumCols(); ++col) {
        if (!Double.isFinite(constant.get(row, col))) {
          throw new IllegalArgumentException(
              "Elements of constant aren't finite. This is usually due to model implementation errors.");
        }
      }
    }

    this.m_constant = constant;
  }

  /**
   * Returns the constants matrix.
   *
   * @return the constants matrix.
   */
  public Matrix<States, N1> getConstant() {
    return m_constant;
  }

  /**
   * Returns an element of the constants matrix.
   *
   * @param row Row of constant.
   * @return The constants matrix at (i, j).
   */
  public double getConstant(int row) {
    return m_constant.get(row, 0);
  }

  /**
   * Computes the new x given the old x and the control input.
   *
   * <p>
   * This is used by state observers directly to run updates based on state
   * estimate.
   *
   * @param x         The current state.
   * @param clampedU  The control input.
   * @param dtSeconds Timestep for model update.
   * @return the updated x.
   */
  @Override
  public Matrix<States, N1> calculateX(
      Matrix<States, N1> x, Matrix<Inputs, N1> clampedU, double dtSeconds) {
    var discABpair = Discretization.discretizeAB(getA(), getB(), dtSeconds);
    var discA = discABpair.getFirst();
    var discB = discABpair.getSecond();
    var discConstants = discB.times(getB().solve(m_constant));

    return discA.times(x).plus(discB.times(clampedU)).plus(discConstants);
  }

  @Override
  public String toString() {
    return String.format(
        "Affine System: A\n%s\n\nB:\n%s\n\nC:\n%s\n\nD:\n%s\nConstants:\n%s\n",
        getA().toString(), getB().toString(), getC().toString(), getD().toString(), m_constant.toString());
  }
}
