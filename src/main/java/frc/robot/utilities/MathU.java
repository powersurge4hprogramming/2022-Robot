// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

public class MathU {

  /**
   * Squares the input, preserving the sign
   */
  public static double squareInput(double input) {
    return Math.pow(input, 2) * Math.signum(input);
  }

  /**
   * Maps from 0 to 1 onto a to b
   * tau is value from zero to one
   * if a = 1, b = 6, tau = 0.5, then result = 3.5
   */
  public static final double lerp(double a, double b, double tau) {
    tau = Math.max(tau, 0.0);
    tau = Math.min(tau, 1.0);
    return (b * tau) + (a * (1 - tau));
  }

  /**
   * Maps from a to b onto 0 to 1
   * val is value from a to b
   * if a = 1, b = 6, val = 3.5, then result = 0.5
   */
  public static final double invLerp(double a, double b, double val) {
    /*
     * val = Math.max(val, 0.2);
     * val = Math.min(val, 0.8);
     */
    return (val - a) / (b - a);
  }

  /**
   * Maps oldA and OldB to newA and newB
   * val is value from oldA to oldB
   * if a = 1, b = 6, val = 3.5, then result = 0.5
   * result is from newA to newB
   */
  public static final double remap(double oldA, double oldB, double newA, double newB, double value) {
    double result = invLerp(oldA, oldB, value);
    return lerp(newA, newB, result);
  }
}
