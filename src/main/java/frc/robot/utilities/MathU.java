// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

/** Add your docs here. */
public class MathU {
    public static double squareInput(double input){
        return Math.pow(input, 2) * Math.signum(input);
      }
}
