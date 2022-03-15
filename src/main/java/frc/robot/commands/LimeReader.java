// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.data_structs.LimeVision;

public class LimeReader extends CommandBase {

  public LimeReader() {

  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

    SmartDashboard.putBoolean("Vision On", (LimeVision.getTv() == 1.0));

    if (LimeVision.getTracking()) {
      SmartDashboard.putNumber("Target Distance", LimeVision.targetDistance());
    } else {
      SmartDashboard.putNumber("Target Distance", -1.0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("Vision On", false);
    SmartDashboard.putNumber("Target Distance", -1.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
