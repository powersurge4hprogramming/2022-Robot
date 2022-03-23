// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.data_structs.LimeVision;
import frc.robot.subsystems.Shooter;

public class VisionShooterCommand extends CommandBase {

  private final Shooter shooter;

  public VisionShooterCommand(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double distance = LimeVision.targetDistance();
    double setSpeed = 1911 * distance + 8948;
    SmartDashboard.putNumber("Vision Shooter Speed", setSpeed);
    shooter.setVelocity(setSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setPercentOutput(0.0);
    SmartDashboard.putNumber("Vision Shooter Speed", 0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
