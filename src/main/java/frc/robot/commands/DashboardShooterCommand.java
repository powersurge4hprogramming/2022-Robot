// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class DashboardShooterCommand extends CommandBase {

  private final Shooter shooter;

  public DashboardShooterCommand(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(this.shooter);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double shooterSpeed = SmartDashboard.getNumber("Shooter Speed", 0.0);
    shooter.setPercentOutput(shooterSpeed);
    SmartDashboard.putNumber("Shooter Speed", shooterSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setPercentOutput(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
