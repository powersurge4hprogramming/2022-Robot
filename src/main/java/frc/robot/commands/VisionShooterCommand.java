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

  /** Creates a new VIsionShooterCommand. */
  public VisionShooterCommand(Shooter shooter) {
    this.shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distance = LimeVision.targetDistance();
    // TODO: Find this equation, set it in
    double setSpeed = 1.0;
    SmartDashboard.putNumber("Vision Shooter Speed", setSpeed);
    shooter.set(setSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.set(0.0);
    SmartDashboard.putNumber("Vision Shooter Speed", 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
