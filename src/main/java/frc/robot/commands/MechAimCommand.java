// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimeVision;

public class MechAimCommand extends CommandBase {

  private final LimeVision vision;
  private final Drivetrain driveTrain;


  /** Creates a new MechAimCommand. */
  public MechAimCommand(LimeVision vision, Drivetrain driveTrain) {
    this.vision = vision;
    this.driveTrain = driveTrain;
    addRequirements(vision, driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
