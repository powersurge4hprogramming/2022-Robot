// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends CommandBase {

  private final Intake intakeSubsystem;

  public IntakeCommand(Intake intake) {
    intakeSubsystem = intake;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    intakeSubsystem.set(Constants.BehaviorConstants.INTAKE_SPEED);
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.set(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
