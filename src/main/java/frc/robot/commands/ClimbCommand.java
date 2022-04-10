// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class ClimbCommand extends CommandBase {

  private final Climber climber;

  public ClimbCommand(Climber climber) {
    this.climber = climber;
    addRequirements(this.climber);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    climber.runMotor(Constants.BehaviorConstants.CLIMBER_SPEED);
  }

  @Override
  public void end(boolean interrupted) {
    climber.runMotor(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
