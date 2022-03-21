// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Gatekeeper;

public class GatekeeperCommand extends CommandBase {

  private final Gatekeeper gatekeeper;

  public GatekeeperCommand(Gatekeeper gatekeeper) {
    this.gatekeeper = gatekeeper;
    addRequirements(this.gatekeeper);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    gatekeeper.set(Constants.BehaviorConstants.GATEKEEPER_SPEED);
  }

  @Override
  public void end(boolean interrupted) {
    gatekeeper.set(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
