// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.MathU;

public class ShooterCommand extends CommandBase {
  private final Shooter shooter;

  private final GenericHID controller;
  /** Creates a new Shootercommand. */
  public ShooterCommand(Shooter shooter, GenericHID controller) {
    this.shooter = shooter;
    this.controller = controller;
    addRequirements(this.shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double shooterSpeed = controller.getRawAxis(Constants.SHOOTER_AXIS);
    shooter.set((float) shooterSpeed);

    double actuatorspeed = controller.getRawAxis(Constants.ACTUATOR_AXIS);
    actuatorspeed = MathU.squareInput(actuatorspeed);
    if(Math.abs(actuatorspeed)<.01){
      actuatorspeed = controller.getRawAxis(Constants.ALT_ACTUATOR_AXIS);    
      actuatorspeed = MathU.squareInput(actuatorspeed);  
      }
    shooter.setactuator((float)actuatorspeed);
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.set(0f);
    shooter.setactuator(0f);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
