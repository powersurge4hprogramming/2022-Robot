// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class DriveCommand extends CommandBase {

  private final Drivetrain drivetrain;
  private final GenericHID driveJoystick;

  /** Creates a new DriveCommand. */
  public DriveCommand(Drivetrain drivetrain, GenericHID driveJoystick) {
      this.drivetrain= drivetrain;
      this.driveJoystick = driveJoystick;
      addRequirements(drivetrain);
        // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      Float Xaxis= (float) driveJoystick.getRawAxis(Constants.DRIVER_JOYSTICK_X_AXIS);
      Float Yaxis= (float) -driveJoystick.getRawAxis(Constants.DRIVER_JOYSTICK_Y_AXIS);
      Float Zaxis= (float) driveJoystick.getRawAxis(Constants.DRIVER_JOYSTICK_Z_AXIS);
//                       y      x      z
      drivetrain.drive(Yaxis, Xaxis, Zaxis);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
