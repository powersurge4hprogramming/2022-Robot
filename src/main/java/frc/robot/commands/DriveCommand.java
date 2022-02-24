// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.MathU;

public class DriveCommand extends CommandBase {

  private final Drivetrain drivetrain;
  private final GenericHID driveJoystick;
  private final SlewRateLimiter yLimiter;
  private final SlewRateLimiter xLimiter;
  private final SlewRateLimiter zLimiter;

  /** Creates a new DriveCommand. */
  public DriveCommand(Drivetrain drivetrain, GenericHID driveJoystick) {
    this.drivetrain = drivetrain;
    this.driveJoystick = driveJoystick;
    yLimiter = new SlewRateLimiter(Constants.InputConstants.DRIVER_LATERAL_SLEW);
    xLimiter = new SlewRateLimiter(Constants.InputConstants.DRIVER_LATERAL_SLEW);
    zLimiter = new SlewRateLimiter(Constants.InputConstants.DRIVER_TWIST_SLEW);
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xAxis = driveJoystick.getRawAxis(Constants.InputConstants.DRIVER_JOYSTICK_X_AXIS);
    double yAxis = -driveJoystick.getRawAxis(Constants.InputConstants.DRIVER_JOYSTICK_Y_AXIS);
    double zAxis = driveJoystick.getRawAxis(Constants.InputConstants.DRIVER_JOYSTICK_Z_AXIS);

    double scale = driveJoystick.getRawAxis(Constants.InputConstants.DRIVER_JOYSTICK_SCALE_AXIS);
    scale = (1 - scale) / 2;

    xAxis = xLimiter.calculate(MathU.squareInput(xAxis) * scale);
    yAxis = yLimiter.calculate(MathU.squareInput(yAxis) * scale);
    zAxis = zLimiter.calculate(MathU.squareInput(zAxis) * scale);

    // y x z
    drivetrain.drive((float) yAxis, (float) xAxis, (float) zAxis, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
