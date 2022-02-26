// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.data_structs.LimeVision;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.MathU;

public class MechAimCommand extends CommandBase {

  private final Drivetrain driveTrain;
  private final PIDController visionController;

  /** Creates a new MechAimCommand. */
  public MechAimCommand(Drivetrain driveTrain) {
    this.driveTrain = driveTrain;

    visionController = new PIDController(Constants.VISION_PID_X, Constants.VISION_PID_I, Constants.VISION_PID_D);
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    visionController.setTolerance(Constants.VISION_PID_TOLERANCE);
    visionController.setSetpoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xOffset = LimeVision.getXOffsetAngle();

    // -27 to 27 is the range of the xOffset passed by the limelight
    xOffset = MathU.remap(-27, 27, -1, 1, xOffset);
    double z = visionController.calculate(xOffset);

    // TODO: Ensure you dont need to negate it, PID setpoint should take care of
    // that
    driveTrain.drive(0f, 0f, (float) z, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

  //  return visionController.atSetpoint();
  return false;
  }
}
