// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.data_structs.LimeVision;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.MathU;

public class MechAimCommand extends CommandBase {

  private final Drivetrain driveTrain;
  private final PIDController visionController;

  public MechAimCommand(Drivetrain driveTrain) {
    this.driveTrain = driveTrain;

    visionController = new PIDController(Constants.VisionConstants.VISION_PID_X, Constants.VisionConstants.VISION_PID_I,
        Constants.VisionConstants.VISION_PID_D);
    addRequirements(this.driveTrain);
  }

  @Override
  public void initialize() {
    visionController.setTolerance(Constants.VisionConstants.VISION_PID_TOLERANCE);
    visionController.setSetpoint(0);
  }

  @Override
  public void execute() {
    double xOffset = LimeVision.getXOffsetAngle() * -1;

    // -27 to 27 is the range of the xOffset passed by the limelight
    xOffset = MathU.remap(-27, 27, -1, 1, xOffset);
    double z = visionController.calculate(xOffset);

    SmartDashboard.putNumber("MechAim Z", z);
    driveTrain.drive(0f, 0f, (float) z, false);
    SmartDashboard.putBoolean("Aimed?", visionController.atSetpoint());
  }

  @Override
  public void end(boolean interrupted) {
    driveTrain.drive(0, 0, 0, false);
  }

  @Override
  public boolean isFinished() {
  //  return visionController.atSetpoint(); TODO
  return false;
  }
}
