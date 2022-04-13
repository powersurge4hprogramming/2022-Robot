// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

/** Add your docs here. */
public class AutoTemplate {
    public static MecanumControllerCommand mechCommandWithDefaults(Trajectory trajectory, Drivetrain drivetrain) {
        return new MecanumControllerCommand(
                trajectory,
                drivetrain::getPose,
                Constants.DriveConstants.MOTOR_FEED_FORWARD,
                Constants.DriveConstants.DRIVE_KINEMATICS,

                // Position contollers
                new PIDController(Constants.DriveConstants.PX_CONTROLLER, 0, 0),
                new PIDController(Constants.DriveConstants.PY_CONTROLLER, 0, 0),
                new ProfiledPIDController(
                        Constants.DriveConstants.P_THETA_CONTROLLER, 0, 0,
                        Constants.DriveConstants.THETA_CONTROLLER_CONSTRAINTS),

                // Needed for normalizing wheel speeds
                Constants.DriveConstants.MAX_SPEED_M_PER_SEC,

                // Velocity PID's
                new PIDController(Constants.DriveConstants.FL_VELOCITY, 0, 0),
                new PIDController(Constants.DriveConstants.RL_VELOCITY, 0, 0),
                new PIDController(Constants.DriveConstants.FR_VELOCITY, 0, 0),
                new PIDController(Constants.DriveConstants.RR_VELOCITY, 0, 0),
                drivetrain::getCurrentWheelSpeeds,

                drivetrain::setDriveMotorControllersVolts, // Consumer for the output motor
                drivetrain);
    }
}
