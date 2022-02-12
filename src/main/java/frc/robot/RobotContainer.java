// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.Intakecommand;
import frc.robot.commands.Shootercommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final GenericHID m_driveJoystick = new GenericHID(Constants.DRIVER_JOYSTICK_PORT);
  private final GenericHID m_operatorJoystick = new GenericHID(Constants.OPERATOR_JOYSTICK_PORT);
  private final Drivetrain m_drivetrain = new Drivetrain(DriveConstants.FRONT_LEFT_MOTOR_CONTROL,
      DriveConstants.FRONT_RIGHT_MOTOR_CONTROL, DriveConstants.BACK_LEFT_MOTOR_CONTROL,
      DriveConstants.BACK_RIGHT_MOTOR_CONTROL);
  private final Shooter m_shooter = new Shooter(Constants.Shooterport, Constants.servoport);

  private final Intake m_Intake = new Intake(Constants.intake);

  private final DriveCommand m_teleopCommand = new DriveCommand(m_drivetrain, m_driveJoystick);
  private final Shootercommand m_shooterCommand = new Shootercommand(m_shooter, m_operatorJoystick);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  public void teleopStart() {
    m_drivetrain.setDefaultCommand(m_teleopCommand);
    m_shooter.setDefaultCommand(m_shooterCommand);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_operatorJoystick, Constants.Intakebutton).whenHeld(new Intakecommand(m_Intake));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    MecanumControllerCommand mecanumControllerCommand = new MecanumControllerCommand(
        exampleTrajectory,
        m_drivetrain::getPose,
        DriveConstants.kFeedforward,
        DriveConstants.kDriveKinematics,

        // Position contollers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints),

        // Needed for normalizing wheel speeds
        AutoConstants.kMaxSpeedMetersPerSecond,

        // Velocity PID's
        new PIDController(DriveConstants.kPFrontLeftVel, 0, 0),
        new PIDController(DriveConstants.kPRearLeftVel, 0, 0),
        new PIDController(DriveConstants.kPFrontRightVel, 0, 0),
        new PIDController(DriveConstants.kPRearRightVel, 0, 0),
        m_drivetrain::getCurrentWheelSpeeds,
        m_drivetrain::setDriveMotorControllersVolts, // Consumer for the output motor voltages
        m_drivetrain);

    // Reset odometry to the starting pose of the trajectory.
    m_drivetrain.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return mecanumControllerCommand.andThen(() -> m_drivetrain.drive(0, 0, 0, false));
  }
}
