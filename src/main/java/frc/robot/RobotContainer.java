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
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.GatekeeperCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.MechAimCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gatekeeper;
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
  // Input Devices
  private final GenericHID m_driveJoystick = new GenericHID(Constants.InputConstants.DRIVER_JOYSTICK_PORT);
  private final GenericHID m_operatorJoystick = new GenericHID(Constants.InputConstants.OPERATOR_JOYSTICK_PORT);

  // Subsystems
  private final Drivetrain m_drivetrain = new Drivetrain(Constants.MotorConstants.FRONT_LEFT_MOTOR_CONTROL,
      Constants.MotorConstants.FRONT_RIGHT_MOTOR_CONTROL, Constants.MotorConstants.BACK_LEFT_MOTOR_CONTROL,
      Constants.MotorConstants.BACK_RIGHT_MOTOR_CONTROL);
  private final Shooter m_shooter = new Shooter(Constants.MotorConstants.SHOOTER_PORT);
  private final Intake m_Intake = new Intake(Constants.MotorConstants.INTAKE_PORT,
      Constants.MotorConstants.TROUGH_PORT);
  private final Gatekeeper m_gatekeeper = new Gatekeeper(Constants.MotorConstants.GATEKEEPER_PORT);
  private final Climber m_climber = new Climber(Constants.MotorConstants.CLIMBER_PORT,
      Constants.MotorConstants.RELEASE_MOTOR_PORT);

  // Commands
  private final DriveCommand m_teleopCommand = new DriveCommand(m_drivetrain, m_driveJoystick);
  private final ShooterCommand m_shooterCommand = new ShooterCommand(m_shooter);

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
    new JoystickButton(m_operatorJoystick, Constants.InputConstants.INTAKE_BUTTON)
        .whenHeld(new IntakeCommand(m_Intake));

    new JoystickButton(m_operatorJoystick, Constants.InputConstants.MECH_AIM_BUTTON)
        .whenHeld(new MechAimCommand(m_drivetrain));

    new JoystickButton(m_operatorJoystick, Constants.InputConstants.GATEKEEPER_ALLOW_BUTTON)
        .whenPressed(
            new GatekeeperCommand(m_gatekeeper).withTimeout(Constants.BehaviorConstants.GATEKEEPER_ALLOW_TIME));

    new JoystickButton(m_operatorJoystick, Constants.InputConstants.CLIMB_BUTTON).whenHeld(new ClimbCommand(m_climber));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        Constants.DriveConstants.MAX_SPEED_M_PER_SEC,
        Constants.DriveConstants.MAX_ACCEL_M_PER_SEC_SQUARED)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.DriveConstants.DRIVE_KINEMATICS);

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
        Constants.DriveConstants.MOTOR_FEED_FORWARD,
        Constants.DriveConstants.DRIVE_KINEMATICS,

        // Position contollers
        new PIDController(Constants.DriveConstants.PX_CONTROLLER, 0, 0),
        new PIDController(Constants.DriveConstants.PY_CONTROLLER, 0, 0),
        new ProfiledPIDController(
            Constants.DriveConstants.P_THETA_CONTROLLER, 0, 0, Constants.DriveConstants.THETA_CONTROLLER_CONSTRAINTS),

        // Needed for normalizing wheel speeds
        Constants.DriveConstants.MAX_SPEED_M_PER_SEC,

        // Velocity PID's
        new PIDController(Constants.DriveConstants.FL_VELOCITY, 0, 0),
        new PIDController(Constants.DriveConstants.RL_VELOCITY, 0, 0),
        new PIDController(Constants.DriveConstants.FR_VELOCITY, 0, 0),
        new PIDController(Constants.DriveConstants.RR_VELOCITY, 0, 0),
        m_drivetrain::getCurrentWheelSpeeds,
        m_drivetrain::setDriveMotorControllersVolts, // Consumer for the output motor voltages
        m_drivetrain);

    // Reset odometry to the starting pose of the trajectory.
    m_drivetrain.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return mecanumControllerCommand.andThen(() -> m_drivetrain.drive(0, 0, 0, false));
  }
}
