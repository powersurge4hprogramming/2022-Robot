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
import frc.robot.commands.LimeReader;
import frc.robot.commands.MechAimCommand;
import frc.robot.commands.VisionShooterCommand;
import frc.robot.commands.DashboardShooterCommand;
import frc.robot.subsystems.ClimbRelease;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gatekeeper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

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
                        Constants.MotorConstants.FRONT_RIGHT_MOTOR_CONTROL,
                        Constants.MotorConstants.BACK_LEFT_MOTOR_CONTROL,
                        Constants.MotorConstants.BACK_RIGHT_MOTOR_CONTROL);
        private final Shooter m_shooter = new Shooter(Constants.MotorConstants.SHOOTER_PORT);
        private final Intake m_Intake = new Intake(Constants.MotorConstants.INTAKE_PORT,
                        Constants.MotorConstants.TROUGH_PORT);
        private final Gatekeeper m_gatekeeper = new Gatekeeper(Constants.MotorConstants.GATEKEEPER_PORT);
        private final Climber m_climber = new Climber(Constants.MotorConstants.CLIMBER_PORT,
                        Constants.MotorConstants.RELEASE_MOTOR_PORT);
        private final ClimbRelease m_climbRelease = new ClimbRelease(Constants.MotorConstants.RELEASE_MOTOR_PORT);

        // Commands
        private final DriveCommand m_teleopCommand = new DriveCommand(m_drivetrain, m_driveJoystick);
        private final DashboardShooterCommand m_shooterCommand = new DashboardShooterCommand(m_shooter);
        private final LimeReader m_limeReader = new LimeReader();

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Configure the button bindings
                configureButtonBindings();
        }

        public void teleopStart() {
                m_limeReader.schedule();
                m_drivetrain.setDefaultCommand(m_teleopCommand);
                m_shooter.setDefaultCommand(m_shooterCommand);
                m_climbRelease.setAngle(90);
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
                                .whenHeld(new ParallelCommandGroup( // new VisionShooterCommand(m_shooter),
                                                new MechAimCommand(m_drivetrain)));

                new JoystickButton(m_operatorJoystick, Constants.InputConstants.GATEKEEPER_ALLOW_BUTTON)
                                .whenPressed(
                                                new GatekeeperCommand(m_gatekeeper)
                                                                .withTimeout(Constants.BehaviorConstants.GATEKEEPER_ALLOW_TIME));

                new JoystickButton(m_operatorJoystick, Constants.InputConstants.CLIMB_BUTTON)
                                .whenHeld(new ClimbCommand(m_climber));

                // Shooter presets FIX!!
                new POVButton(m_operatorJoystick, 0)
                                .whenHeld(new RunCommand(() -> m_shooter.setPercentOutput(1.00), m_shooter));
                new POVButton(m_operatorJoystick, 90)
                                .whenHeld(new RunCommand(() -> m_shooter.setPercentOutput(0.95), m_shooter));
                new POVButton(m_operatorJoystick, 270)
                                .whenHeld(new RunCommand(() -> m_shooter.setPercentOutput(0.89), m_shooter));
                new POVButton(m_operatorJoystick, 180)
                                .whenHeld(new RunCommand(() -> m_shooter.setPercentOutput(0.60), m_shooter));
                new POVButton(m_operatorJoystick, 225)
                                .whenHeld(new RunCommand(() -> m_shooter.setPercentOutput(0.81), m_shooter));
                new POVButton(m_operatorJoystick, 315)
                                .whenHeld(new RunCommand(() -> m_shooter.setPercentOutput(0.77), m_shooter));
                new POVButton(m_operatorJoystick, 45)
                                .whenHeld(new RunCommand(() -> m_shooter.setPercentOutput(0.45), m_shooter));

                new JoystickButton(m_operatorJoystick, Constants.InputConstants.CLIMB_RELEASE_BUTTON_1)
                                .whenPressed(() -> m_climbRelease.release(), m_climbRelease);

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         * 
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                m_climbRelease.setAngle(90);
                // Create config for trajectory
                /*
                 * TrajectoryConfig config = new TrajectoryConfig(
                 * Constants.DriveConstants.MAX_SPEED_M_PER_SEC,
                 * Constants.DriveConstants.MAX_ACCEL_M_PER_SEC_SQUARED)
                 * // Add kinematics to ensure max speed is actually obeyed
                 * .setKinematics(Constants.DriveConstants.DRIVE_KINEMATICS);
                 * 
                 * Trajectory mechTrajectory1 = TrajectoryGenerator.generateTrajectory(
                 * // Start at the origin facing the +X direction
                 * new Pose2d(0, 0, new Rotation2d(0)),
                 * // Pass through nothing
                 * List.of(),
                 * // End 1.03 meters backwards ahead of where we started, facing forward
                 * new Pose2d(1.03, 0, Rotation2d.fromDegrees(180)),
                 * config);
                 * 
                 * Trajectory mechTrajectory2 = TrajectoryGenerator.generateTrajectory(
                 * // Start at the origin facing the +X direction
                 * new Pose2d(0, 0, new Rotation2d(0)),
                 * // Pass through these two interior waypoints, making an 's' curve path
                 * List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                 * // End 3 meters straight ahead of where we started, facing forward
                 * new Pose2d(3, 0, new Rotation2d(0)),
                 * config);
                 * 
                 * MecanumControllerCommand mechMove1 = new MecanumControllerCommand(
                 * mechTrajectory1,
                 * m_drivetrain::getPose,
                 * Constants.DriveConstants.MOTOR_FEED_FORWARD,
                 * Constants.DriveConstants.DRIVE_KINEMATICS,
                 * 
                 * // Position contollers
                 * new PIDController(Constants.DriveConstants.PX_CONTROLLER, 0, 0),
                 * new PIDController(Constants.DriveConstants.PY_CONTROLLER, 0, 0),
                 * new ProfiledPIDController(
                 * Constants.DriveConstants.P_THETA_CONTROLLER, 0, 0,
                 * Constants.DriveConstants.THETA_CONTROLLER_CONSTRAINTS),
                 * 
                 * // Needed for normalizing wheel speeds
                 * Constants.DriveConstants.MAX_SPEED_M_PER_SEC,
                 * 
                 * // Velocity PID's
                 * new PIDController(Constants.DriveConstants.FL_VELOCITY, 0, 0),
                 * new PIDController(Constants.DriveConstants.RL_VELOCITY, 0, 0),
                 * new PIDController(Constants.DriveConstants.FR_VELOCITY, 0, 0),
                 * new PIDController(Constants.DriveConstants.RR_VELOCITY, 0, 0),
                 * m_drivetrain::getCurrentWheelSpeeds,
                 * m_drivetrain::setDriveMotorControllersVolts, // Consumer for the output motor
                 * voltages
                 * m_drivetrain);
                 * 
                 * MecanumControllerCommand mechMove2 = new MecanumControllerCommand(
                 * mechTrajectory2,
                 * m_drivetrain::getPose,
                 * Constants.DriveConstants.MOTOR_FEED_FORWARD,
                 * Constants.DriveConstants.DRIVE_KINEMATICS,
                 * 
                 * // Position contollers
                 * new PIDController(Constants.DriveConstants.PX_CONTROLLER, 0, 0),
                 * new PIDController(Constants.DriveConstants.PY_CONTROLLER, 0, 0),
                 * new ProfiledPIDController(
                 * Constants.DriveConstants.P_THETA_CONTROLLER, 0, 0,
                 * Constants.DriveConstants.THETA_CONTROLLER_CONSTRAINTS),
                 * 
                 * // Needed for normalizing wheel speeds
                 * Constants.DriveConstants.MAX_SPEED_M_PER_SEC,
                 * 
                 * // Velocity PID's
                 * new PIDController(Constants.DriveConstants.FL_VELOCITY, 0, 0),
                 * new PIDController(Constants.DriveConstants.RL_VELOCITY, 0, 0),
                 * new PIDController(Constants.DriveConstants.FR_VELOCITY, 0, 0),
                 * new PIDController(Constants.DriveConstants.RR_VELOCITY, 0, 0),
                 * m_drivetrain::getCurrentWheelSpeeds,
                 * m_drivetrain::setDriveMotorControllersVolts, // Consumer for the output motor
                 * voltages
                 * m_drivetrain);
                 * 
                 * // Reset odometry to the starting pose of the trajectory.
                 * m_drivetrain.resetOdometry(mechTrajectory1.getInitialPose());
                 * 
                 * ParallelRaceGroup auto1 = new ParallelRaceGroup(
                 * new IntakeCommand(m_Intake),
                 * new SequentialCommandGroup(
                 * mechMove1.andThen(() -> m_drivetrain.drive(0, 0, 0, false)),
                 * new ParallelRaceGroup(new MechAimCommand(m_drivetrain)
                 * .andThen(new GatekeeperCommand(m_gatekeeper)
                 * .withTimeout(
                 * Constants.BehaviorConstants.GATEKEEPER_ALLOW_TIME
                 * 2)),
                 * new VisionShooterCommand(m_shooter)),
                 * mechMove2.andThen(() -> m_drivetrain.drive(0, 0, 0, false))
                 * 
                 * ));
                 */

                // If we cant get auto working
                // new InstantCommand(toRun, m_drivetrain);

                SequentialCommandGroup hackDrive = new SequentialCommandGroup(
                                // Drive 0.2 back for 0.75 seconds
                                new RunCommand(() -> m_drivetrain.drive(-0.2f, 0f, 0f, false),
                                                m_drivetrain).withTimeout(0.75),

                                // Spin up shooter to 0.65 for 4 seconds, then run gatekeeper while shooting,
                                // then end after 5 seconds
                                new ParallelRaceGroup(
                                                new RunCommand((() -> m_shooter.setPercentOutput(0.75)), m_shooter),
                                                new SequentialCommandGroup(new WaitCommand(2),
                                                                new GatekeeperCommand(m_gatekeeper).withTimeout(2))),

                                new InstantCommand((() -> m_shooter.setPercentOutput(0.0)), m_shooter),

                                new RunCommand(() -> m_drivetrain.drive(0f, 0f, 0.3f, false),
                                                m_drivetrain).withTimeout(0.9));

                return hackDrive;

                // Run path following command, then stop at the end.
        }
}
