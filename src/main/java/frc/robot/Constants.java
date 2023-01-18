// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
        public static final class DriveConstants {

                // DONE:
                public static final double TRACK_WIDTH = 0.586;
                // Distance between centers of right and left wheels on robot
                public static final double WHEEL_BASE = 0.516;
                // Distance between centers of front and back wheels on robot

                public static final MecanumDriveKinematics DRIVE_KINEMATICS = new MecanumDriveKinematics(
                                new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
                                new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
                                new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
                                new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2));

                public static final double DIST_PER_COUNT_NEO = (0.1524 * Math.PI / 8.45);

                // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
                // These characterization values MUST be determined either experimentally or
                // theoretically
                // for *your* robot's drive.
                // The SysId tool provides a convenient method for obtaining these values for
                // your robot.
                public static final SimpleMotorFeedforward MOTOR_FEED_FORWARD = new SimpleMotorFeedforward(0.1962, 2.2131,
                0.41191);

                // TODO Sysid Values
                public static final double FL_VELOCITY = 3.0239;
                public static final double RL_VELOCITY = 3.0239;
                public static final double FR_VELOCITY = 3.0239;
                public static final double RR_VELOCITY = 3.0239;

                public static final double MAX_SPEED_M_PER_SEC = 3;
                public static final double MAX_ACCEL_M_PER_SEC_SQUARED = 3;
                public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SEC = Math.PI;
                public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SEC_SQUARED = Math.PI;

                public static final double PX_CONTROLLER = 0.5;
                public static final double PY_CONTROLLER = 0.5;
                public static final double P_THETA_CONTROLLER = 0.5;

                // Constraint for the motion profilied robot angle controller
                public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(
                                MAX_ANGULAR_SPEED_RADIANS_PER_SEC, MAX_ANGULAR_SPEED_RADIANS_PER_SEC_SQUARED);
        }

        public static final class VisionConstants {

                // TODO SysId
                public static final double VISION_PID_P = 0.4;
                public static final double VISION_PID_I = 0.1;
                public static final double VISION_PID_D = 0;
                public static final double VISION_PID_TOLERANCE = 0.035;

                // DIST in meters
                public static final double DIST_LIME_HEIGHT = 0.508;
                public static final double DIST_HUB_HEIGHT = 2.64;
                public static final double DIST_LIME_MOUNT_ANGLE = 20.9;

        }

        public final static class MotorConstants {

                public static final int FRONT_LEFT_MOTOR_CONTROL = 5;
                public static final int FRONT_RIGHT_MOTOR_CONTROL = 10;
                public static final int BACK_LEFT_MOTOR_CONTROL = 11;
                public static final int BACK_RIGHT_MOTOR_CONTROL = 14;

                public static final int INTAKE_PORT = 9;
                public static final int SHOOTER_PORT = 17;
                public static final int TROUGH_PORT = 12;
                public static final int GATEKEEPER_PORT = 7;

                public static final int CLIMBER_PORT = 18;

                public static final int CLIMB_RELEASE_SERVO_PORT = 4; // PWM
                public static final int FINGER_RELEASE_SERVO_PORT = 5; // PWM

        }

        public final static class ShooterConstants {

                public static final double TALON_KP = 0.0; // 0.18
                public static final double TALON_KI = 0.000; // 0.002
                public static final double TALON_KD = 0; // 4
                public static final double TALON_KF = (1*1023.0)/22071.0; // Second number is velocity units at general % output

                public static final int TALON_TIMEOUT = 30;

                public static final double TALON_NEUTRAL_DEADBAND = 0.001;
        }

        public final static class BehaviorConstants {
                public static final double GATEKEEPER_ALLOW_TIME = 0.5;

                public static final double CLIMBER_SPEED = 0.6;
                public static final double GATEKEEPER_SPEED = 1.0;
                public static final double INTAKE_SPEED = 1.0;

                public static final int CLIMB_RELEASE_SERVO_ANGLE = 0;
                public static final int CLIMB_START_SERVO_ANGLE = 90;

                public static final int FINGER_RELEASE_SERVO_ANGLE = 90; // TODO
                public static final int FINGER_START_SERVO_ANGLE = 0;
        }

        public final static class InputConstants {

                public static final int DRIVER_JOYSTICK_PORT = 0;
                public static final int OPERATOR_JOYSTICK_PORT = 1;

                public static final int DRIVER_JOYSTICK_X_AXIS = 0;
                public static final int DRIVER_JOYSTICK_Y_AXIS = 1;
                public static final int DRIVER_JOYSTICK_Z_AXIS = 2;
                public static final int DRIVER_JOYSTICK_SCALE_AXIS = 3;

                public static final double DRIVER_LATERAL_SLEW = 5;
                public static final double DRIVER_TWIST_SLEW = 5;

                //public static final int INTAKE_BUTTON = 1;
                public static final int INTAKE_BUTTON = 2;

                public static final int MECH_AIM_BUTTON = 5;


                //public static final int GATEKEEPER_ALLOW_BUTTON = 2;
                public static final int GATEKEEPER_ALLOW_BUTTON = 4; 

                //public static final int CLIMB_BUTTON = 11;
                public static final int CLIMB_BUTTON = 1;

                //public static final int CLIMB_RELEASE_BUTTON = 9;
                public static final int CLIMB_RELEASE_BUTTON = 3;

                //public static final int FINGER_RELEASE_BUTTON = 10;
                public static final int FINGER_RELEASE_BUTTON = 7;

                // This is the line that conflicts with the finger release
                public static final int Shooter_Speed = 7;

                //public static final int MUSIC_CONTROL_BUTTON = 8;
                public static final int MUSIC_CONTROL_BUTTON = 10;

        }

        public final static class DashboardConstants {
                public static final String DASHBOARD_SHOOTER_SPEED_KEY = "Shooter Speed";
        }

}
