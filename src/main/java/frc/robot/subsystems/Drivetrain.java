package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
    private final MecanumDrive mecanum;

    private final RelativeEncoder frontLeftEncoder;
    private final RelativeEncoder frontRightEncoder;
    private final RelativeEncoder backLeftEncoder;
    private final RelativeEncoder backRightEncoder;

    private final CANSparkMax frontLeftSpark;
    private final CANSparkMax frontRightSpark;
    private final CANSparkMax backRightSpark;
    private final CANSparkMax backLeftSpark;

    private final Gyro gyro;

    private final MecanumDriveOdometry mecanumDriveOdometry;

    public Drivetrain(int frontLeftMotorControl, int frontRightMotorControl, int backLeftMotorControl,
            int backRightMotorControl) {

        frontLeftSpark = new CANSparkMax(frontLeftMotorControl, MotorType.kBrushless);
        frontRightSpark = new CANSparkMax(frontRightMotorControl, MotorType.kBrushless);
        backRightSpark = new CANSparkMax(backRightMotorControl, MotorType.kBrushless);
        backLeftSpark = new CANSparkMax(backLeftMotorControl, MotorType.kBrushless);

        frontLeftEncoder = frontLeftSpark.getEncoder();
        frontRightEncoder = frontRightSpark.getEncoder();
        backLeftEncoder = backLeftSpark.getEncoder();
        backRightEncoder = backRightSpark.getEncoder();

        gyro = new ADXRS450_Gyro();

        mecanumDriveOdometry = new MecanumDriveOdometry(DriveConstants.DRIVE_KINEMATICS, gyro.getRotation2d());

        mecanum = new MecanumDrive(frontLeftSpark, backLeftSpark, frontRightSpark, backRightSpark);
    }

    public void drive(float ySpeed, float xSpeed, float zRotation, boolean fieldRelative) {
        if (fieldRelative) {
            mecanum.driveCartesian(ySpeed, xSpeed, zRotation, -gyro.getAngle());
        } else {
            mecanum.driveCartesian(ySpeed, xSpeed, zRotation);
        }
    }

    @Override
    public void periodic() {
        mecanumDriveOdometry.update(gyro.getRotation2d(), getCurrentWheelSpeeds());
    }

    public Pose2d getPose() {
        return mecanumDriveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        mecanumDriveOdometry.resetPosition(pose, gyro.getRotation2d());
    }

    public void setDriveMotorControllersVolts(MecanumDriveMotorVoltages volts) {
        frontLeftSpark.setVoltage(volts.frontLeftVoltage);
        frontRightSpark.setVoltage(volts.rearLeftVoltage);
        backRightSpark.setVoltage(volts.frontRightVoltage);
        backLeftSpark.setVoltage(volts.rearRightVoltage);
    }

    public MecanumDriveWheelSpeeds getCurrentWheelSpeeds() {
        // TODO: fix with setConversionFactor
        return new MecanumDriveWheelSpeeds(frontLeftEncoder.getVelocity(),
                frontRightEncoder.getVelocity(), backLeftEncoder.getVelocity(), backRightEncoder.getVelocity());
    }

}