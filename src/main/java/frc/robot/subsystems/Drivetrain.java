package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

    private final MecanumDrive mecanumDrive;

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

        frontLeftSpark.setIdleMode(IdleMode.kCoast);
        frontRightSpark.setIdleMode(IdleMode.kCoast);
        backLeftSpark.setIdleMode(IdleMode.kCoast);
        backRightSpark.setIdleMode(IdleMode.kCoast);

        frontRightSpark.setInverted(true);
        backRightSpark.setInverted(true);

        frontLeftEncoder = frontLeftSpark.getEncoder();
        frontRightEncoder = frontRightSpark.getEncoder();
        backLeftEncoder = backLeftSpark.getEncoder();
        backRightEncoder = backRightSpark.getEncoder();

        frontLeftEncoder.setPositionConversionFactor(Constants.DriveConstants.DIST_PER_COUNT_NEO);
        frontRightEncoder.setPositionConversionFactor(Constants.DriveConstants.DIST_PER_COUNT_NEO);
        backLeftEncoder.setPositionConversionFactor(Constants.DriveConstants.DIST_PER_COUNT_NEO);
        backRightEncoder.setPositionConversionFactor(Constants.DriveConstants.DIST_PER_COUNT_NEO);

        gyro = new ADXRS450_Gyro();
        gyro.calibrate();

        mecanumDriveOdometry = new MecanumDriveOdometry(Constants.DriveConstants.DRIVE_KINEMATICS,
                gyro.getRotation2d());

        mecanumDrive = new MecanumDrive(frontLeftSpark, backLeftSpark, frontRightSpark, backRightSpark);
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
        frontLeftEncoder.setPosition(0);
        frontRightEncoder.setPosition(0);
        backLeftEncoder.setPosition(0);
        backRightEncoder.setPosition(0);
    }

    public void drive(float ySpeed, float xSpeed, float zRotation, boolean fieldRelative) {
        if (fieldRelative) {
            mecanumDrive.driveCartesian(ySpeed, xSpeed, zRotation, -gyro.getAngle());
        } else {
            mecanumDrive.driveCartesian(ySpeed, xSpeed, zRotation);
        }
    }

    public void setDriveMotorControllersVolts(MecanumDriveMotorVoltages volts) {
        frontLeftSpark.setVoltage(volts.frontLeftVoltage);
        frontRightSpark.setVoltage(volts.frontRightVoltage);
        backRightSpark.setVoltage(volts.rearRightVoltage);
        backLeftSpark.setVoltage(volts.rearLeftVoltage);
    }

    public MecanumDriveWheelSpeeds getCurrentWheelSpeeds() {
        return new MecanumDriveWheelSpeeds(frontLeftEncoder.getVelocity(),
                frontRightEncoder.getVelocity(), backLeftEncoder.getVelocity(), backRightEncoder.getVelocity());
    }

    public void resetEncoders() {
        frontLeftEncoder.setPosition(0);
        frontRightEncoder.setPosition(0);
        backLeftEncoder.setPosition(0);
        backRightEncoder.setPosition(0);
        gyro.reset();
    }

}