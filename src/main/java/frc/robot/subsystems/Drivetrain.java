package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
    MecanumDrive Mecanum;

    public Drivetrain(int frontLeftMotorControl, int frontRightMotorControl, int backLeftMotorControl,
            int backRightMotorControl) {
        CANSparkMax frontLeftSpark = new CANSparkMax(frontLeftMotorControl, MotorType.kBrushless);
        CANSparkMax frontRightSpark = new CANSparkMax(frontRightMotorControl, MotorType.kBrushless);
        CANSparkMax backRightSpark = new CANSparkMax(backRightMotorControl, MotorType.kBrushless);
        CANSparkMax backLeftSpark = new CANSparkMax(backLeftMotorControl, MotorType.kBrushless);
        Mecanum = new MecanumDrive(frontLeftSpark, backLeftSpark, frontRightSpark, backRightSpark);
    }

    public void drive(float ySpeed, float xSpeed, float zRotation) {
        Mecanum.driveCartesian(ySpeed, xSpeed, zRotation);
    }

}