package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
    MecanumDrive Mecanum;


    public Drivetrain() {
        Mecanum= new MecanumDrive();
    }
}  