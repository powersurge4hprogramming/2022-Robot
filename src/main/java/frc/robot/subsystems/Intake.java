// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final CANSparkMax CANintake;

  public Intake(int intakeport) {
    CANintake = new CANSparkMax(intakeport, MotorType.kBrushed);
  }

  public void set(Float speed){
CANintake.set(speed);
  }
}
