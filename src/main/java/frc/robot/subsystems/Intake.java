// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private final CANSparkMax intakeMotor;
  private final CANSparkMax troughMotor;

  public Intake(int intakePort, int troughPort) {
    intakeMotor = new CANSparkMax(intakePort, MotorType.kBrushed);
    troughMotor = new CANSparkMax(troughPort, MotorType.kBrushed);
    troughMotor.setInverted(true);
  }

  @Override
  public void periodic() {
  }

  public void set(double speed) {
    intakeMotor.set(speed);
    troughMotor.set(speed);
  }
}
