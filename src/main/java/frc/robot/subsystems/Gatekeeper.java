// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gatekeeper extends SubsystemBase {

  private final CANSparkMax gatekeeperMotor;

  public Gatekeeper(int gatekeeperPort) {
    gatekeeperMotor = new CANSparkMax(gatekeeperPort, MotorType.kBrushed);
    gatekeeperMotor.setInverted(true);
    SmartDashboard.putNumber("TEST GATE", 0.0);
  }

  @Override
  public void periodic() {
    double shooterSpeed = SmartDashboard.getNumber("TEST GATE", 0.0);
    gatekeeperMotor.set(shooterSpeed);
    SmartDashboard.putNumber("TEST GATE", shooterSpeed);
  }

  public void set(double speed) {
    gatekeeperMotor.set(speed);
  }
}
