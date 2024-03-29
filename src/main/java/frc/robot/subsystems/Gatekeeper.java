// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gatekeeper extends SubsystemBase {

  private final CANSparkMax gatekeeperMotor;

  public Gatekeeper(int gatekeeperPort) {
    gatekeeperMotor = new CANSparkMax(gatekeeperPort, MotorType.kBrushed);
    gatekeeperMotor.setInverted(true);
  }

  @Override
  public void periodic() {
  }

  public void set(double speed) {
    gatekeeperMotor.set(speed);
  }
}
