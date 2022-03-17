// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  private final WPI_TalonFX climberMotor;

  public Climber(int climberMotor, int releaseMotor) {
    this.climberMotor = new WPI_TalonFX(climberMotor);
  }

  @Override
  public void periodic() {
  }

  public void runMotor(double speed) {
    climberMotor.set(speed);
  }
}
