// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private final WPI_TalonFX climberMotor;
  private final Servo releaseMotor;

  /** Creates a new Climber. */
  public Climber(int climberMotor, int releaseMotor) {
    this.climberMotor = new WPI_TalonFX(climberMotor);
    this.releaseMotor = new Servo(releaseMotor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void release() {
    releaseMotor.setAngle(Constants.BehaviorConstants.RELEASE_SERVO_ANGLE);
  }

  public void runMotor(double speed) {
    climberMotor.set(speed);
  }
}
