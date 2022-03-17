// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbRelease extends SubsystemBase {

  private final Servo releaseMotor;

  public ClimbRelease(int releaseMotor) {
    this.releaseMotor = new Servo(releaseMotor);
  }

  @Override
  public void periodic() {
  }

  public void release() {
    releaseMotor.setAngle(Constants.BehaviorConstants.RELEASE_SERVO_ANGLE);
  }
  public void setAngle(int angle) {
    releaseMotor.setAngle(angle);
  }
  
}
