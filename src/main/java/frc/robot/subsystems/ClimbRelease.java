// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbRelease extends SubsystemBase {

  private final Servo climbReleaseServo;
  private final Servo fingerReleaseServo;

  public ClimbRelease(int climbReleaseServo, int fingerReleaseServo) {
    this.climbReleaseServo = new Servo(climbReleaseServo);
    this.fingerReleaseServo = new Servo(fingerReleaseServo);

  }

  @Override
  public void periodic() {
  }

  public void clampAll() {
    climbReleaseServo.setAngle(Constants.BehaviorConstants.CLIMB_START_SERVO_ANGLE);
    fingerReleaseServo.setAngle(Constants.BehaviorConstants.FINGER_START_SERVO_ANGLE);
  }

  public void releaseClimber() {
    climbReleaseServo.setAngle(Constants.BehaviorConstants.CLIMB_RELEASE_SERVO_ANGLE);
  }

  public void releaseFinger() {
    fingerReleaseServo.setAngle(Constants.BehaviorConstants.FINGER_RELEASE_SERVO_ANGLE);
  }

}
