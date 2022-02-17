// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private final CANSparkMax shooterMotor;

    private final Servo servo;

  public Shooter (int shooterPort, int linearActuatorPort) {
    shooterMotor = new CANSparkMax(shooterPort, MotorType.kBrushed);
    servo = new Servo(linearActuatorPort);
  }

  public void set(float speed){
    shooterMotor.set(speed);
  }

  public void setactuator(float speed) {
    servo.setSpeed(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
