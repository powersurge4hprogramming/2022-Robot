// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private final WPI_TalonFX shooterMotor;

  public Shooter(int shooterPort) {
    shooterMotor = new WPI_TalonFX(shooterPort);
    shooterMotor.setInverted(true);

    TalonFXConfiguration configs = new TalonFXConfiguration();
    /* select integ-sensor for PID0 (it doesn't matter if PID is actually used) */
    configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    /* config all the settings */
    shooterMotor.configAllSettings(configs);
    shooterMotor.setNeutralMode(NeutralMode.Coast);
  }

  public void setVelocity(double speed) {
    shooterMotor.set(ControlMode.PercentOutput, 0.1);
  }

  public void setPercentOutput(double speed) {
    shooterMotor.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    double selSenVel = shooterMotor.getSelectedSensorVelocity(); /* position units per 100ms */
		double vel_RotPerSec = selSenVel / 2048 * 10; /* scale per100ms to perSecond */
    double vel_RotPerMin = vel_RotPerSec * 60.0;
    double vel_MetersPerSec = vel_RotPerSec*2*Math.PI*0.1524;
    SmartDashboard.putNumber("Shooter Velocity", vel_MetersPerSec);
  }
}
