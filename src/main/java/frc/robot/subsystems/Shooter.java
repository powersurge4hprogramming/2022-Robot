// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utilities.Gains;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private final WPI_TalonFX shooterMotor;

  public Shooter(int shooterPort) {
    int timeout = 30;
    Gains kGains_Velocit = new Gains(0.10, 0.001, 5, 1023.0 / 21940.0, 300, 1.00);
    shooterMotor = new WPI_TalonFX(shooterPort);
    shooterMotor.setInverted(true);
    /* config all the settings */
    shooterMotor.configFactoryDefault();
    shooterMotor.configNeutralDeadband(0.001);
    shooterMotor.setNeutralMode(NeutralMode.Coast);

    shooterMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
        0,
        timeout);

    shooterMotor.configNominalOutputForward(0, timeout);
    shooterMotor.configNominalOutputReverse(0, timeout);
    shooterMotor.configPeakOutputForward(1, timeout);
    shooterMotor.configPeakOutputReverse(-1, timeout);

    shooterMotor.config_kF(0, kGains_Velocit.kF, timeout);
    shooterMotor.config_kP(0, kGains_Velocit.kP, timeout);
    shooterMotor.config_kI(0, kGains_Velocit.kI, timeout);
    shooterMotor.config_kD(0, kGains_Velocit.kD, timeout);

  }

  public void setVelocity(double speed) {
    shooterMotor.set(ControlMode.Velocity, 1650);
  }

  public void setPercentOutput(double speed) {
    shooterMotor.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Velocity", shooterMotor.getSelectedSensorVelocity());
  }
}
