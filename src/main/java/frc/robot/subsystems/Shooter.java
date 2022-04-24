// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  private final WPI_TalonFX shooterMotor;

  public Shooter(int shooterPort) {
    shooterMotor = new WPI_TalonFX(shooterPort);

    /* config all the settings */
    shooterMotor.configFactoryDefault();
    shooterMotor.setInverted(true);

    shooterMotor.configNeutralDeadband(Constants.ShooterConstants.TALON_NEUTRAL_DEADBAND);
    shooterMotor.setNeutralMode(NeutralMode.Coast);

    shooterMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
        0,
        Constants.ShooterConstants.TALON_TIMEOUT);

    shooterMotor.configNominalOutputForward(0,
        Constants.ShooterConstants.TALON_TIMEOUT);
    shooterMotor.configNominalOutputReverse(0,
        Constants.ShooterConstants.TALON_TIMEOUT);
    shooterMotor.configPeakOutputForward(1,
        Constants.ShooterConstants.TALON_TIMEOUT);
    shooterMotor.configPeakOutputReverse(-1,
        Constants.ShooterConstants.TALON_TIMEOUT);

    shooterMotor.config_kF(0, Constants.ShooterConstants.TALON_KF,
        Constants.ShooterConstants.TALON_TIMEOUT);
    shooterMotor.config_kP(0, Constants.ShooterConstants.TALON_KP,
        Constants.ShooterConstants.TALON_TIMEOUT);
    shooterMotor.config_kI(0, Constants.ShooterConstants.TALON_KI,
        Constants.ShooterConstants.TALON_TIMEOUT);
    shooterMotor.config_kD(0, Constants.ShooterConstants.TALON_KD,
        Constants.ShooterConstants.TALON_TIMEOUT);

  }

  public void setVelocity(double speed) {
    shooterMotor.set(TalonFXControlMode.Velocity, speed);
  }

  public void setPercentOutput(double speed) {
    shooterMotor.set(TalonFXControlMode.PercentOutput, speed);
  }

  public WPI_TalonFX getTalon() {
    return shooterMotor;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Velocity", shooterMotor.getSelectedSensorVelocity(0));
  }
}
