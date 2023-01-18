// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DjKaleb extends SubsystemBase {

  private Orchestra orchestra;
  private final List<SubsystemBase> subsystems;

  // TODO - Change this to generic motors, and make it a collection instead so you
  // can pass in as many as you want
  public DjKaleb(Shooter shooter, Climber climber) {
    orchestra = new Orchestra();
    subsystems = new ArrayList<>();
    orchestra.addInstrument(shooter.getTalon());
    orchestra.addInstrument(climber.getTalon());
    subsystems.add(shooter);
    subsystems.add(climber);
  }

  public DjKaleb(SubsystemBase[] subsystems, WPI_TalonFX... motors) {
    orchestra = new Orchestra();
    this.subsystems = new ArrayList<>();
    for (SubsystemBase subsystem : subsystems) {
      this.subsystems.add(subsystem);
    }
    for (WPI_TalonFX motor : motors) {
      orchestra.addInstrument(motor);
    }
  }

  public void loadMusic(String filePath) {
    System.out.println(orchestra.loadMusic(filePath));
  }

  public ErrorCode playMusic() {
    return orchestra.play();
  }

  public void cancelMusic() {
    orchestra.stop();
  }

  @Override
  public void periodic() {
  }
}
