// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DjKaleb extends SubsystemBase {

  private Orchestra orchestra;

  public DjKaleb(Shooter shooter, Climber climber) {
    orchestra = new Orchestra();
    orchestra.addInstrument(shooter.getTalon());
    orchestra.addInstrument(climber.getTalon());
  }

  public void loadMusic(String filePath) {
    orchestra.loadMusic(filePath);
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
