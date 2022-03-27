// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class EasterEgg extends CommandBase {
  Shooter shooter;
  int count = 1000;

  /*
   * Talon FXs to play music through.
   * More complex music MIDIs will contain several tracks, requiring multiple
   * instruments.
   */
  public EasterEgg(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    shooter.loadMusic();
  }

  @Override
  public void execute() {
    if (count == 0) {
      shooter.playMusic();
    } else if (count > 0) {
      count--;
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.cancelMusic();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
