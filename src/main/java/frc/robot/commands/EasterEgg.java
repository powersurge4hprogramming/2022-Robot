// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DjKaleb;

public class EasterEgg extends CommandBase {
  private final DjKaleb dj;
  private int count = 1000;
  private final String musicPath = "rick-roll.chrp";

  public EasterEgg(DjKaleb dj) {
    this.dj = dj;
  }

  @Override
  public void initialize() {
    dj.loadMusic(musicPath);
  }

  @Override
  public void execute() {
    if (count == 0) {
      dj.playMusic();
    } else if (count > 0) {
      count--;
    }
  }

  @Override
  public void end(boolean interrupted) {
    dj.cancelMusic();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
