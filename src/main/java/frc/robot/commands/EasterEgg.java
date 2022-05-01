// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DjKaleb;

public class EasterEgg extends CommandBase {
  // In addition to having DjKaleb, it also has access to the subsystems whose motors are being used as instruments.
  // This will cause it to override the behaviors of the default commands, and avoid issues of unsafe code
  private final DjKaleb dj;
  private int count = 30;
  private final String musicPath = "star-wars2.chrp";

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
      System.out.println(dj.playMusic());
      count--;
    } else if (count >=0) {
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
