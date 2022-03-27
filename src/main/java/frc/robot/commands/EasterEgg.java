// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.music.Orchestra;

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
  /** Creates a new EasterEgg. */
  public EasterEgg(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.loadMusic();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (count == 0) {
      shooter.playMusic();
    } else if (count > 0) {
      count--;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.cancelMusic();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
