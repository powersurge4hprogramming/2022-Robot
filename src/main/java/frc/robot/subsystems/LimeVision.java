// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeVision extends SubsystemBase {

  private final NetworkTableEntry tx;
  private final NetworkTableEntry ty;
  private final NetworkTableEntry ta;
  private final NetworkTableEntry camMode;
  private final NetworkTableEntry ledMode;
  private final NetworkTableEntry tv;

  /** Creates a new LimeVision. */
  public LimeVision(NetworkTable limelightTable) {
    tx = limelightTable.getEntry("tx");
    ty = limelightTable.getEntry("ty");
    ta = limelightTable.getEntry("ta");
    tv = limelightTable.getEntry("tv");

    ledMode = limelightTable.getEntry("ledMode");
    camMode = limelightTable.getEntry("camMode");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putBoolean("Vision On", (tv.getDouble(0.0) == 1.0));

    if (getTracking()) {
      // All measurements in METERS and RADIANS -  see https://docs.limelightvision.io/en/latest/cs_estimating_distance.html

      // Limelight height off of ground, TODO
      double h1 = 1.016;

      // Upper Hub height off of ground, DONE
      double h2 = 2.64;

      // Limelight mounted angle from horizontal, TODO
      double a1 = Math.toRadians(0);

      // Limelight angle from mounted to target
      double a2 = Math.toRadians(getYOffsetAngle());

      double d = (h2-h1) / Math.tan(a1+a2);

      SmartDashboard.putNumber("Target Distance", d);
    } else {
      SmartDashboard.putNumber("Target Distance", 0.0);
    }
  }

  public double getledMode() {
    return ledMode.getDouble(-1);
  }

  /**
   * Sets limelight’s LED state
   * 0 use the LED Mode set in the current pipeline
   * 1 force off
   * 2 force blink
   * 3 force on
   */
  public boolean setledMode(Number mode) {
    return ledMode.setNumber(mode);
  }

  public double getcamMode() {
    return camMode.getDouble(-1);
  }

  /**
   * Sets limelight’s operation mode
   * 0 Vision processor
   * 1 Driver Camera (Increases exposure, disables vision processing)
   */
  public boolean setcamMode(Number mode) {
    return camMode.setNumber(mode);
  }

  public double getXOffsetAngle() {
    double x = tx.getDouble(0.0);
    return x;
  }

  public double getYOffsetAngle() {
    double y = ty.getDouble(0.0);
    return y;
  }

  public double getPercentArea() {
    double area = ta.getDouble(0.0);
    return area;
  }

  public boolean getTracking() {
    return (tv.getDouble(0.0) == 1.0);
  }

}
