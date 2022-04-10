// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.data_structs;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

public class LimeVision {

  private static final NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

  private static final NetworkTableEntry tx = limelightTable.getEntry("tx");
  private static final NetworkTableEntry ty = limelightTable.getEntry("ty");
  private static final NetworkTableEntry ta = limelightTable.getEntry("ta");
  private static final NetworkTableEntry camMode = limelightTable.getEntry("camMode");
  private static final NetworkTableEntry ledMode = limelightTable.getEntry("ledMode");
  private static final NetworkTableEntry tv = limelightTable.getEntry("tv");

  public static double getTv() {
    return tv.getDouble(0.0);
  }

  public static double getledMode() {
    return ledMode.getDouble(-1);
  }

  /**
   * Sets limelight’s LED state
   * 0 use the LED Mode set in the current pipeline
   * 1 force off
   * 2 force blink
   * 3 force on
   */
  public static boolean setledMode(Number mode) {
    return ledMode.setNumber(mode);
  }

  public static double getcamMode() {
    return camMode.getDouble(-1);
  }

  /**
   * Sets limelight’s operation mode
   * 0 Vision processor
   * 1 Driver Camera (Increases exposure, disables vision processing)
   */
  public static boolean setcamMode(Number mode) {
    return camMode.setNumber(mode);
  }

  public static double getXOffsetAngle() {
    double x = tx.getDouble(0.0);
    return x;
  }

  public static double getYOffsetAngle() {
    double y = ty.getDouble(0.0);
    return y;
  }

  public static double getPercentArea() {
    double area = ta.getDouble(0.0);
    return area;
  }

  public static boolean getTracking() {
    return (tv.getDouble(0.0) == 1.0);
  }

  public static double targetDistance() {
    // All measurements in METERS and RADIANS - see
    // https://docs.limelightvision.io/en/latest/cs_estimating_distance.html

    // Limelight height off of ground, DONE
    double h1 = Constants.VisionConstants.DIST_LIME_HEIGHT;

    // Upper Hub height off of ground, DONE
    double h2 = Constants.VisionConstants.DIST_HUB_HEIGHT;

    // Limelight mounted angle from horizontal, DONE
    double a1 = Math.toRadians(Constants.VisionConstants.DIST_LIME_MOUNT_ANGLE);

    // Limelight angle from mounted to target
    double a2 = Math.toRadians(getYOffsetAngle());

    double d = (h2 - h1) / Math.tan(a1 + a2);

    return d;
  }

}
