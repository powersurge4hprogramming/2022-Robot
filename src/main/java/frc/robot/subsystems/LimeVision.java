// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeVision extends SubsystemBase {

  private final NetworkTable limelightTable;

  private final NetworkTableEntry tx;
  private final NetworkTableEntry ty;
  private final NetworkTableEntry ta;



  /** Creates a new LimeVision. */
  public LimeVision(NetworkTable limelightTable) {
    this.limelightTable = limelightTable;
    tx = limelightTable.getEntry("tx");
    ty = limelightTable.getEntry("ty");
    ta = limelightTable.getEntry("ta");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getXOffsetAngle(){
    double x = tx.getDouble(0.0);
    return x;
  }
  public double getYOffsetAngle(){
    double y = ty.getDouble(0.0);
    return y;
  }
  public double getPercentArea(){
    double area = ta.getDouble(0.0);
    return area;
  }
  
}
