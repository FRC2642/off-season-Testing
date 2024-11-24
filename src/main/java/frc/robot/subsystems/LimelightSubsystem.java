// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LimelightSubsystem extends SubsystemBase {

  private  String NetworkTableName;
  

  public enum DetectionType {
    NOTE(1),
    FIDUCIAL(0),
    FIDUCIAL_ZOOM(1),
    NONE(-1);

    public final int pipeline;

    private DetectionType(int pipeline) {
      this.pipeline = pipeline;
    }
  }

  private DetectionType detectionType = DetectionType.FIDUCIAL;

  public void setDetectionType(DetectionType type) {
    if (detectionType != type) {
      detectionType = type;
    }

  }

  public DetectionType getDetectionType() {
    return detectionType;
  }

  /*
   * Enum which states the most recent limelight detection error, determined in
   * the update method.
   */





  // private double x;
  // private double y;
  // private double area;
  private NetworkTable limelightTable;

  private void initialize() {
    limelightTable = NetworkTableInstance.getDefault().getTable(NetworkTableName);
  }

  public boolean isInitialized() {
    return limelightTable != null;
  }

  // private final DataStreamFilter filterX = new DataStreamFilter(10, 2d);
  // private final DataStreamFilter filterY = new DataStreamFilter(10, 2d);

  public double x;
  public double y;
  public double a;
  public boolean detectTag;
  public double distance;
  public final double targetHeight = 95;
  public final double cameraHeight = 70;
  public final double cameraAngle = 25;
  public final double DegreesToRadians=(180/Math.PI);
  public NetworkTableEntry tv = null;


  private void reset() {
    x = 0;
    y = 0;
    a = -1;
  }





    




  public void update2DMeasurements() {
    a = limelightTable.getEntry("ta").getDouble(0);
    x = limelightTable.getEntry("tx").getDouble(0);
    y = limelightTable.getEntry("ty").getDouble(0);
    NetworkTableEntry tv = limelightTable.getEntry("tv");
    if( tv != null){
      SmartDashboard.putNumber("Detected?:",0);
    }
    detectTag = tv.getDouble(0.0) == 1.0;
    if( detectTag){
      SmartDashboard.putNumber("Detected?:", 1);
    }

    distance = (targetHeight - cameraHeight) / Math.tan(y + (cameraAngle*DegreesToRadians));
    SmartDashboard.putNumber("Area:",a);
    SmartDashboard.putNumber("X:", x);
    SmartDashboard.putNumber("Y:", x);
    SmartDashboard.putNumber("distance:", distance);
  }

}