// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.subsystems.LimelightSubsystem;

public class RobotContainer {
  private final LimelightSubsystem limelight = new LimelightSubsystem();

  public RobotContainer() {
    limelight.limelightDiagnostic();
    limelight.update2DMeasurements();
    String limelightError=limelight.detectionError.name();
    System.out.println(limelightError);
  }
}