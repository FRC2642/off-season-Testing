// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.subsystems.Limelight;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

@SuppressWarnings("unused")
public class AutoMeasure extends SubsystemBase {
  private final Limelight limelight = new Limelight();
  /** Creates a new AutoMeasure. */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
