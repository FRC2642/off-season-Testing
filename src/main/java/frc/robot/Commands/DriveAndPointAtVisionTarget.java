// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveAndPointAtVisionTarget extends Command {
  
  private final PhotonVisionSubsystem vision;
  private final SwerveSubsystem swerve;
  private final XboxController control;
  private int AprilTagID;
  
public DriveAndPointAtVisionTarget(SwerveSubsystem swerve, PhotonVisionSubsystem vision, int AprilTagID, XboxController control) {
    this.vision = vision;
    this.swerve = swerve;
    this.control = control;
    this.AprilTagID = AprilTagID;
    addRequirements(swerve, vision);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Rotation2d targetVector = vision.getAprilTagYaw(AprilTagID);
    double xSpeed = -control.getLeftY();
    double ySpeed = -control.getLeftX();

    swerve.driveFacingAngle(targetVector, xSpeed, ySpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !control.getAButton();
  }
}
