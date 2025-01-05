// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveCommand extends Command {
  
  private final SwerveSubsystem swerve;
  private final XboxController control;
  
  public DriveCommand(SwerveSubsystem swerve, XboxController control) {
    this.swerve = swerve;
    this.control = control;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
  //Get controller values
    double xSpeed = -control.getLeftY();
    double ySpeed = -control.getLeftX();
    double rotation = -control.getRightX();
    Rotation2d pointVector = new Rotation2d(-control.getLeftY(), -control.getLeftX());
  //Decide Movement
    if(control.getBButton()) {
      swerve.Point(pointVector);
    }
    else{
    if(control.getAButton()){
      //Math.abs(xSpeed) <= 0.1f && Math.abs(ySpeed) <= 0.1f && Math.abs(rotation) <= 0.1f
      swerve.Stop();
    }
    else{
      swerve.Drive(xSpeed, ySpeed, rotation);
    }}

  //Other settings
    if(control.getLeftBumper()){
      swerve.SetFieldCentric();
    }
    swerve.OtherUnknownUse();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
