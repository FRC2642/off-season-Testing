// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PhotonVisionSubsystem;

public class ResetGyroCommand extends Command {
  
  private final double yaw;

  public ResetGyroCommand(double yaw) {
    this.yaw = yaw;
  }

  @Override
  public void execute() {
    PhotonVisionSubsystem.resetGyro(yaw);
  }

    @Override
  public boolean isFinished() {
    return true;
  }
}
