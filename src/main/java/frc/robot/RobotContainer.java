// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;

public class RobotContainer {

    // Maximum speeds and angular rates
    private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // Desired top speed
    private double MaxAngularRate = 1.5 * Math.PI; // Max angular velocity (3/4 rotation per second)

    private double HalfSpeed = MaxSpeed / 2;
    private double HalfAngularRate = MaxAngularRate / 2;

    // Controller and drivetrain initialization
    private final CommandXboxController joystick = new CommandXboxController(0); // Joystick
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // Drivetrain
    private final LimelightSubsystem limelight= new LimelightSubsystem();

    // Swerve requests
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1) // 10% deadband for speed
        .withRotationalDeadband(MaxAngularRate * 0.1) // 10% deadband for rotation
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Field-centric driving (open loop)


    // Telemetry logger
    private final Telemetry logger = new Telemetry(HalfSpeed);

    // Configure control bindings
    private void configureBindings() {
    }

    // Constructor
    public RobotContainer() {
        configureBindings();
        while(true){
        limelight.limelightDiagnostic();//Checks for error and prints to console
        limelight.update2DMeasurements();//Gives a ton of info about april tags
    }
  }

    // Default autonomous command
    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
