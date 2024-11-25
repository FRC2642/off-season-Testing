// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  private double HalfSpeed = MaxSpeed / 2;
  private double HalfAngularRate = MaxAngularRate / 2;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final LimelightSubsystem limelight = new LimelightSubsystem();


  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(HalfSpeed);

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * HalfSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * HalfSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * HalfAngularRate) // Drive counterclockwise with negative X (left)
        ));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    joystick.rightTrigger().onTrue(drivetrain.runOnce(() -> LimelightMode()));
    joystick.leftTrigger().onTrue(drivetrain.runOnce(()-> DiagnosticMode()));
    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    configureBindings();
  }
  public Command LimelightMode(){
      while(limelight.detectTag!=true){
        drivetrain.applyRequest(() -> drive.withRotationalRate(HalfAngularRate));
        limelight.update2DMeasurements();

      }
      if(limelight.detectTag==true && limelight.x>=0){
        limelight.update2DMeasurements();
        drivetrain.applyRequest(() ->drive.withRotationalRate(-HalfAngularRate));
        if(limelight.x<=0){
          limelight.update2DMeasurements();
          drivetrain.applyRequest(() ->drive.withRotationalRate(0));
          SmartDashboard.putNumber("Target Locked",1);
        }
      }
      if(limelight.detectTag==true && limelight.x<=(0)){
        limelight.update2DMeasurements();
        drivetrain.applyRequest(() ->drive.withRotationalRate(HalfAngularRate));
        if(limelight.x>=0){
          limelight.update2DMeasurements();
          drivetrain.applyRequest(() ->drive.withRotationalRate(0));
          SmartDashboard.putNumber("Target Locked:", 1);
        }
      }
      if(limelight.detectTag==true && limelight.x<=(3) && limelight.x>=(-3)){
        limelight.update2DMeasurements();
        drivetrain.applyRequest(() ->drive.withVelocityX(HalfSpeed));
        if(limelight.distance<=50){
          limelight.update2DMeasurements();
          drivetrain.applyRequest(() ->drive.withVelocityX(0));
        }
      }
            return null;
  }
  public Command DiagnosticMode(){
    limelight.limelightDiagnostic();
    limelight.update2DMeasurements();
    String limelightError = limelight.detectionError.name(); 
    SmartDashboard.putNumber(limelightError,1);
    for( int i=0; i<4; i++){
    drivetrain.applyRequest(()->point.withModuleDirection(new Rotation2d(1.57)));
    limelight.limelightDiagnostic();
    limelight.update2DMeasurements();
    }
    drivetrain.applyRequest(()-> drive.withRotationalRate(HalfAngularRate/1.5));
    for( int i=0; i<8; i++){
      limelight.limelightDiagnostic();
      limelight.update2DMeasurements();
    }



    return(null);
  }
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}