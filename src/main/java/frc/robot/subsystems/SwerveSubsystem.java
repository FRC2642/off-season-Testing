// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Telemetry;
import frc.robot.generated.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;

public class SwerveSubsystem extends SubsystemBase {

  //DriveTrain
    private CommandSwerveDrivetrain drivetrain;
  //Constants (currently using half speed)
    final double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps;
    final double MaxAngularRate = 1.5 * Math.PI;
    final double HalfSpeed = MaxSpeed / 2;
    final double HalfAngularRate = MaxAngularRate / 2;
  //SwerveRequests
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1)
        .withRotationalDeadband(MaxAngularRate * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake stop = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.FieldCentricFacingAngle driveFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
        .withDeadband(MaxSpeed * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  //Telemetry (need to figure out)
    private final Telemetry logger = new Telemetry(HalfSpeed);                //Speed

  public SwerveSubsystem() {
   drivetrain = TunerConstants.DriveTrain;
  }

  public void Drive(double xSpeed, double ySpeed, double rotation){
    drivetrain.applyRequest(() -> drive
              .withVelocityX(xSpeed * HalfSpeed)                              //Speed
              .withVelocityY(ySpeed * HalfSpeed)                              //Speed
              .withRotationalRate(rotation * HalfAngularRate));               //Angular Rate
  }
  public void Stop(){
    drivetrain.applyRequest(() -> stop);
  }

  public void Point(Rotation2d pointVector){
    drivetrain.applyRequest(() -> point.withModuleDirection(pointVector));
  }

  public void driveFacingAngle(Rotation2d pointVector, double xSpeed, double ySpeed){
    drivetrain.applyRequest(()-> driveFacingAngle
              .withVelocityX(xSpeed * HalfSpeed)                              //Speed
              .withVelocityY(ySpeed * HalfSpeed)                              //Speed
              .withTargetDirection(pointVector));
  }

  public void SetFieldCentric(){
    drivetrain.runOnce(() -> drivetrain.seedFieldRelative());
  }

  public void OtherUnknownUse(){
    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
