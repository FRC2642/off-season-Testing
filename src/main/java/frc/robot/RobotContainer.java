// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;



import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;

@SuppressWarnings("unused")
public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  private double HalfSpeed = MaxSpeed / 2;
  private double HalfAngularRate = MaxAngularRate / 2;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final Limelight limelight = new Limelight();


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
/**
 * Executes the Limelight Mode command.
 * This mode uses the Limelight subsystem to align the robot with a detected tag.
 * The robot rotates and moves forward based on the tag's position to achieve alignment.
 *
 * @return null (as the function executes actions directly on the drivetrain)
 */
public Command LimelightMode() {
  while(limelight.x>3 && limelight.x<-3){
  // Rotate until a tag is detected
  while (!limelight.detectTag) {
      drivetrain.applyRequest(() -> drive.withRotationalRate(HalfAngularRate));
      limelight.update2DMeasurements();
  }
  double speed = Math.copySign(Math.min(Math.abs(limelight.x) / 10.0, MaxAngularRate), limelight.x); 
  if(limelight.x>0){
  drivetrain.applyRequest(() -> drive.withRotationalRate(-speed));
  }
  if(limelight.x<0){
    drivetrain.applyRequest(() -> drive.withRotationalRate(speed));
  }
  drivetrain.applyRequest(()->brake);
}
  return null; // Command is executed directly
}

/**
* Executes the Diagnostic Mode command.
* This mode runs diagnostics on the Limelight and the drivetrain modules,
* logging relevant errors and checking drivetrain behavior.
*
* @return null (as the function executes actions directly on the drivetrain)
*/
public Command DiagnosticMode() {
  // Run initial diagnostics on the Limelight subsystem
  limelight.limelightDiagnostic();
  limelight.update2DMeasurements();
  String limelightError = limelight.detectionError.name();
  System.out.println("LimelightError="+limelightError);

  // Point all drivetrain modules in a fixed direction and run diagnostics
  for (int i = 0; i < 4; i++) {
      drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(1.57))); // Point modules in one direction
      limelight.limelightDiagnostic();
      limelight.update2DMeasurements();
  }

  // Rotate the drivetrain and continue diagnostics
  drivetrain.applyRequest(() -> drive.withRotationalRate(HalfAngularRate / 1.5));
  for (int i = 0; i < 4; i++) {
      limelight.limelightDiagnostic();
      limelight.update2DMeasurements();
      System.out.println("LimelightError="+limelightError);
  }

  // Apply brakes to the drivetrain after diagnostics are complete
  drivetrain.applyRequest(() -> brake);

  return null; // Command is executed directly
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
