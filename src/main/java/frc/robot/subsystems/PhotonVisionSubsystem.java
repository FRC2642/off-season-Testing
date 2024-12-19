// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVisionSubsystem extends SubsystemBase {

  private static AHRS gyro;

  PhotonCamera frontCamera;    
  PhotonCamera backCamera;
  PhotonCamera leftCamera;
  PhotonCamera rightCamera;
  PhotonCamera[] cameras = { frontCamera, backCamera, leftCamera, rightCamera }; //possible fault

  private static double yawOffsetDegrees = 0;

  PhotonCamera activeCamera;
  double targetYaw = 0.0;
  boolean targetVisible;

  public PhotonVisionSubsystem() {

    gyro = new AHRS();
    frontCamera = new PhotonCamera("frontCamera");
    backCamera = new PhotonCamera("backCamera");
    leftCamera = new PhotonCamera("leftCamera");
    rightCamera = new PhotonCamera("rightCamera");
  }

  public double getGyroYaw() {
    return gyro.getYaw() + yawOffsetDegrees;
    //Get the yaw of the robot (field relative)
    //Test positive vs negative
  }

  public static void resetGyro(double yawDegrees) {
    gyro.reset();
    yawOffsetDegrees = yawDegrees;
  }

  //get vector of an april tag (field centric)
  public Rotation2d getAprilTagYaw(int tag){
    activeCamera = null;
    targetYaw = 0.0;
    targetVisible = false;

    for (PhotonCamera camera : cameras) {
      var results = camera.getAllUnreadResults();
      if (!results.isEmpty()) {
          var result = results.get(results.size() - 1);
          if (result.hasTargets()) {
              for (var target : result.getTargets()) {
                  if (target.getFiducialId() == tag) { // Check for specific target
                      targetYaw = target.getYaw();
                      targetVisible = true;
                      activeCamera = camera; // Set the active camera
                      break; // Exit inner loop if target found
                  }
              }
          }
      }
      if (targetVisible) break;
    }
    //Adjust for different camera angles
    if(activeCamera == rightCamera){
      targetYaw =+ 90;
    }
    if(activeCamera == backCamera){
      targetYaw =+ 180;
    }
    if(activeCamera == leftCamera){
      targetYaw =+ 270;
    }
    //account for robot rotation
    double robotYaw = gyro.getYaw() + yawOffsetDegrees;
    targetYaw += robotYaw;

    //convert target yaw to target vector (on the unit circle)
    double angleRadians = Math.toRadians(targetYaw);
    Rotation2d targetVector = new Rotation2d(Math.cos(angleRadians), Math.sin(angleRadians));

    return targetVector;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
