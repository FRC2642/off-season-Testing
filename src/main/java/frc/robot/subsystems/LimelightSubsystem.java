// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem to manage the Limelight vision system.
 * Handles detection types, measurements, and diagnostics.
 */
public class LimelightSubsystem extends SubsystemBase {

    // Name of the NetworkTable used by the Limelight
    private String NetworkTableName;

    // Detection type enum, used to set Limelight pipelines
    public enum DetectionType {
        NOTE(1), 
        FIDUCIAL(0), 
        FIDUCIAL_ZOOM(1), 
        NONE(-1);

        public final int pipeline;

        private DetectionType(int pipeline) {
            this.pipeline = pipeline;
        }
    }

    // Current detection type (default is FIDUCIAL)
    private DetectionType detectionType = DetectionType.FIDUCIAL;

    /**
     * Sets the Limelight detection type and updates the pipeline if changed.
     *
     * @param type New detection type
     */
    public void setDetectionType(DetectionType type) {
        if (detectionType != type) {
            detectionType = type;
        }
    }

    /**
     * Gets the current detection type.
     *
     * @return Current DetectionType
     */
    public DetectionType getDetectionType() {
        return detectionType;
    }

    // Enum to represent possible Limelight detection errors
    public enum DetectionError {
        NOT_STARTED, // Subsystem not initialized
        NO_DETECTIONS, // No tags or targets detected
        TV_NULL, // Limelight "tv" entry is null
        NO_PIPELINES, // No pipelines available
        UNDETERMINED, // Unable to determine state
        NO_BOTPOSE, // No bot pose data
        SUCCESS, // Detection successful
        UNKNOWN // Unknown error
    }

    // Limelight NetworkTable and other related data
    private NetworkTable limelightTable;

    // Measurements and diagnostics data
    public double x; // Horizontal offset
    public double y; // Vertical offset
    public double a; // Area of detected target
    public boolean detectTag; // Whether a tag is detected
    public double distance; // Distance to the target

    // Camera and target parameters
    public final double targetHeight = 95;
    public final double cameraHeight = 70;
    public final double cameraAngle = 25;
    public final double DegreesToRadians = (180 / Math.PI);

    // Current detection error state
    public DetectionError detectionError = DetectionError.NOT_STARTED;

    /**
     * Initializes the Limelight NetworkTable.
     */
    private void initialize() {
        limelightTable = NetworkTableInstance.getDefault().getTable(NetworkTableName);
    }

    /**
     * Checks if the Limelight subsystem has been initialized.
     *
     * @return True if initialized, false otherwise
     */
    public boolean isInitialized() {
        initialize();
        return limelightTable != null;
    }

    /**
     * Resets Limelight measurements.
     */
    private void reset() {
        x = 0;
        y = 0;
        a = -1;
    }

    /**
     * Diagnoses the Limelight subsystem, ensuring all components are operational.
     *
     * @return Current detection error state
     */
    public DetectionError limelightDiagnostic() {
        reset();

        if (!isInitialized()) {
            return DetectionError.NOT_STARTED;
        }

        NetworkTableEntry pipeline = limelightTable.getEntry("pipeline");
        if (pipeline == null) {
            return DetectionError.NO_PIPELINES;
        }
        pipeline.setDouble(detectionType != null ? detectionType.pipeline : 0); // Avoid potential NPE

        NetworkTableEntry tv = limelightTable.getEntry("tv");
        if (tv == null) {
            return DetectionError.TV_NULL;
        }

        if (!detectTag) {
            return DetectionError.NO_DETECTIONS;
        }

        DetectionError updatedError = update2DMeasurements();
        if (updatedError != DetectionError.SUCCESS) {
            return updatedError;
        }

        if (detectionType == DetectionType.FIDUCIAL) {
            // Add any specific logic for FIDUCIAL if necessary
        }

        return DetectionError.SUCCESS; // Default return for successful execution
    }

    /**
     * Gets the current detection error.
     *
     * @return Current DetectionError state
     */
    public DetectionError getDetectionError() {
        return detectionError;
    }

    /**
     * Updates the 2D measurements for detected targets and calculates the distance.
     *
     * @return DetectionError.SUCCESS if successful
     */
    public DetectionError update2DMeasurements() {
        a = limelightTable.getEntry("ta").getDouble(0); // Target area
        x = limelightTable.getEntry("tx").getDouble(0); // Horizontal offset
        y = limelightTable.getEntry("ty").getDouble(0); // Vertical offset

        NetworkTableEntry tv = limelightTable.getEntry("tv");
        detectTag = tv.getDouble(0.0) == 1.0; // Target detection status

        // Publish detection status to SmartDashboard
        SmartDashboard.putNumber("Detected?:", detectTag ? 1 : 0);

        // Calculate distance using geometry
        distance = (targetHeight - cameraHeight) / Math.tan(y + (cameraAngle * DegreesToRadians));

        // Publish measurements to SmartDashboard
        SmartDashboard.putNumber("Area:", a);
        SmartDashboard.putNumber("X:", x);
        SmartDashboard.putNumber("Y:", y);
        SmartDashboard.putNumber("Distance:", distance);

        return DetectionError.SUCCESS;
    }
}
