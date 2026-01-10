package org.firstinspires.ftc.teamcode.pedroPathing.Hardware;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * A wrapper class for the Limelight camera to simplify its use in OpModes.
 * This class handles initialization, configuration, and provides helper methods
 * for calculating distance to AprilTags.
 */
public class LimelightCamera {

    private Limelight3A limelight;

    // Camera mounting parameters - these are specific to your robot's physical setup
    private static final double LIMELIGHT_MOUNT_ANGLE_DEGREES = 0; //OLD Mount19.83;
    private static final double LIMELIGHT_LENS_HEIGHT_INCHES = 9.25;
    private static final boolean IS_LIMELIGHT_INVERTED = true;
    private static final int GPP = 21;
    private static final int PGP = 22;
    private static final int PPG = 23;


    // Constants for known target heights
    public static final double OBELISK_HEIGHT = 19.50;
    public static final double GOAL_TAG_HEIGHT = 29.5;


    /**
     * Constructor to initialize the Limelight camera.
     * @param hardwareMap The HardwareMap from your OpMode.
     * @param deviceName The name of the Limelight configured in the Driver Station.
     */
    public LimelightCamera(HardwareMap hardwareMap, String deviceName) {
        limelight = hardwareMap.get(Limelight3A.class, deviceName);
    }

    /**
     * Starts the background data polling thread for the Limelight.
     * Must be called after initialization.
     */
    public void start() {
        if (limelight != null) {
            limelight.start();
        }
    }

    /**
     * Stops the background data polling thread.
     * Should be called at the end of an OpMode.
     */
    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }

    /**
     * Switches the Limelight to a specific pipeline.
     * @param pipelineIndex The index of the pipeline to activate (0-9).
     */
    public void setPipeline(int pipelineIndex) {
        if (limelight != null) {
            limelight.pipelineSwitch(pipelineIndex);
        }
    }

    /**
     * Gets the most recent result frame from the Limelight.
     * @return The LLResult object, or null if no result is available.
     */
    public LLResult getLatestResult() {
        if (limelight != null) {
            return limelight.getLatestResult();
        }
        return null;
    }

    // ... inside the LimelightCamera class, after getLatestResult()

    /**
     * Gets the ID of the first visible AprilTag.
     * This method fetches the latest result from the camera.
     *
     * @return The fiducial ID of the first detected tag, or -1 if no tag is visible.
     */
    public int getTagId() {
        LLResult result = getLatestResult();

        // Check if the result is valid and contains any AprilTag data
        if (result != null && result.isValid() && !result.getFiducialResults().isEmpty()) {
            // Return the ID of the first tag in the list
            return result.getFiducialResults().get(0).getFiducialId();
        }

        // If no valid tag is seen, return -1 as an indicator
        return -1;
    }

    /**
     * Processes the latest Limelight result to find the primary AprilTag and returns its data.
     * This combines getting the result, checking for validity, and calculating distance.
     *
     * @return A {@link TagData} object containing the tag's info, or {@code null} if no tag is found.
     */
    public TagData getAprilTagData() {
        LLResult result = getLatestResult();

        // Check if the result is valid and has detected any fiducial tags
        if (result != null && result.isValid() && !result.getFiducialResults().isEmpty()) {

            // Look at the first tag detected
            LLResultTypes.FiducialResult tag = result.getFiducialResults().get(0);

            // Calculate the distance to the tag
            double distance = getDistanceToTarget(result, GOAL_TAG_HEIGHT);

            // Create and return a new TagData object with all the information
            return new TagData(tag.getFiducialId(), distance, result.getTx(), result.getTy());
        }

        // If no valid tag is found, return null
        return null;
    }
    /**
     * Calculates the horizontal distance to a target based on its height and the camera's vertical angle.
     *
     * @param result         The LLResult containing the target data.
     * @param goalHeightInches The known height of the AprilTag target from the floor.
     * @return The calculated distance to the target in inches.
     */
    public double getDistanceToTarget(LLResult result, double goalHeightInches) {
        if (result == null || !result.isValid()) {
            return 0.0; // Return 0 or an invalid value if no target is seen
        }

        double cameraMountAngle = LIMELIGHT_MOUNT_ANGLE_DEGREES;

        // Account for the camera being mounted upside down
        if (IS_LIMELIGHT_INVERTED) {
            cameraMountAngle = -cameraMountAngle;
        }

        // 'ty' is the vertical angle from the crosshair to the target
        double targetOffsetAngle_Vertical = result.getTy();

        // Calculate the total angle from the horizontal plane to the target
        double angleToGoalDegrees = cameraMountAngle + targetOffsetAngle_Vertical;
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);

        // Basic trigonometry to find the distance
        double distanceToTargetInches = (goalHeightInches - LIMELIGHT_LENS_HEIGHT_INCHES) / Math.tan(angleToGoalRadians);

        // If the camera is inverted, the distance calculation might need to be negated depending on axis conventions
        if (IS_LIMELIGHT_INVERTED) {
            return -distanceToTargetInches;
        } else {
            return distanceToTargetInches;
        }
    }
    /**
     * A simple data class to hold the processed information from a detected AprilTag.
     * This is a "public static nested class", which allows it to be public
     * while still living inside the LimelightCamera.java file.
     */
    public static class TagData {
        public final int id;
        public final double distance;
        public final double tx; // Horizontal angle
        public final double ty; // Vertical angle

        public TagData(int id, double distance, double tx, double ty) {
            this.id = id;
            this.distance = distance;
            this.tx = tx;
            this.ty = ty;
        }
    }
}


