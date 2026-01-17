package org.firstinspires.ftc.teamcode.TeleOp.Sensors;

import static org.firstinspires.ftc.teamcode.TeleOp.Hardware.TeleOpShooter.SHOOTER_DEFENSE_GOAL;
import static org.firstinspires.ftc.teamcode.TeleOp.Hardware.TeleOpShooter.SHOOTER_VELOCITY_BACK_GOAL;
import static org.firstinspires.ftc.teamcode.TeleOp.Hardware.TeleOpShooter.SHOOTER_VELOCITY_FRONT_GOAL;
import static org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Shooter.SHOOT_DEFENSE_VELOCITY;
import static org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Shooter.SHOOT_GOAL_VELOCITY;
import static org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Shooter.SHOOT_TRIANGLE_VELOCITY;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.TeleOp.Hardware.TeleOpRobot;

/**
 * A wrapper class for the Limelight camera to simplify its use in OpModes.
 * This class handles initialization, configuration, and provides helper methods
 * for calculating distance to AprilTags.
 */
public class TelopLimelightCamera {

    private Limelight3A limelight;

    // Camera mounting parameters - these are specific to your robot's physical setup
    private static final double LIMELIGHT_MOUNT_ANGLE_DEGREES = 0; //OLD Mount19.83;
    private static final double LIMELIGHT_LENS_HEIGHT_INCHES = 9.25;
    private static final boolean IS_LIMELIGHT_INVERTED = true;
    public LLResult result;
    private LLResultTypes.FiducialResult tag;
    public double tagDistance = 0;
    public int tagID = 0;
    public double tagXAngle = 0;
    public double tagYAngle = 0;

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
    public TelopLimelightCamera(HardwareMap hardwareMap, String deviceName) {
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
     */
    public void getAprilTagData() {
        result = getLatestResult();

        // Check if the result is valid and has detected any fiducial tags
        if (result != null && result.isValid() && !result.getFiducialResults().isEmpty()) {

            // Look at the first tag detected
            tag = result.getFiducialResults().get(0);

            // Calculate the distance to the tag
            tagDistance = getDistanceToTarget(GOAL_TAG_HEIGHT);
            tagID = tag.getFiducialId();
            tagXAngle = result.getTx();
            tagYAngle = result.getTy();
        }
    }

    /**
     * Calculates the horizontal distance to a target based on its height and the camera's vertical angle.
     *
     * @param goalHeightInches The known height of the AprilTag target from the floor.
     * @return The calculated distance to the target in inches.
     */
    public double getDistanceToTarget(double goalHeightInches) {

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

    public void updateShooterVelocityBasedOnDistance(TeleOpRobot robot) {

        if (result != null) {
            // If a tag is visible, adjust velocity based on its distance
            if (tagDistance > 70  && tagDistance < 115) {
                robot.shooter.setExplicitVelocity(SHOOTER_DEFENSE_GOAL);
            } else if (tagDistance >= 115  && tagDistance < 135) {
                robot.shooter.setExplicitVelocity(SHOOTER_VELOCITY_BACK_GOAL);
            } else if (tagDistance >= 135) {
                robot.shooter.setExplicitVelocity(2750);
            }else {
                // If the tag is at a medium distance, use the defense velocity
                robot.shooter.setExplicitVelocity(SHOOTER_VELOCITY_FRONT_GOAL);
            }
        }
        else
            robot.shooter.setExplicitVelocity(SHOOTER_VELOCITY_FRONT_GOAL);
    }
}


