package org.firstinspires.ftc.teamcode.pedroPathing.examples;

import com.bylazar.limelightproxy.TestLimelightServer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@TeleOp(name = "Test: Limelight AprilTags", group = "Test")
//@Disabled
public class TestLimelightAprilTag extends LinearOpMode {

    private Limelight3A limelight;
    public static final double OBELISK_HEIGHT = 19.50;
    public static final double GOAL_TAG_HEIGHT = 32.75;
    public static double limelightMountAngleDegrees = 19.83;       //Done - Adrienne ADD THIS VALUE  THIS IS THE ANGLE THE CAMERA IS MOUNTED AT.  i THINK YOU WERE ABOUT 15 DEGREES, BUT DOUBLE CHECK
    public static double limelightLensHeightInches = 9.50;        //Done - Adrienne ADD THIS VALUE  DISTANCE FROM CENTER OF LIMELIGHT LENS TO THE FLOOR
    private static boolean limelightInverted = true;    //current camera mounted upside down

    @Override
    public void runOpMode() {
        telemetry.addLine("Initializing Limelight...");
        telemetry.update();

        // Make sure the device name here matches your Driver Station config
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Select the AprilTag pipeline (0 is common, change if yours is different)
        limelight.pipelineSwitch(0);

        // Start the data polling thread
        limelight.start();

        telemetry.addLine("Ready. Press PLAY and show an AprilTag.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {

                // All detected AprilTags
                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

                if (!tags.isEmpty()) {
                    // Look at the first tag detected
                    LLResultTypes.FiducialResult tag = tags.get(0);

                    double tagDistance = getDistanceToTarget(result, OBELISK_HEIGHT);

                    telemetry.addData("Tag ID", tag.getFiducialId());
                    telemetry.addData("tx (deg)", tag.getTargetXDegrees());
                    telemetry.addData("ty (deg)", tag.getTargetYDegrees());
                    telemetry.addData("Result ty (deg)", result.getTy());
                    telemetry.addData("Result tx (deg)", result.getTx());
                    telemetry.addData("Exact distance from camera to tag", tagDistance);
                    telemetry.addData("Angle to Camera (deg)", tag.getTargetXDegrees());

                } else {
                    telemetry.addLine("No tags in view");
                }

            } else {
                telemetry.addLine("No valid Limelight result");
            }

            telemetry.update();
        }

        limelight.stop();
    }

    public double getDistanceToTarget(LLResult result, double goalHeightInches) {

            double cameraMountAngle = limelightMountAngleDegrees;

            if (limelightInverted) {
                cameraMountAngle = -cameraMountAngle;
            }

            double targetOffsetAngle_Vertical = result.getTy();

            double angleToGoalDegrees = cameraMountAngle + targetOffsetAngle_Vertical;
            double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180);

            //calculate distance to target
            double distanceToTargetInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);

            if (limelightInverted) {
                return (-distanceToTargetInches);
            } else {
                return distanceToTargetInches;
            }
    }
}

