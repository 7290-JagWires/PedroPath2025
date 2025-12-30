package org.firstinspires.ftc.teamcode.pedroPathing.examples;

// Import your new class
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.LimelightCamera;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@TeleOp(name = "Test: Limelight AprilTags", group = "Test")
//@Disabled
public class TestLimelightAprilTag extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry.addLine("Initializing Limelight...");
        telemetry.update();

        // Make sure the device name here matches your Driver Station config
        LimelightCamera limelight = new LimelightCamera(hardwareMap, "limelight");

        // Select the AprilTag pipeline (0 for tags 21, 22 & 23)
        limelight.setPipeline(0);

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

                    // distance function we created to
                    double tagDistance = limelight.getDistanceToTarget(result, LimelightCamera.OBELISK_HEIGHT);

                    telemetry.addData("Tag ID", limelight.getTagId());
                    telemetry.addData("Angle from camera to center of Target (Ty)", result.getTy());
                    telemetry.addData("Angle to Camera (deg) (Tx)", result.getTx());
                    telemetry.addData("Exact distance from camera to tag", tagDistance);

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
}

