package org.firstinspires.ftc.teamcode.TeleOp.OpModes;

// Import your new class

import static org.firstinspires.ftc.teamcode.TeleOp.Sensors.TelopLimelightCamera.GOAL_TAG_HEIGHT;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TeleOp.Sensors.TelopLimelightCamera;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.LimelightCamera;

import java.util.List;

@TeleOp(name = "Test: TELOP CAMERA", group = "Test")
@Disabled
public class TestLimelightAprilTag extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry.addLine("Initializing Limelight...");
        telemetry.update();

        // Make sure the device name here matches your Driver Station config
        TelopLimelightCamera limelight = new TelopLimelightCamera(hardwareMap, "limelight");

        // Select the AprilTag pipeline (0 for tags 21, 22 & 23)
        limelight.setPipeline(1);

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
                    double tagDistance = limelight.getDistanceToTarget(GOAL_TAG_HEIGHT);

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

