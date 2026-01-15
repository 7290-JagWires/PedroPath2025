package org.firstinspires.ftc.teamcode.TeleOp.Sensors;

import android.annotation.SuppressLint;
import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

/**
 * A wrapper class for managing an AprilTag detection pipeline using a webcam.
 * This class simplifies the initialization of the AprilTag processor and the Vision Portal,
 * and provides utility methods to access detection data.
 *
 * <p>Usage:</p>
 * <pre>{@code
 * // In your OpMode
 * AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
 *
 * // During initialization (e.g., in runOpMode() before waitForStart())
 * aprilTagWebcam.init(hardwareMap, telemetry);
 *
 * // In the main loop
 * while (opModeIsActive()) {
 *     // Update the list of detected tags
 *     aprilTagWebcam.update();
 *
 *     // Get all detected tags
 *     List<AprilTagDetection> detections = aprilTagWebcam.getDetectedTags();
 *
 *     // Iterate through detections and display telemetry
 *     for (AprilTagDetection detection : detections) {
 *         aprilTagWebcam.displayDetectionTelemetry(detection);
 *     }
 *     telemetry.update();
 *
 *     // Or, get a specific tag by its ID
 *     AprilTagDetection specificTag = aprilTagWebcam.getTagBySpecificId(5);
 *     if (specificTag != null) {
 *         // Do something with the specific tag
 *     }
 * }
 *
 * // At the end of the OpMode, to release resources
 * aprilTagWebcam.stop();
 * }</pre>
 */
public class TeleOpAprilTagWebcam {
    public AprilTagProcessor aprilTagProcessor;
    public VisionPortal visionPortal;
    public List<AprilTagDetection> detectedTags = new ArrayList<>();
    private Telemetry telemetry;

    public void init(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hwMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTagProcessor);

        visionPortal = builder.build();
    }

    public void update() {
        detectedTags = aprilTagProcessor.getDetections();
    }

    public List<AprilTagDetection> getDetectedTags() {
        return detectedTags;
    }

    @SuppressLint("DefaultLocale")
    public void displayDetectionTelemetry(AprilTagDetection tag) {
        if (tag == null) {
            telemetry.addLine(String.format("\nALIGN ROBOT CORRECTLY!!!"));
            telemetry.addLine(String.format("\nALIGN ROBOT CORRECTLY!!!"));
            telemetry.addLine(String.format("\nALIGN ROBOT CORRECTLY!!!"));
            return;}

        if (tag.metadata != null) {
            telemetry.addLine(String.format("\nROBOT CORRECTLY AIMED AT (ID %d) %s", tag.id, tag.metadata.name));
            telemetry.addLine(String.format("\n==== (ID %d) %s", tag.id, tag.metadata.name));
            telemetry.addLine(String.format("Distance to Target %6.1f (inch)", tag.ftcPose.range));
            telemetry.addLine(String.format("Angle at Target %6.1f (inch)", tag.ftcPose.bearing));
        } else {
            telemetry.addLine(String.format("\n==== (ID %d) Unknown", tag.id));
            telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", tag.center.x, tag.center.y));
        }
    }   // end for() loop

    public AprilTagDetection getTagBySpecificId(int id) {
        for (AprilTagDetection tag : detectedTags) {
            if (tag.id == id) {
                return tag;
            }
        }
        return null;
    }

    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

}
