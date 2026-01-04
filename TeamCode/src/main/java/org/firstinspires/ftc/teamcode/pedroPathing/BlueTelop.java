package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Sensors.ColorSensor;
import org.firstinspires.ftc.teamcode.pedroPathing.Sensors.IndicatorLight;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseStorage;

@TeleOp()
public class BlueTelop extends OpMode {

    private static final int LAUNCH_DELAY_MILLISECONDS = 1000;

    // Class member variables
    private Robot robot;
    private ColorSensor colorSensor;
    private IndicatorLight indicator;

    private boolean dumpMode = false;  // toggle for RT dump-all
    private boolean prevRT = false;
    private int dumpCount = 0;

    private final ElapsedTime launchTimer = new ElapsedTime();
    private final ElapsedTime doorTimer = new ElapsedTime();

    // Homing state variables
    private enum HomingState { HOMING, COMPLETE }
    private HomingState homingState = HomingState.HOMING;
    private Follower follower;

    // Add this enum to your class member variables
    private enum ShootState {
        IDLE,        // Waiting for input
        OPENING_DOOR,  // The door is opening
        CLOSING_DOOR,  // The door is closing
        INDEXING// Advancing the spindexer
    }
    private ShootState shootState = ShootState.IDLE;

    /**
     * This method runs ONCE when the driver hits "INIT" on the Driver Station.
     * This is where you initialize all your hardware.
     */
    @Override
    public void init() {
        telemetry.addLine("Initializing...");

        robot = new Robot(this);
        colorSensor = new ColorSensor(hardwareMap, "color_sensor");
        indicator = new IndicatorLight(hardwareMap, "rgbServo");

        follower = Constants.createFollower(hardwareMap);
        Pose startingPose = PoseStorage.loadPoseFromFile(hardwareMap);
        follower.setPose(startingPose);

        // Initialize all your other robot hardware here
        // example: leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");

        telemetry.addLine("TeleOp Initialized");
        telemetry.addData("Pose Loaded From Auto", "X: %.2f, Y: %.2f, H: %.2f",
                startingPose.getX(), startingPose.getY(), startingPose.getHeading());

        telemetry.addLine("Initialization Complete. Ready for Homing.");
        telemetry.update();
    }

    /**
     * This method runs in a loop AFTER "INIT" is pressed but BEFORE "PLAY" is pressed.
     * It's perfect for tasks like auto-homing that need to happen before the match starts.
     */
    @Override
    public void init_loop() {
        if (homingState == HomingState.HOMING) {
            telemetry.addLine("Homing Spindexer...");

            robot.spindexer.setModeRunUsingEncoder();
            robot.spindexer.setPower(0.5);

            // Check if the magnet is triggered
            if (robot.spindexerMag.isTriggered()) {
                robot.spindexer.stop();
                homingState = HomingState.COMPLETE;
                telemetry.addLine("Homing Complete. Ready to Start.");
            }
        }
        telemetry.update();
    }

    /**
     * This method runs ONCE when the driver hits "PLAY".
     * Any actions that should happen right at the start of the match go here.
     */
    @Override
    public void start() {
        // You could reset timers here if needed, but it's often empty for TeleOp.
        robot.spindexer.setPower(0.9);

    }

    /**
     * This method runs in a loop continuously from the moment "PLAY" is pressed until "STOP" is pressed.
     * This is where all your driver controls and main logic will live.
     */
    @Override
    public void loop() {
        // --- Sensor Updates ---
        updateColorSensor();

        // --- Driver 1: Drivetrain ---
        handleDriveControls();

        // --- Driver 2: Mechanisms ---
        handleDumpMode();
        handleManualShoot(); // <-- NEW: This replaces the old code

        // Run the main robot update loop
        robot.update();

        // Auto-close door after indexing
        if (robot.spindexer.justIndexed) {
            robot.door.forceClose();
            robot.spindexer.justIndexed = false;
        }

        // IMPORTANT: Update the follower in every loop cycle to track position
        follower.update();

        // --- Telemetry ---
        updateTelemetry();
    }

    /**
     * This method runs ONCE when "STOP" is pressed.
     * Use this to safely shut down motors and other hardware.
     */
    @Override
    public void stop() {
        robot.stopAll();
    }

    // =========================================================================================
    //                            Helper Methods for the Loop()
    // =========================================================================================

    private void updateColorSensor() {
        colorSensor.update();
        ColorSensor.BallColor color = colorSensor.getBallColor();

        if (color == ColorSensor.BallColor.PURPLE) {
            indicator.setPurple();
        } else if (color == ColorSensor.BallColor.GREEN) {
            indicator.setGreen();
        } else {
            indicator.off();
        }
    }

    private void handleDriveControls() {
        double y = -gamepad1.left_stick_y;  // forward/back
        double x =  gamepad1.left_stick_x;  // strafe
        double r =  gamepad1.right_stick_x; // rotate
        robot.mecanumDrive.drive(x, y, r);
    }

    private void handleDumpMode() {
        // Toggle logic for dump mode
        boolean rt = gamepad2.right_trigger > 0.6;
        if (rt && !prevRT) {
            dumpMode = !dumpMode;
            if (dumpMode) {
                dumpCount = 0;
                launchTimer.reset();
            }
        }
        prevRT = rt;

        // Main dump mode logic
        if (dumpMode) {
            robot.door.forceOpenLock();

            // Auto-advance spindexer
            if (launchTimer.milliseconds() > LAUNCH_DELAY_MILLISECONDS) {
                robot.spindexer.nextCompartment();
                dumpCount++; // Increment count each time we advance
                launchTimer.reset();
            }

            // End dump mode after 3 compartments
            if (dumpCount >= 3) {
                dumpMode = false;
                robot.door.unlock();
                robot.door.forceClose();
            }
        }
    }

    // Add this new helper method to your BlueTelop class
    private void handleManualShoot() {
        switch (shootState) {
            case IDLE:
                // If the Y button is pressed and we're not busy with another sequence...
                if (gamepad2.y && !dumpMode) {
                    // 1. Open the door
                    robot.door.forceOpenLock();
                    doorTimer.reset();
                    // 2. Move to the next state
                    shootState = ShootState.OPENING_DOOR;
                }
                break;

            case OPENING_DOOR:
                // Wait for a short time (e.g., 250ms) for the servo to move
                if (doorTimer.milliseconds() > 250) {
                    // 1. Close the door
                    robot.door.forceClose();
                    doorTimer.reset();
                    // 2. Move to the next state
                    shootState = ShootState.CLOSING_DOOR;
                }
                break;

            case CLOSING_DOOR:
                // Wait again for the servo to close
                if (doorTimer.milliseconds() > 250) {
                    // 1. Advance the spindexer to the next compartment
                    robot.spindexer.nextCompartment();
                    // 2. Return to IDLE, ready for the next button press
                    shootState = ShootState.IDLE;
                }
                break;

            // This case is not used in the sequence but good to have
            case INDEXING:
                // This state could be used if indexing took time, but nextCompartment() is instant.
                // We can go directly back to IDLE.
                shootState = ShootState.IDLE;
                break;
        }
    }

    private void updateTelemetry() {
        // Display the robot's current position on the telemetry
        Pose currentPose = follower.getPose();
        telemetry.addData("Current Pose", "X: %.2f, Y: %.2f, H: %.2f",
                currentPose.getX(), currentPose.getY(), currentPose.getHeading());
        telemetry.addData("Dump Mode", dumpMode);
        telemetry.addData("Intake Comp", robot.spindexer.getIntakeCompartment());
        telemetry.addData("Shooter Comp", robot.spindexer.getShooterCompartment());
        telemetry.addData("Next Up", robot.spindexer.getNextUpCompartment());
        telemetry.addData("Magnet", robot.spindexerMag.isTriggered());
        telemetry.addData("Detected Color", colorSensor.getBallColor());
        telemetry.addData("Ball Present", colorSensor.isBallPresent());
        telemetry.update();
    }
}
