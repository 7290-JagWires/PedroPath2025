package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Shooter.SHOOT_GOAL_VELOCITY;
import static org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Shooter.SHOOT_TRIANGLE_VELOCITY;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.LimelightCamera;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.GamepadManager;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.LimelightCamera.TagData;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Utilities.DumpManager;
import org.firstinspires.ftc.teamcode.pedroPathing.Utilities;

import kotlin.time.Instant;

@TeleOp()
public class BlueTelop extends OpMode {

    private static final int LAUNCH_DELAY_MILLISECONDS = 1000;
    // Class member variables
    public boolean tagDetectionLogicHasRun = false;
    private int startingTagId = -1;

    private Robot robot;
    // Homing state variables
    private enum HomingState { IDLE, HOMING, COMPLETE }
    private HomingState homingState = HomingState.IDLE;
    private Follower follower;
    private boolean Ypressed;

    private GamepadManager gamepadManager1;
    private GamepadManager gamepadManager2;
    private TagData detectedTag;

    /**
     * This method runs ONCE when the driver hits "INIT" on the Driver Station.
     * This is where you initialize all your hardware.
     */
    @Override
    public void init() {
        robot = new Robot(this);

        follower = Constants.createFollower(hardwareMap);
        Pose startingPose = PoseStorage.loadPoseFromFile(hardwareMap);
        follower.setPose(startingPose);

        // Initialize the GamepadManagers
        gamepadManager1 = new GamepadManager();
        gamepadManager2 = new GamepadManager();

        // Start the homing process on init
        homingState = HomingState.HOMING;

        tagDetectionLogicHasRun = false;
        startingTagId = -1; // Reset the tag ID at the beginning of every init

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
        handleHoming();
        telemetry.update();
    }

    /**
     * This method runs ONCE when the driver hits "PLAY".
     * Any actions that should happen right at the start of the match go here.
     */
    @Override
    public void start() {
        // You could reset timers here if needed, but it's often empty for TeleOp.
        robot.shooter.setExplicitVelocity(SHOOT_GOAL_VELOCITY);
    }

    /**
     * This method runs in a loop continuously from the moment "PLAY" is pressed until "STOP" is pressed.
     * This is where all your driver controls and main logic will live.
     */
    @Override
    public void loop() {
        // --- UPDATE HELPERS (Call these at the top of the loop!) ---
        gamepadManager1.update(gamepad1); // Update button states for gamepad 1
        gamepadManager2.update(gamepad2); // Update button states for gamepad 2

        // Run the main robot update loop
        robot.update();

        // IMPORTANT: Update the follower in every loop cycle to track position
        follower.update();

        //Limelight updates
        detectedTag = robot.limelight.getAprilTagData();

        // --- Main State Machine ---
        // If we are currently homing, block all other actions and continue homing.
        if (homingState == HomingState.HOMING) {
            handleHoming();
        } else {
            // --- Driver 1: Drivetrain ---
            Utilities.handleRobotCentricDrive(gamepad1, robot);

            // --- Driver 2: Mechanisms ---
            Utilities.isBall(robot.colorSensor, robot.indicator);

            // Shooter Controls
            // Use d-pad up/down to toggle between default shooting velocities
            if (gamepadManager2.dpad_up_just_pressed) {
                robot.shooter.setExplicitVelocity(SHOOT_GOAL_VELOCITY);        }
            if (gamepadManager2.dpad_down_just_pressed) {
                robot.shooter.setExplicitVelocity(SHOOT_TRIANGLE_VELOCITY);
            }

            if (gamepadManager2.y_just_pressed) {
                robot.shooter.setExplicitVelocity(SHOOT_GOAL_VELOCITY);
                robot.manualShootManager.update(true, robot.dumpManager.isDumping());
            } else {
                // IMPORTANT: Always call update, passing 'false' when the button isn't newly pressed.
                robot.manualShootManager.update(false, robot.dumpManager.isDumping());
            }

            //Dump Controls
            robot.dumpManager.updateTeleOp(gamepad2.right_trigger > 0.6);

            // Spindexer Manual Rotation
            if (gamepadManager2.a_just_pressed) {
                robot.spindexerRotator.start(1);  // Rotate one compartment forward
            }
            if (gamepadManager2.b_just_pressed) {
                robot.spindexer.previousCompartment(); // Rotate one compartment backward
            }

            // Manual Re-Homing
            if (gamepadManager2.x_just_pressed) {
                homingState = HomingState.HOMING;
            }

            // Auto-close door after indexing
            if (robot.spindexer.justIndexed) {
                robot.door.forceClose();
                robot.spindexer.justIndexed = false;
            }
        }

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
    /**
     * This is the new helper method that contains the homing logic.
     */
    private void handleHoming() {
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
    private void updateTelemetry() {
        // Display the robot's current position on the telemetry
        Pose currentPose = follower.getPose();
        telemetry.addData("Current Pose", "X: %.2f, Y: %.2f, H: %.2f",
                currentPose.getX(), currentPose.getY(), currentPose.getHeading());
        telemetry.addData("Dump Mode Active", robot.dumpManager.isDumping());
        telemetry.addData("Dump State", robot.dumpManager.getState());
        telemetry.addData("Intake Comp", robot.spindexer.getIntakeCompartment());
        telemetry.addData("Shooter Comp", robot.spindexer.getShooterCompartment());
        telemetry.addData("Next Up", robot.spindexer.getNextUpCompartment());
        telemetry.addData("Magnet", robot.spindexerMag.isTriggered());
        telemetry.addData("Detected Color", robot.colorSensor.getBallColor());
        telemetry.addData("Ball Present", robot.colorSensor.isBallPresent());
        telemetry.addData("Shoot State", robot.manualShootManager.getState()); // <-- Optional: new telemetry

        // Check if a tag was found
        if (detectedTag != null) {
            // Now you have all the data in a clean object!
            telemetry.addData("Tag ID", detectedTag.id);
            telemetry.addData("Angle to Camera (deg) (Tx)", "%.2f", detectedTag.tx);
            telemetry.addData("Angle from camera to center of Target (Ty)", "%.2f", detectedTag.ty);
            telemetry.addData("Exact distance from camera to tag", "%.2f", detectedTag.distance);
        } else {
            telemetry.addLine("No tags in view");
        }
        telemetry.update();
    }
}
