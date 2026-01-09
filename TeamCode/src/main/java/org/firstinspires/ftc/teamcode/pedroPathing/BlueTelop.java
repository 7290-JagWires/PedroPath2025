package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Shooter.SHOOT_GOAL_VELOCITY;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Utilities.DumpManager;
import org.firstinspires.ftc.teamcode.pedroPathing.Utilities;

import kotlin.time.Instant;

@TeleOp()
public class BlueTelop extends OpMode {

    private static final int LAUNCH_DELAY_MILLISECONDS = 1000;

    // Class member variables
    private Robot robot;

    // Homing state variables
    private enum HomingState { HOMING, COMPLETE }
    private HomingState homingState = HomingState.HOMING;
    private Follower follower;
    private boolean Ypressed;


    /**
     * This method runs ONCE when the driver hits "INIT" on the Driver Station.
     * This is where you initialize all your hardware.
     */
    @Override
    public void init() {
        telemetry.addLine("Initializing...");

        robot = new Robot(this);

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
        // Run the main robot update loop
        robot.update();

        // --- Sensor Updates ---
        Utilities.isBall(robot.colorSensor, robot.indicator);

        Ypressed = gamepad2.y;
        if (Ypressed) {
           robot.shooter.setExplicitVelocity(SHOOT_GOAL_VELOCITY);
///
///             we will need to use distance to toggle between shooting goals and triangles
///             this one will be needed if we use the dpad for speeding up and down before shooting
///             robot.shooter.setExplicitVelocity(Shooter.SHOOT_TRIANGLE_VELOCITY);
///
           robot.manualShootManager.update(Ypressed, robot.dumpManager.isDumping());
       }

        // --- Driver 1: Drivetrain ---
        Utilities.handleRobotCentricDrive(gamepad1, robot);

        // --- Driver 2: Mechanisms ---
        // The manager Instant.Companion.now handles the toggle logic and the entire state machine.
        robot.dumpManager.updateTeleOp(gamepad2.right_trigger > 0.6);



        // Handle Manual Rotation forward and back
        if (gamepad2.aWasPressed()) {
            robot.spindexerRotator.start(1);  //we now rotate one compartment
        }

        if (gamepad2.bWasPressed()) {
            robot.spindexerLogic.previousCompartment();  //we now rotate one compartment backwards
        }

        //Handle a manual homing
        if (gamepad2.xWasPressed()) {
            homingState = HomingState.HOMING;
            handleHoming();
        }

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
        telemetry.addData("Y Pressed State", Ypressed); // <-- Optional: new telemetry
        telemetry.update();
    }
}
