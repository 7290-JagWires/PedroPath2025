package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Robot;

@TeleOp()
public class BlueTelop extends OpMode {

   // Class member variables
    private Robot robot;
    private Follower follower;

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
        if (!robot.spindexer.isHomed()) {
            telemetry.addLine("Homing Spindexer...");

            // Tell the spindexer to start the homing process if it hasn't already.
            robot.spindexer.startHoming();

            // Run the homing state machine.
            robot.spindexer.home();

        } else {
            telemetry.addLine("Homing Complete. Ready to Start.");
        }

        // Display the internal state for debugging.
        telemetry.addData("Spindexer Homing State", robot.spindexer.getHomingState());
        telemetry.update();    }

    /**
     * This method runs ONCE when the driver hits "PLAY".
     * Any actions that should happen right at the start of the match go here.
     */
    @Override
    public void start() {
        // You could reset timers here if needed, but it's often empty for TeleOp.

    }

    /**
     * This method runs in a loop continuously from the moment "PLAY" is pressed until "STOP" is pressed.
     * This is where all your driver controls and main logic will live.
     */
    @Override
    public void loop() {
        // --- Sensor Updates ---
        Utilities.isBall(robot.colorSensor, robot.indicator);

        // --- Driver 1: Drivetrain ---
        Utilities.handleRobotCentricDrive(gamepad1, robot);

        // --- Driver 2: Controller ---
        robot.manualShootManager.update(gamepad2.yWasPressed(), robot.dumpManager.isDumping());
        robot.dumpManager.updateTeleOp(gamepad2.right_trigger > 0.6);

        robot.teleOpPickupManager.update(gamepad2.bWasPressed());

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
        telemetry.update();
    }
}
