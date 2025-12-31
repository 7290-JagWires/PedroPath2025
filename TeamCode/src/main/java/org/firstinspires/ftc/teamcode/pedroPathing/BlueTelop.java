package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Sensors.ColorSensor;
import org.firstinspires.ftc.teamcode.pedroPathing.Sensors.IndicatorLight;

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

    // Homing state variables
    private enum HomingState { HOMING, COMPLETE }
    private HomingState homingState = HomingState.HOMING;

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
        handleSpindexerControls();

        // Run the main robot update loop
        robot.update();

        // Auto-close door after indexing
        if (robot.spindexer.justIndexed) {
            robot.door.forceClose();
            robot.spindexer.justIndexed = false;
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

    private void handleSpindexerControls() {
        // Manual spindexer controls (only if NOT in dump mode)
        if (!dumpMode) {
            if (gamepad2.a) {
                robot.spindexer.nextCompartment();
            }
            if (gamepad2.b) {
                robot.spindexer.previousCompartment();
            }
        }
    }

    private void updateTelemetry() {
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
