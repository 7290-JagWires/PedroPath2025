package org.firstinspires.ftc.teamcode.pedroPathing.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Shooter {

    private final OpMode myOpMode;
    private final DcMotorEx shooter;

    // --- Velocity Constants (in Ticks per Second) ---
    // Public so other classes can see them (like your Auto op-modes)
    public static final double SHOOTER_OFF_VELOCITY = 0;
    public static final double SHOOTER_ADJUSTMENT_VELOCITY = 50;
    public static final double SHOOT_TRIANGLE_VELOCITY = 2550;
    public static final double SHOOT_DEFENSE_VELOCITY = 2200;
    public static final double SHOOT_GOAL_VELOCITY = 1970;
    public static final double VELOCITY_TOLERANCE = 25;
    public static final double NEGATIVE_VELOCITY = -1000;


    // This variable will store the current "default" speed for TeleOp
    private double targetVelocity = SHOOT_GOAL_VELOCITY; // Default to goal velocity on startup

    public Shooter(OpMode opMode) {
        myOpMode = opMode;

        shooter = myOpMode.hardwareMap.get(DcMotorEx.class, "shooter");

        shooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);

        // Start with the motor off
        shooter.setVelocity(SHOOTER_OFF_VELOCITY);
    }

    // -------------------------------------------------------------
    // PRIMARY CONTROL METHODS
    // -------------------------------------------------------------

    /**
     * Sets the default shooting velocity that will be used when setTargetVelocity() is called.
     * Call this from your Auto op-mode to prepare for TeleOp.
     *
     * @param defaultVelocity The desired default speed (e.g., SHOOT_TRIANGLE_VELOCITY).
     */
    public void setDefaultVelocity(double defaultVelocity) {
        this.targetVelocity = defaultVelocity;
    }

    /**
     * Spins the shooter up to the currently set default velocity.
     */
    public void setTargetVelocity() {
        shooter.setVelocity(this.targetVelocity);
    }

    /**
     * Overrides the default and sets the shooter to a specific velocity.
     * Useful for distance-based shooting.
     *
     * @param velocity The specific target velocity in ticks per second.
     */
    public void setExplicitVelocity(double velocity) {
        shooter.setVelocity(velocity);
    }

    /**
     * Stops the shooter by setting its target velocity to 0.
     * The motor's PID will actively work to hold this zero velocity.
     */
    public void stopShooter() {
        shooter.setVelocity(SHOOTER_OFF_VELOCITY);
    }


    // -------------------------------------------------------------
    // UTILITY METHODS
    // -------------------------------------------------------------

    /**
     * Gets the current velocity of the shooter motor.
     *
     * @return The current velocity in ticks per second.
     */
    public double getVelocity() {
        return shooter.getVelocity();
    }

    /**
     * Gets the currently configured default target velocity.
     *
     * @return The default velocity in ticks per second.
     */
    public double getDefaultVelocity() {
        return this.targetVelocity;
    }

    // Add this method inside your Shooter.java file

    /**
     * Returns the current target velocity of the shooter.
     *
     * @return The target velocity in ticks per second.
     */
    public boolean shooterAtSpeed(Robot robot, LimelightCamera.TagData detectedTag) {

        if (detectedTag != null)
        {
            if (detectedTag.distance > 70  && detectedTag.distance < 115) {
                return (Math.abs(robot.shooter.getVelocity() - SHOOT_DEFENSE_VELOCITY) <= VELOCITY_TOLERANCE);
            } else if (detectedTag.distance >= 115) {
                return (Math.abs(robot.shooter.getVelocity() - SHOOT_TRIANGLE_VELOCITY) <= VELOCITY_TOLERANCE);
            } else {
                return (Math.abs(robot.shooter.getVelocity() - SHOOT_GOAL_VELOCITY) <= VELOCITY_TOLERANCE);
            }
        }
            return true;
        }
    }
