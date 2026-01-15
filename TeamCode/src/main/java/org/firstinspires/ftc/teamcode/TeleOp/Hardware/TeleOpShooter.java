package org.firstinspires.ftc.teamcode.TeleOp.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TeleOpShooter {

    private final LinearOpMode opMode;
    private final DcMotorEx shooter;

    // POWER MODE CONSTANTS
    //private static final double SHOOTER_FORWARD = 0.9;
    //private static final double SHOOTER_REVERSE = -0.9;

    private static final double SHOOTER_FORWARD_LONG_GOAL = 1.0;
    private static final double SHOOTER_FORWARD_SHORT_GOAL = 0.75;
    private static final double SHOOTER_OFF     = 0.0;

    public TeleOpShooter(HardwareMap hardwareMap, LinearOpMode opMode) {
        this.opMode = opMode;

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        // Start in encoder mode to allow velocity control later
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Force brake when power = 0 (helps but not enough alone)
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter.setPower(0);
    }

    // -------------------------------------------------------------
    // TELEOP RUNTIME CONTROL
    // -------------------------------------------------------------
    /** Default behavior: shooter ON unless RB pressed or LT reverse */
    public void run() {

        if (opMode.gamepad2.right_bumper) {
            shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooter.setPower(SHOOTER_FORWARD_LONG_GOAL);
        }
        else {
            shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooter.setPower(SHOOTER_FORWARD_SHORT_GOAL);
        }
    }

    // -------------------------------------------------------------
    // ACTIVE BRAKING (FAST STOP)
    // -------------------------------------------------------------
    /** Stops the shooter quickly with a short reverse pulse */
    public void stopFast() {
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Apply short reverse pulse to cancel inertia
        shooter.setPower(-0.20);
        opMode.sleep(50);

        shooter.setPower(0);
    }

    // -------------------------------------------------------------
    // VELOCITY CONTROL FOR SHOOTERTEST
    // -------------------------------------------------------------
    /** Switch to velocity mode and set ticks/sec */
    public void setVelocity(double velocity) {
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setVelocity(velocity);
    }

    /** Stops the shooter from velocity mode using active braking */
    public void stopVelocity() {
        // Leave velocity mode → go to power mode for braking
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter.setPower(-0.20);
        opMode.sleep(50);

        shooter.setPower(0);
    }

    /** Allow ShooterTest to read shooter velocity */
    public double getVelocity() {
        return shooter.getVelocity();
    }
}
