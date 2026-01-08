package org.firstinspires.ftc.teamcode.pedroPathing.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {

    private OpMode myOpMode;
    private final DcMotorEx shooter;

    // POWER MODE CONSTANTS
    private static final double SHOOTER_FORWARD = 0.8;
    private static final double SHOOTER_REVERSE = -0.8;
    private static final double SHOOTER_OFF     = 0.0;

    private static final double SHOOT_TRIANGLE_VELOCITY = 2650;
    private static final double SHOOT_GOAL_VELOCITY = 1900;


    public Shooter(OpMode opMode) {
        myOpMode = opMode;


        shooter = myOpMode.hardwareMap.get(DcMotorEx.class, "shooter");

        // Start in encoder mode to allow velocity control later
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        shooter.setVelocity(SHOOTER_OFF);

        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    // -------------------------------------------------------------
    // TELEOP RUNTIME CONTROL
    // -------------------------------------------------------------
    /** Default behavior: shooter ON unless RB pressed or LT reverse */
    public void run() {

        if (myOpMode.gamepad2.right_bumper) {
            stopFast();     // active braking
        }
        else if (myOpMode.gamepad2.left_trigger > 0.1) {
            shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooter.setPower(SHOOTER_REVERSE);
        }
        else {
            shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooter.setPower(SHOOTER_FORWARD);
        }
    }
    public void stop() {
        shooter.setPower(SHOOTER_OFF);
    }

    // -------------------------------------------------------------
    // ACTIVE BRAKING (FAST STOP)
    // -------------------------------------------------------------
    /** Stops the shooter quickly with a short reverse pulse */
    public void stopFast() {
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Apply short reverse pulse to cancel inertia
        shooter.setPower(-0.20);
//        wait(50);
//        myOpMode.sleep(50);

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
//        wait(50);
//        myOpMode.sleep(50);

        shooter.setPower(0);
    }

    /** Allow ShooterTest to read shooter velocity */
    public double getVelocity() {
        return shooter.getVelocity();
    }
}
