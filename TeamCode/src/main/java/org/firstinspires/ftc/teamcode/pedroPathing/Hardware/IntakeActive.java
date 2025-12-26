package org.firstinspires.ftc.teamcode.pedroPathing.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * IntakeActive subsystem — controls the active intake motor.
 * Intake runs continuously unless gamepad2 left bumper is pressed.
 */
public class IntakeActive {
    private final DcMotorEx intake;
    private OpMode myOpMode;

    // Speed constants
    private static final double INTAKE_ON  = -1.0;
    private static final double INTAKE_OFF =  0.0;
    private static final double INTAKE_REVERSE=  1.0;


    public IntakeActive(OpMode opMode) {
        myOpMode = opMode;
        intake = opMode.hardwareMap.get(DcMotorEx.class, "intake");
        intake.setPower(INTAKE_OFF); // ensure off at init
    }

    public void intakeOn() {
        intake.setPower(INTAKE_ON); // ensure off at init
    }

    public void intakeOff() {
        intake.setPower(INTAKE_ON); // ensure off at init
    }

    /** Runs intake based on gamepad2 left bumper input. */
    public void run() {
        if (myOpMode.gamepad2.left_bumper) {
            // Turn intake off when gamepad2 left bumper is pressed
            intake.setPower(INTAKE_OFF);
        }else if (myOpMode.gamepad2.left_trigger>0.1) {
            // Gamepad2 left trigger reverses the intake
            intake.setPower(INTAKE_REVERSE);
        } else {
            // Otherwise, intake is always running
            intake.setPower(INTAKE_ON);
        }

        }
}

