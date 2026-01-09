package org.firstinspires.ftc.teamcode.pedroPathing.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Sensors.MagneticLimitSwitch;

public class Spindexer {

    private final DcMotorEx motor;
    private final MagneticLimitSwitch limit;

    // constants
    public static final int COMPARTMENTS = 3;
    public static final int TICKS_PER_REV = 4063;
    public static final int TICKS_PER_COMPARTMENT = TICKS_PER_REV / COMPARTMENTS;

    private static final double INDEX_POWER = 0.9;


    // 0 = Comp1, 1 = Comp2, 2 = Comp3
    private int compartmentIndex = 0;
    private int moveDirection = 0; // 1 for forward, -1 for backward, 0 for stopped

    // Lets TeleOp know a move happened (used for auto-close door)
    public boolean justIndexed = false;

   // boolean limitSwitchTriggered = false;

    boolean lookingForLimitSwitch = false;

    private void resetEncoder() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void runUsingEncoder() {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }



    public Spindexer(OpMode opMode, MagneticLimitSwitch limit) {
        motor = opMode.hardwareMap.get(DcMotorEx.class, "spindexer");
        this.limit = limit;


        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void setModeRunUsingEncoder() {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void stop() {
        motor.setPower(0);
        moveDirection = 0;
    }

    public void update() {
        limit.update();
        updateMagnetZero();

        if (checkLimitReached()) {
            stop();
            advanceIndex();
            justIndexed = true;
        }
    }

    public void setPower(double power) {
        motor.setPower(power);
    }


    //Moved from original logic class
    public void updateMagnetZero() {
        if (limit.wasJustTriggered()) {
            resetEncoder();
            compartmentIndex = 0;
        }
    }

    //This is new/changed - do not know if it will work
    private boolean checkLimitReached() {
        if (lookingForLimitSwitch && limit.isTriggered()) {
            lookingForLimitSwitch = false;
            return true;
        }

        if (!limit.isTriggered()) {
            lookingForLimitSwitch = true;
        }

        return false;
    }

    private void advanceIndex() {
        if (moveDirection == 1) {
            compartmentIndex = (compartmentIndex + 1) % COMPARTMENTS;
        } else if (moveDirection == -1) {
            compartmentIndex--;
            if (compartmentIndex < 0) compartmentIndex = COMPARTMENTS - 1;
        }
    }

    public int getIntakeCompartment() { return compartmentIndex + 1; }
    public int getShooterCompartment() { return ((compartmentIndex + 1) % 3) + 1; }
    public int getNextUpCompartment() { return ((compartmentIndex + 2) % 3) + 1; }

}
