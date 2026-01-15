package org.firstinspires.ftc.teamcode.TeleOp.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TeleOpSpindexerMotor {

    private final DcMotorEx motor;

    // constants
    public static final int TICKS_PER_REV = 4063;
    public static final int COMPARTMENTS = 3;
    public static final int TICKS_PER_COMPARTMENT = TICKS_PER_REV / COMPARTMENTS;

    public void setPower(double power) {
        motor.setPower(power);
    }


    public TeleOpSpindexerMotor(HardwareMap hwMap) {
        motor = hwMap.get(DcMotorEx.class, "spindexer");

        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motor.setTargetPosition(0);
//        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // ---------------------------------------------------------
    // Helper: switch to RUN_USING_ENCODER mode (for homing)
    // ---------------------------------------------------------
    public void setModeRunUsingEncoder() {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // ---------------------------------------------------------
    // Helper: switch back to RUN_TO_POSITION mode
    // ---------------------------------------------------------
//    public void setModeRunToPosition() {
//        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    }

    public void resetEncoder() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //motor.setTargetPosition(0);
        //motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setTargetTicks(int ticks, double power) {
        motor.setTargetPosition(ticks);
        motor.setPower(power);
        //motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    public boolean isBusy() {
        return motor.isBusy();
    }

    public void stop() {
        motor.setPower(0);
    }

    /** Returns current compartment index: 0, 1, or 2 */
    public int getCompartment() {
        int index = getCurrentPosition() / TICKS_PER_COMPARTMENT;
        return index % COMPARTMENTS;
    }

    /** Returns compartment as 1, 2, or 3 for telemetry */
    public int getCompartmentDisplay() {
        return getCompartment() + 1;
    }

    // ---------------------------------------------------------
    // Optional but useful: expose underlying motor if needed
    // ---------------------------------------------------------
    public DcMotorEx getMotor() {
        return motor;
    }
}
