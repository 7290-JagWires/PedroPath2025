package org.firstinspires.ftc.teamcode.pedroPathing.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TestingFerrisWheel {

    private final DcMotorEx motor;

    public static final int TICKS_PER_REV = 4063;
    public static final int COMPARTMENTS = 3;
    public static final int TICKS_PER_COMPARTMENT = TICKS_PER_REV / COMPARTMENTS;

    public TestingFerrisWheel(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "spindexer");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setTargetPosition(int targetPosition, double power) {
        motor.setTargetPosition(targetPosition);
        motor.setPower(power);
    }

    public boolean isBusy() {
        return motor.isBusy();
    }

    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }
}
