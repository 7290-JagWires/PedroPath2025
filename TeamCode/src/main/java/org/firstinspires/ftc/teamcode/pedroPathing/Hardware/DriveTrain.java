package org.firstinspires.ftc.teamcode.pedroPathing.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveTrain {
    public DcMotorEx left_front;
    public DcMotorEx left_rear;
    public DcMotorEx right_front;
    public DcMotorEx right_rear;

    public DriveTrain(HardwareMap hardwareMap) {
        left_front  = hardwareMap.get(DcMotorEx.class, "left_front");
        left_rear   = hardwareMap.get(DcMotorEx.class, "left_rear");
        right_front = hardwareMap.get(DcMotorEx.class, "right_front");
        right_rear  = hardwareMap.get(DcMotorEx.class, "right_rear");

        // --- Motor directions ---
        // Try this setup first (most FTC mecanum builds use this)
        left_front.setDirection(DcMotor.Direction.FORWARD);
        left_rear.setDirection(DcMotor.Direction.FORWARD);
        right_front.setDirection(DcMotor.Direction.REVERSE);
        right_rear.setDirection(DcMotor.Direction.REVERSE);

        // --- Encoders enabled ---
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_rear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_rear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_rear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_rear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_rear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_rear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stop();
    }

    public void stop() {
        left_front.setPower(0);
        right_front.setPower(0);
        left_rear.setPower(0);
        right_rear.setPower(0);
    }
}
