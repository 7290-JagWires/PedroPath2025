package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.TestingFerrisWheel;

@Disabled
@TeleOp(name = "Test Ferris Wheel", group = "Test")
public class TestFerrisWheelTeleOp extends OpMode {

    private double motorPower = 0.75;

    private TestingFerrisWheel ferrisWheel;

    // Button debouncing helpers
    private boolean aPressedLast = false;
    private boolean bPressedLast = false;
    private boolean dpadUpPressedLast = false;
    private boolean dpadDownPressedLast = false;

    private int targetPosition = 0;

    @Override
    public void init() {
        ferrisWheel = new TestingFerrisWheel(hardwareMap);
        telemetry.log().add("Brand new Ferris Wheel Test Ready!");

        // --- Instructions ---
        telemetry.addLine("Controls:");
        telemetry.addLine("A Button: Move Forward One Compartment");
        telemetry.addLine("B Button: Move Backward One Compartment");
        telemetry.addLine("D-Pad Up/Down: Adjust Motor Power");
        telemetry.update();
    }

    @Override
    public void loop() {
        // --- Power Adjustment ---
        if (gamepad1.dpad_up && !dpadUpPressedLast) {
            motorPower = Math.min(motorPower + 0.05, 1.0);
        }
        dpadUpPressedLast = gamepad1.dpad_up;

        if (gamepad1.dpad_down && !dpadDownPressedLast) {
            motorPower = Math.max(motorPower - 0.05, 0.0);
        }
        dpadDownPressedLast = gamepad1.dpad_down;

        // --- Compartment Movement ---
        // Only set a new target if the motor isn'''t already running to a position.
        if (!ferrisWheel.isBusy()) {
            if (gamepad1.a && !aPressedLast) {
                targetPosition += TestingFerrisWheel.TICKS_PER_COMPARTMENT;
            }

            if (gamepad1.b && !bPressedLast) {
                targetPosition -= TestingFerrisWheel.TICKS_PER_COMPARTMENT;
            }
        }

        // Update the motor'''s target position and power. This is safe to call every loop.
        ferrisWheel.setTargetPosition(targetPosition, motorPower);

        aPressedLast = gamepad1.a;
        bPressedLast = gamepad1.b;

        // --- Telemetry ---
        telemetry.addData("Motor Power", "%.2f", motorPower);
        telemetry.addData("Target Position", targetPosition);
        telemetry.addData("Current Position (Encoder Ticks)", ferrisWheel.getCurrentPosition());
        telemetry.addData("Motor Busy", ferrisWheel.isBusy());
        telemetry.update();
    }
}
