package org.firstinspires.ftc.teamcode.pedroPathing.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class SpindexerMotor {

    private final DcMotorEx motor;

    // These constants are correct based on your previous file.
    public static final int TICKS_PER_REV = 4063;
    public static final int COMPARTMENTS = 3;
    public static final int TICKS_PER_COMPARTMENT = TICKS_PER_REV / COMPARTMENTS; // Approximately 1354

    public static final double TUNED_P = 12.0; // A slightly higher P might give a snappier response
    public static final double TUNED_I = 3.0;
    public static final double TUNED_D = 0.5;  // A small D can help reduce overshoot
    public static final double TUNED_F = 0.0;


    public SpindexerMotor(HardwareMap hwMap) {
        motor = hwMap.get(DcMotorEx.class, "spindexer");

        // The direction might need to be FORWARD or REVERSE depending on your motor orientation.
        // If the motor spins the wrong way, change this to REVERSE.
        motor.setDirection(DcMotor.Direction.REVERSE);

        // This behavior is crucial for holding position.
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset the encoder to 0 at startup. This is a critical step.
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        PIDFCoefficients pidfCoefficients = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);

        // Update the coefficients with YOUR tuned values.
        pidfCoefficients.p = TUNED_P;
        pidfCoefficients.i = TUNED_I;
        pidfCoefficients.d = TUNED_D;
        pidfCoefficients.f = TUNED_F;

        // Set an initial target position of 0.
        motor.setTargetPosition(0);

        // IMPORTANT: Switch the motor to RUN_TO_POSITION mode.
        // It will now actively try to reach and hold the target position.
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Commands the motor to move to a specific encoder tick value.
     * The motor will use the power it was last given to move to this position.
     *
     * @param ticks The target encoder position.
     */
    public void setTargetTicks(int ticks) {
        motor.setTargetPosition(ticks);
    }

    /**
     * Commands the motor to move to a specific encoder tick value with a new power setting.
     *
     * @param ticks The target encoder position.
     * @param power The power (speed) to use for the movement (0.0 to 1.0).
     */
    public void setTargetTicks(int ticks, double power) {
        motor.setTargetPosition(ticks);
        motor.setPower(power);
    }

    /**
     * Resets the motor's current encoder reading to zero.
     * This is useful for re-homing the spindexer.
     */
    public void resetEncoder() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(0); // It's good practice to reset the target as well.
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(1.0); // Re-apply power to hold the new zero position.
    }

    /**
     * Provides direct power control to the motor.
     * NOTE: This will override RUN_TO_POSITION behavior until the mode is reset.
     * It should primarily be used for manual homing routines.
     *
     * @param power The power to apply (-1.0 to 1.0).
     */
    public void setPower(double power) {
        // To use setPower for manual control, we must switch modes.
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPower(power);
    }

    /**
     * Stops all motor movement and re-engages RUN_TO_POSITION to hold position.
     */
    public void stop() {
        motor.setPower(0);
        // It's often better to re-engage RUN_TO_POSITION to hold the current spot.
        setTargetTicks(getCurrentPosition());
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(1.0);
    }

    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    /**
     * Checks if the motor is currently moving towards a target position.
     * @return true if the motor is busy, false otherwise.
     */
    public boolean isBusy() {
        return motor.isBusy();
    }

    /** Returns current compartment index: 0, 1, or 2 */
    public int getCompartment() {
        // Using modulo allows this to work even if the encoder count is very high.
        int effectivePosition = getCurrentPosition() % TICKS_PER_REV;
        if (effectivePosition < 0) {
            effectivePosition += TICKS_PER_REV;
        }
        return effectivePosition / TICKS_PER_COMPARTMENT;
    }

    /** Returns compartment as 1, 2, or 3 for telemetry */
    public int getCompartmentDisplay() {
        return getCompartment() + 1;
    }
}
