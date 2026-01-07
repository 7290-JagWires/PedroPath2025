package org.firstinspires.ftc.teamcode.pedroPathing.Logic;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.SpindexerMotor;
import org.firstinspires.ftc.teamcode.pedroPathing.Sensors.MagneticLimitSwitch;
import org.firstinspires.ftc.teamcode.pedroPathing.Utilities;

public class SpindexerIndexerLogic {

    private final OpMode opMode;
    private final SpindexerMotor motor;
    private final MagneticLimitSwitch limit;
    // In SpindexerIndexerLogic.java
    public enum IndexerState { IDLE, MOVING, REHOMING, FINISHED }

    // This is the only state variable we need to track.
    private int currentTargetCompartment = 0;
    private int cumulativeTargetTicks = 0; // Tracks the absolute target position
    // Lets TeleOp know a move happened (used for auto-close door)
    public boolean justIndexed = false;

    private IndexerState indexerState = IndexerState.IDLE;

    public SpindexerIndexerLogic(OpMode opMode,
                                 SpindexerMotor motor,
                                 MagneticLimitSwitch limit,
                                 int ticksPerCompartment) { // ticksPerCompartment is not used anymore but can stay for now

        this.opMode = opMode;
        this.motor = motor;
        this.limit = limit;

        // The SpindexerMotor constructor already sets the motor to RUN_TO_POSITION
        // and holds position 0. We don't need to do anything here.
    }

    /**
     How this should work
     - CurrentTargetCompartment starts at 0.
     - We add 1 to it, making it 1.
     - The modulo operator (%) calculates the remainder of a division. Since COMPARTMENTS is 3, we get: 1 % 3 equals 1.
     - The result is stored back into currentTargetCompartment.
     - Outcome: currentTargetCompartment is now 1. This line of code handles the "wrap-around" logic automatically.
                When currentTargetCompartment is 2, the calculation becomes (2 + 1) % 3, which is 3 % 3,
                and the remainder is 0. So the sequence will always be 0 -> 1 -> 2 -> 0 -> ....
     - Now that we know which compartment we want (1), we need to translate that into a physical position for the motor.
     - The SpindexerMotor class defines TICKS_PER_COMPARTMENT (which is 4063 / 3, or approximately 1354).
     - The calculation becomes: targetTicks = 1 * 1354•Outcome: The variable targetTicks is now 1354.
       The motor needs to spin until its encoder reads this value.
     - We set the power to 100% and move to the target.
     -

     */
    // In SpindexerIndexerLogic.java
    public void nextCompartment() {
        if (indexerState == IndexerState.IDLE || indexerState == IndexerState.FINISHED) {
            // Check if the NEXT compartment is the zero position.
            if ((currentTargetCompartment + 1) % SpindexerMotor.COMPARTMENTS == 0) {
                // --- THIS IS A RE-HOMING ROTATION ---
                // Instead of using encoders, we'll spin with raw power and look for the magnet.
                motor.setPower(.5); // Use a method that sets RUN_USING_ENCODER
                indexerState = IndexerState.REHOMING;
                opMode.telemetry.addLine("Re-homing");
            } else {
                // --- THIS IS A NORMAL ENCODER ROTATION ---
                currentTargetCompartment = (currentTargetCompartment + 1) % SpindexerMotor.COMPARTMENTS;
                cumulativeTargetTicks += SpindexerMotor.TICKS_PER_COMPARTMENT;
                motor.setTargetTicks(cumulativeTargetTicks, .75);
                indexerState = IndexerState.MOVING;
            }
            justIndexed = true;
        }
    }
    /**
     * Checks if the homing magnet is detected and resets the encoder to zero.
     * This is useful for re-calibrating the spindexer's position.
     */
    public void updateMagnetZero() {
        if (limit.wasJustTriggered()) {
            motor.resetEncoder();
            // When we reset at the magnet, we know we are at compartment 0.
            currentTargetCompartment = 0;
        }
    }
    public void resetPosition() {
        currentTargetCompartment = 0;
        cumulativeTargetTicks = 0;
    }
    /**
     * Main update loop for this logic class. Call this from your Robot's update() method.
     */
    // In SpindexerIndexerLogic.java
    public void update() {
        limit.update();

        switch (indexerState) {
            case MOVING:
                // For normal encoder moves, we just wait for the motor to be done.
                if (!motor.isBusy()) {
                    indexerState = IndexerState.FINISHED;
                }
                break;

            case REHOMING:
                // For re-homing moves, we wait for the rising edge of the magnet.
                if (limit.wasJustTriggered()) {
                    motor.stop();         // Stop the motor immediately.
                    motor.resetEncoder(); // Reset the physical encoder to 0.
                    resetPosition();      // Reset all our software counters (cumulative ticks, etc.).
                    // We are now at a perfect, drift-free zero.
                    indexerState = IndexerState.FINISHED;
                }
                break;

            // IDLE and FINISHED do nothing.
            case IDLE:
            case FINISHED:
                break;
        }
    }

    public int getIntakeCompartment() { return currentTargetCompartment + 1; }
    public int getShooterCompartment() { return ((currentTargetCompartment + 1) % 3) + 1; }
    public int getNextUpCompartment() { return ((currentTargetCompartment + 2) % 3) + 1; }

    public boolean isFinished() {
        return indexerState == IndexerState.IDLE || indexerState == IndexerState.FINISHED;
    }

    /**
     * Checks if the spindexer motor is currently moving to a target position.
     * @return true if the motor is busy, false if it has reached its target.
     */
    public boolean isBusy() {
        // Pass the call through to the underlying motor.
        return motor.isBusy();
    }
}
