package org.firstinspires.ftc.teamcode.pedroPathing.Logic;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.SpindexerMotor;
import org.firstinspires.ftc.teamcode.pedroPathing.Sensors.MagneticLimitSwitch;

public class SpindexerIndexerLogic {

    private final OpMode opMode;
    private final SpindexerMotor motor;
    private final MagneticLimitSwitch limit;

    // This is the only state variable we need to track.
    private int currentTargetCompartment = 0;

    // Lets TeleOp know a move happened (used for auto-close door)
    public boolean justIndexed = false;

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
     * This is the NEW, REWRITTEN nextCompartment method.
     * It calculates the next target position and tells the motor to go there.
     */
    public void nextCompartment() {
        // 1. Increment the target compartment, wrapping around from 2 back to 0.
        currentTargetCompartment = (currentTargetCompartment + 1) % SpindexerMotor.COMPARTMENTS;

        // 2. Calculate the target encoder ticks for that compartment.
        int targetTicks = currentTargetCompartment * SpindexerMotor.TICKS_PER_COMPARTMENT;

        // 3. Tell the motor to go to the new position with a specific power.
        // The motor's RUN_TO_POSITION mode will handle the movement and stopping.
        motor.setTargetTicks(targetTicks, 1); // Using 100% power

        // 4. Signal that an indexing action has started.
        justIndexed = true;
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

    /**
     * Main update loop for this logic class. Call this from your Robot's update() method.
     */
    public void update() {
        limit.update();
        // You could optionally re-home if the magnet is seen, but this can be dangerous during a match.
        // updateMagnetZero();
    }

    public int getIntakeCompartment() { return currentTargetCompartment + 1; }
    public int getShooterCompartment() { return ((currentTargetCompartment + 1) % 3) + 1; }
    public int getNextUpCompartment() { return ((currentTargetCompartment + 2) % 3) + 1; }

    /**
     * Checks if the spindexer motor is currently moving to a target position.
     * @return true if the motor is busy, false if it has reached its target.
     */
    public boolean isBusy() {
        // Pass the call through to the underlying motor.
        return motor.isBusy();
    }
}
