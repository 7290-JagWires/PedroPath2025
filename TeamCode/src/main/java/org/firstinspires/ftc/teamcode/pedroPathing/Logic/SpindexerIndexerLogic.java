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
    private int cumulativeTargetTicks = 0; // Tracks the absolute target position
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
    public void nextCompartment() {
        // Increment which logical compartment we are targeting.
        currentTargetCompartment = (currentTargetCompartment + 1) % SpindexerMotor.COMPARTMENTS;

        // Increment the ABSOLUTE target position by one compartment's worth of ticks.
        cumulativeTargetTicks += SpindexerMotor.TICKS_PER_COMPARTMENT;

        // Tell the motor to go to the new, always-increasing target.
        motor.setTargetTicks(cumulativeTargetTicks, 1); // Using 100% power

        // Signal that an indexing action has started.
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
