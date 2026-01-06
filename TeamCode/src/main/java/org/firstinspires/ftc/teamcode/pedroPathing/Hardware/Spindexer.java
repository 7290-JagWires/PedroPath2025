package org.firstinspires.ftc.teamcode.pedroPathing.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Sensors.MagneticLimitSwitch;

public class Spindexer {

    private final SpindexerMotor motor;
    private final MagneticLimitSwitch limit;

    // constants

    public static final int COMPARTMENTS = 3;

    // 0 = Comp1, 1 = Comp2, 2 = Comp3
    private int compartmentIndex = 0;
    private int moveDirection = 0; // 1 for forward, -1 for backward, 0 for stopped

    // Lets TeleOp know a move happened (used for auto-close door)
    public boolean justIndexed = false;

    boolean lookingForLimitSwitch = false;
    // Add this enum at the top of your Spindexer class, with the other member variables.
    public enum HomingState {
        IDLE,    // Not doing anything
        START,   // The OpMode has requested homing to start
        HOMING,  // The motor is actively spinning, looking for the magnet
        COMPLETE // The magnet has been found and the position is zeroed
    }

    // Add this variable to track the current state.
    private HomingState homingState = HomingState.IDLE;

    public Spindexer(OpMode opMode) {
        // --- THIS IS THE ONLY CHANGE NEEDED IN THE CONSTRUCTOR ---
        // Before: motor = opMode.hardwareMap.get(DcMotorEx.class, "spindexer");
        // After:
        motor = new SpindexerMotor(opMode.hardwareMap); // <-- Use the new class

        limit = new MagneticLimitSwitch(opMode.hardwareMap, "magnetic_limit_sensor");
        // ... rest of constructor
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

    //Moved from original logic class
    public void updateMagnetZero() {
        if (limit.wasJustTriggered()) {
 //           resetEncoder();
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

    // In Spindexer.java

    public void home() {
        // Always update the sensor reading first
        limit.update();
        switch (homingState) {
            case IDLE:
                // No changes here
                break;
            case START:
                // --- CHANGE HERE ---
                // Use the motor's dedicated `setPower` method for manual control.
                motor.setPower(0.5); // Start the motor spinning slowly
                homingState = HomingState.HOMING;
                break;
            case HOMING:
                // --- CHANGE HERE ---
                if (limit.isTriggered()) {
                    // When the magnet is found:
                    // 1. Tell the SpindexerMotor to reset its internal encoder count to 0.
                    //    This also re-enables RUN_TO_POSITION mode to hold the new zero.
                    motor.resetEncoder();

                    // 2. Set the state to complete.
                    homingState = HomingState.COMPLETE;
                }
                break;
            case COMPLETE:
                // No changes needed here. The motor will automatically hold its zero position.
                break;
        }
    }

    /**
     * Checks if the homing process has been successfully completed.
     * @return true if the spindexer is homed, false otherwise.
     */
    public boolean isHomed() {
        return homingState == HomingState.COMPLETE;
    }

    /**
     * Returns the current state of the homing state machine. Useful for telemetry.
     * @return The current HomingState.
     */
    public HomingState getHomingState() {
        return homingState;
    }

    // Add this method to your Spindexer class.

    /**
     * Kicks off the homing sequence.
     * Call this from your OpMode's init_loop.
     */
    public void startHoming() {
        // Only start if we are idle (not already homing or complete)
        if (homingState == HomingState.IDLE) {
            homingState = HomingState.START;
        }
    }



}
