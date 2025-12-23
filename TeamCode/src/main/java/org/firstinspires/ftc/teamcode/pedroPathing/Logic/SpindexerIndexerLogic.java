package org.firstinspires.ftc.teamcode.pedroPathing.Logic;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.SpindexerMotor;
import org.firstinspires.ftc.teamcode.pedroPathing.Sensors.MagneticLimitSwitch;

public class SpindexerIndexerLogic {

    private final OpMode opMode;
    private final SpindexerMotor motor;
    private final MagneticLimitSwitch limit;

    private static final double INDEX_POWER = 0.75;

    private final int ticksPerCompartment;

    // 0 = Comp1, 1 = Comp2, 2 = Comp3
    private int compartmentIndex = 0;

    private int moveDirection = 0; // 1 for forward, -1 for backward, 0 for stopped

    // Lets TeleOp know a move happened (used for auto-close door)
    public boolean justIndexed = false;

    boolean limitSwitchTriggered = false;

    boolean lookingForLimitSwitch = false;



    public SpindexerIndexerLogic(OpMode opMode,
                                 SpindexerMotor motor,
                                 MagneticLimitSwitch limit,
                                 int ticksPerCompartment) {

        this.opMode = opMode;
        this.motor = motor;
        this.limit = limit;
        this.ticksPerCompartment = ticksPerCompartment;
    }

    public void setCompartment(int comp) {
        compartmentIndex = comp - 1;
        if (compartmentIndex < 0) compartmentIndex = 0;
        if (compartmentIndex > 2) compartmentIndex = 2;
    }

    public void updateMagnetZero() {
        if (limit.wasJustTriggered()) {
            motor.resetEncoder();
            compartmentIndex = 0;
        }
    }

    /**
     * The idea here is the limit switch can only be triggered after it has
     * been false. That way, the magnet sitting at the limit switch won't show
     * it continually triggered.
     *
     * @return <code>true</code> boolean if the switch is triggered <code>false</code> if not.
     *
     **/
    public boolean limitSwitchTriggered() {

        // We'll only return true once we've recorded a false reading
        if(lookingForLimitSwitch){
            opMode.telemetry.addLine("lookingForLimitSwitch");
            if(limit.isTriggered()){
                lookingForLimitSwitch = false;
                return true;
            }
        }
        // get the current state
        limitSwitchTriggered = limit.isTriggered();

        // if the limit switch is currently false
        if( !limitSwitchTriggered ){
            // we're looking for the next trigger
            lookingForLimitSwitch = true;
        }

        return false;
    }

    /**  Spindexer starts moving to the next compartment. You must also call
        spinderLimitSwitchCheck() in the main loop so it stops when the limit switch is triggered
    **/
    public void nextCompartment() {
        // I think the biggest problem was the motor was set to the RunToPosition mode
        // so setting the power didn't make it move. I added setModeRunUsingEncoder()
        moveDirection = 1; // Record we are moving forward
        motor.setModeRunUsingEncoder();
        motor.setPower(INDEX_POWER);
    }

    public void previousCompartment() {

        moveDirection = -1; // Record we are moving backward
        motor.setModeRunUsingEncoder();
        // reverse direction
        motor.setPower(-INDEX_POWER);
    }




//    /**
//     * This checks to see if the limit switch has been reached and stops the motor
//     * if it has.
//     * @return <code>true</code> if the limit switch has been reached and the motor stopped.
//     * <code>false</code> if not.
//     */
//    public boolean spindexerLimitSwitchCheck() {
//        if( limitSwitchTriggered()){
//            motor.stop();
//            return true;
//        }
//        return false;
//    }

    /**
     * This checks to see if the limit switch has been reached and stops the motor
     * if it has. It also updates the compartment index.
     */
    public boolean spindexerLimitSwitchCheck() {
        if (limitSwitchTriggered()) {
            motor.stop();
            justIndexed = true; // Signal that a move completed

            // Update index based on which way we were going
            if (moveDirection == 1) {
                // Moving Forward: Increment index (0 -> 1 -> 2 -> 0)
                compartmentIndex++;
                if (compartmentIndex > 2) {
                    compartmentIndex = 0;
                }
            } else if (moveDirection == -1) {
                // Moving Backward: Decrement index (0 -> 2 -> 1 -> 0)
                compartmentIndex--;
                if (compartmentIndex < 0) {
                    compartmentIndex = 2;
                }
            }

            // Reset direction since we are now stopped
            moveDirection = 0;

            return true;
        }
        return false;
    }


    public int getIntakeCompartment() { return compartmentIndex + 1; }
    public int getShooterCompartment() { return ((compartmentIndex + 1) % 3) + 1; }
    public int getNextUpCompartment() { return ((compartmentIndex + 2) % 3) + 1; }

    public void update() {
        limit.update();
        updateMagnetZero();
    }
}
