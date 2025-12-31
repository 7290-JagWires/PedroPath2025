package org.firstinspires.ftc.teamcode.pedroPathing.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Sensors.MagneticLimitSwitch;

public class Spindexer {

    private final DcMotorEx motor;
    private final MagneticLimitSwitch limit;

    // constants

    public static final int COMPARTMENTS = 3;
    public static final int TICKS_PER_REV = 4063;
    public static final int TICKS_PER_COMPARTMENT = TICKS_PER_REV / COMPARTMENTS;

    private static final double INDEX_POWER = 0.50;


    // 0 = Comp1, 1 = Comp2, 2 = Comp3
    private int compartmentIndex = 0;
    private int moveDirection = 0; // 1 for forward, -1 for backward, 0 for stopped

    // Lets TeleOp know a move happened (used for auto-close door)
    public boolean justIndexed = false;

   // boolean limitSwitchTriggered = false;

    boolean lookingForLimitSwitch = false;

    private void resetEncoder() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void runUsingEncoder() {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }



    public Spindexer(OpMode opMode, MagneticLimitSwitch limit) {
        motor = opMode.hardwareMap.get(DcMotorEx.class, "spindexer");
        this.limit = limit;


        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void setModeRunUsingEncoder() {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**  Spindexer starts moving to the next compartment. You must also call
     spinderLimitSwitchCheck() in the main loop so it stops when the limit switch is triggered
     **/
    public void nextCompartment() {
        // I think the biggest problem was the motor was set to the RunToPosition mode
        // so setting the power didn't make it move. I added setModeRunUsingEncoder()
        moveDirection = 1; // Record we are moving forward
        runUsingEncoder();
        motor.setPower(INDEX_POWER);
    }

    public void previousCompartment() {

        moveDirection = -1; // Record we are moving backward
        runUsingEncoder();;
        // reverse direction
        motor.setPower(-INDEX_POWER);
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

    public void setPower(double power) {
        motor.setPower(power);
    }


    //Moved from original logic class
    public void updateMagnetZero() {
        if (limit.wasJustTriggered()) {
            resetEncoder();
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

    //********
//
//    /** Returns current compartment index: 0, 1, or 2 */
//    public int getCompartment() {
//        int index = getCurrentPosition() / TICKS_PER_COMPARTMENT;
//        return index % COMPARTMENTS;
//    }
//
//    /** Returns compartment as 1, 2, or 3 for telemetry */
//    public int getCompartmentDisplay() {
//        return getCompartment() + 1;
//    }
//
//
//    public DcMotorEx getMotor() {
//        return motor;
//    }
//
//    public void setCompartment(int comp) {
//        compartmentIndex = comp - 1;
//        if (compartmentIndex < 0) compartmentIndex = 0;
//        if (compartmentIndex > 2) compartmentIndex = 2;
//    }
//
//
//    /**
//     * The idea here is the limit switch can only be triggered after it has
//     * been false. That way, the magnet sitting at the limit switch won't show
//     * it continually triggered.
//     *
//     * @return <code>true</code> boolean if the switch is triggered <code>false</code> if not.
//     *
//     **/
//    public boolean limitSwitchTriggered() {
//
//        // We'll only return true once we've recorded a false reading
//        if(lookingForLimitSwitch){
//            opMode.telemetry.addLine("lookingForLimitSwitch");
//            if(limit.isTriggered()){
//                lookingForLimitSwitch = false;
//                return true;
//            }
//        }
//        // get the current state
//        limitSwitchTriggered = limit.isTriggered();
//
//        // if the limit switch is currently false
//        if( !limitSwitchTriggered ){
//            // we're looking for the next trigger
//            lookingForLimitSwitch = true;
//        }
//
//        return false;
//    }
//
//    /**
//     * This checks to see if the limit switch has been reached and stops the motor
//     * if it has. It also updates the compartment index.
//     */
//    public boolean spindexerLimitSwitchCheck() {
//        if (limitSwitchTriggered()) {
//            motor.stop();
//            justIndexed = true; // Signal that a move completed
//
//            // Update index based on which way we were going
//            if (moveDirection == 1) {
//                // Moving Forward: Increment index (0 -> 1 -> 2 -> 0)
//                compartmentIndex++;
//                if (compartmentIndex > 2) {
//                    compartmentIndex = 0;
//                }
//            } else if (moveDirection == -1) {
//                // Moving Backward: Decrement index (0 -> 2 -> 1 -> 0)
//                compartmentIndex--;
//                if (compartmentIndex < 0) {
//                    compartmentIndex = 2;
//                }
//            }
//
//            // Reset direction since we are now stopped
//            moveDirection = 0;
//
//            return true;
//        }
//        return false;
//    }


}
