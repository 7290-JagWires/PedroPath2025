//package org.firstinspires.ftc.teamcode.TeleOp.EncoderTest;
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//public class EncoderTeleOpSpindexer {
//
//    private final DcMotorEx motor;
//
//    // ---------------- Constants ----------------
//    public static final int TICKS_PER_REV = 4063;   // keep your value
//    public static final int COMPARTMENTS = 3;
//    public static final int TICKS_PER_COMPARTMENT = TICKS_PER_REV / COMPARTMENTS;
//
//    private static final int POSITION_TOLERANCE_TICKS = 15; // adjust if needed
//
//    // ---------------- State ----------------
//    private double motorPower = 0.75;   // default
//    private int targetPosition = 0;     // encoder ticks target
//
//    // 0 = Comp1, 1 = Comp2, 2 = Comp3
//    private int compartmentIndex = 0;
//
//    // Lets TeleOp know a move happened (used for auto-close door)
//    public boolean justIndexed = false;
//
//    // Used to edge-detect “move finished”
//    private boolean wasBusyLast = false;
//
//    public EncoderTeleOpSpindexer(HardwareMap hwMap) {
//        motor = hwMap.get(DcMotorEx.class, "spindexer");
//
//        motor.setDirection(DcMotorSimple.Direction.REVERSE);
//        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        motor.setTargetPosition(0);
//        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motor.setPower(0.0);
//
//        motor.setTargetPositionTolerance(POSITION_TOLERANCE_TICKS);
//
//        compartmentIndex = 0;
//        targetPosition = 0;
//    }
//
//    // ---------------------------------------------------------
//    // Public controls
//    // ---------------------------------------------------------
//
//    /** Adjust default power used for indexing (0..1). */
//    public void setMotorPower(double power) {
//        motorPower = clamp(power, 0.0, 1.0);
//    }
//
//    /** Immediately stop motor power (does not change targetPosition). */
//    public void stop() {
//        motor.setPower(0.0);
//    }
//
//    /** Hard reset encoder position to 0 (use only if you are physically at Comp1). */
//    public void resetEncoderToZero() {
//        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motor.setTargetPosition(0);
//        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        targetPosition = 0;
//        compartmentIndex = 0;
//
//        motor.setPower(0.0);
//        justIndexed = false;
//        wasBusyLast = false;
//    }
//
//    /** Move forward one compartment (wraps 1->2->3->1). */
//    public void nextCompartment() {
//        if (isBusy()) return;
//
//        compartmentIndex = wrapIndex(compartmentIndex + 1);
//        targetPosition += TICKS_PER_COMPARTMENT;
//
//        applyTarget();
//    }
//
//    /** Move backward one compartment (wraps 1<-2<-3<-1). */
//    public void previousCompartment() {
//        if (isBusy()) return;
//
//        compartmentIndex = wrapIndex(compartmentIndex - 1);
//        targetPosition -= TICKS_PER_COMPARTMENT;
//
//        applyTarget();
//    }
//
//    /** Go to compartment 1..3 using the shortest direction (optional helper). */
//    public void goToCompartment(int compartment1to3) {
//        if (isBusy()) return;
//
//        int desired = clamp(compartment1to3, 1, 3) - 1; // to 0..2
//        int current = compartmentIndex;
//
//        int forwardSteps  = (desired - current + COMPARTMENTS) % COMPARTMENTS;  // 0..2
//        int backwardSteps = (current - desired + COMPARTMENTS) % COMPARTMENTS; // 0..2
//
//        compartmentIndex = desired;
//
//        if (forwardSteps <= backwardSteps) {
//            targetPosition += forwardSteps * TICKS_PER_COMPARTMENT;
//        } else {
//            targetPosition -= backwardSteps * TICKS_PER_COMPARTMENT;
//        }
//
//        applyTarget();
//    }
//
//    /**
//     * Call every loop. Handles justIndexed edge detect when a move finishes.
//     */
//    public void update() {
//        justIndexed = false;
//
//        boolean busy = isBusy();
//
//        if (wasBusyLast && !busy) {
//            justIndexed = true;
//            compartmentIndex = getCompartmentIndexFromEncoder();
//        }
//
//        wasBusyLast = busy;
//    }
//
//    // ---------------------------------------------------------
//    // Telemetry helpers / getters
//    // ---------------------------------------------------------
//
//    public boolean isBusy() {
//        return motor.isBusy();
//    }
//
//    public int getCurrentPosition() {
//        return motor.getCurrentPosition();
//    }
//
//    public int getTargetPosition() {
//        return targetPosition;
//    }
//
//    /** 1..3 */
//    public int getIntakeCompartment() { return compartmentIndex + 1; }
//    public int getShooterCompartment() { return ((compartmentIndex + 1) % 3) + 1; }
//    public int getNextUpCompartment() { return ((compartmentIndex + 2) % 3) + 1; }
//
//    // ---------------------------------------------------------
//    // Internal helpers
//    // ---------------------------------------------------------
//
//    private void applyTarget() {
//        motor.setTargetPosition(targetPosition);
//        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motor.setPower(motorPower);
//    }
//
//    private int getCompartmentIndexFromEncoder() {
//        int pos = motor.getCurrentPosition();
//        int idx = Math.round((float) pos / (float) TICKS_PER_COMPARTMENT);
//        return wrapIndex(idx);
//    }
//
//    private int wrapIndex(int idx) {
//        int m = idx % COMPARTMENTS;
//        if (m < 0) m += COMPARTMENTS;
//        return m;
//    }
//
//    private static double clamp(double v, double min, double max) {
//        return Math.max(min, Math.min(max, v));
//    }
//
//    private static int clamp(int v, int min, int max) {
//        return Math.max(min, Math.min(max, v));
//    }
//}
