package org.firstinspires.ftc.teamcode.TeleOp.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TeleOp.Hardware.TeleOpRobot;

@TeleOp(name = "TeleOpMain", group = "Linear Opmode")
public class TeleOpMain extends LinearOpMode {

    private static final int LAUNCH_DELAY_MILLISECONDS = 1000;

    private TeleOpRobot robot;

    private boolean dumpMode = false;  // toggle for RT dump-all
    private boolean prevRT = false;

    private ElapsedTime launchTimer = new ElapsedTime();

    @Override
    public void runOpMode() {

        telemetry.addLine("Initializing...");
        telemetry.update();

        robot = new TeleOpRobot(hardwareMap, this);

        // ---------------------- AUTO-HOME DURING INIT ----------------------
//        boolean homed = false;
//
//        while (opModeInInit()) {
//
//            if (!homed) {
//                robot.spindexerMotor.setModeRunUsingEncoder();
//                robot.spindexerMotor.setPower(0.18);
//
//                if (robot.spindexerMag.isTriggered()) {
//                    robot.spindexerMotor.stop();
//               //     robot.spindexerMotor.resetEncoder();
//               //     robot.spindexerLogic.setCompartment(1);
//                    homed = true;
//                }
//            }
//
//            telemetry.addData("Spindexer Homed?", homed);
//            telemetry.update();
//            idle();
//        }

        // --------------------------- PLAY STARTS ---------------------------
        //Wait for the driver to press play
        waitForStart();

        // ---------------------- AUTO-HOME AFTER START ----------------------
        telemetry.addLine("Homing Spindexer...");
        telemetry.update();

        robot.spindexerMotor.setModeRunUsingEncoder();
        robot.spindexerMotor.setPower(0.18);

        // Run until magnet is triggered or stop is pressed
        while (opModeIsActive() && !robot.spindexerMag.isTriggered()) {
            idle();
        }

        // Stop and reset
        robot.spindexerMotor.stop();

        telemetry.addLine("Homing Complete. Start Driving.");
        telemetry.update();

// --------------------------- MAIN LOOP --------------------------
        int dumpCount = 0;

        while (opModeIsActive()) {

            // ===================== DUMP-ALL TOGGLE (RT) =====================
            boolean rt = gamepad2.right_trigger > 0.6;
            if (rt && !prevRT) {
                dumpMode = !dumpMode;
                if(dumpMode){
                    dumpCount = 0;
                    // make sure we wait at least the delay needed to launch the ball before moving to the next compartment
                    launchTimer.reset();
                }
            }
            prevRT = rt;

            // --------------------------- DUMP MODE ---------------------------
            if (dumpMode) {

                // Keep door open the whole time
                robot.door.forceOpenLock();
                // Rotate through 3 compartments one time
                if( dumpCount >= 3 ){
                    // End dump mode
                    dumpMode = false;
                    robot.door.unlock();
                    robot.door.forceClose();
                }
            }

            // -------------------------- DRIVER 1 DRIVE --------------------------
            double y = -gamepad1.left_stick_y;  // forward/back
            double x =  gamepad1.left_stick_x;  // strafe
            double r =  gamepad1.right_stick_x; // rotate

            robot.mecanumDrive.drive(x, y, r);

            // -------------------------- DRIVER 2 CONTROLS --------------------------

            // Manual homing (DPAD UP)
//            if (gamepad2.dpad_up) {
//
//                robot.spindexerMotor.setModeRunUsingEncoder();
//                robot.spindexerMotor.setPower(0.4);
//
//                while (opModeIsActive() && !robot.spindexerMag.isTriggered()) {
//                    idle();
//                }
//
//                robot.spindexerMotor.stop();
//                robot.spindexerMotor.resetEncoder();
//                robot.spindexerLogic.setCompartment(1);
//            }

            // *** FIXED: A moves to next compartment ***
            if (gamepad2.a) {
                robot.spindexerLogic.nextCompartment();
            }



            // *** ADDED: B moves to previous compartment ***
            if (gamepad2.b) {                robot.spindexerLogic.previousCompartment();
            }


            // If we are dumping and we've waited at least the launch delay milliseconds, move to the next compartment
            if( dumpMode && launchTimer.milliseconds() > LAUNCH_DELAY_MILLISECONDS ){
                // move to the next compartment
                robot.spindexerLogic.nextCompartment();
            }

            // we have hit the limit switch so we're at the next compartment
            if( robot.spindexerLogic.spindexerLimitSwitchCheck()){
                // If we're in dumpmode...
                if(dumpMode){
                    // increment the dumpCount
                    dumpCount++;
                    // reset the timer so our call to:  robot.spindexerLogic.nextCompartment() won't
                    // happen until enough time has elapsed to let the ball fall into the shooter.
                    launchTimer.reset();
                    telemetry.addLine("dumpCount: " + dumpCount);
                }

                telemetry.addLine("hit limit switch");

            }

            robot.update();

            // Auto-close door after A-index
            if (robot.spindexerLogic.justIndexed) {
                robot.door.forceClose();
                robot.spindexerLogic.justIndexed = false;
            }

            // --------------------------- TELEMETRY ---------------------------
            //telemetry.addData("Dump Mode", dumpMode);
            telemetry.addData("Intake Comp", robot.spindexerLogic.getIntakeCompartment());
            telemetry.addData("Shooter Comp", robot.spindexerLogic.getShooterCompartment());
            telemetry.addData("Next Up", robot.spindexerLogic.getNextUpCompartment());
            telemetry.addData("Encoder", robot.spindexerMotor.getCurrentPosition());
            //telemetry.addData("Magnet", robot.spindexerMag.isTriggered());
            telemetry.update();

            idle();
        }

        robot.stopAll();
    }
}
