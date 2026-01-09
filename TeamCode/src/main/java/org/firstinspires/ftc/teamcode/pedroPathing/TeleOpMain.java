//package org.firstinspires.ftc.teamcode.pedroPathing;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Robot;
//import org.firstinspires.ftc.teamcode.pedroPathing.Sensors.ColorSensor;
//import org.firstinspires.ftc.teamcode.pedroPathing.Sensors.IndicatorLight;
//
//@TeleOp(name = "TeleOpMain", group = "Linear Opmode")
//public class TeleOpMain extends LinearOpMode {
//
//    private static final int LAUNCH_DELAY_MILLISECONDS = 1000;
//
//    private Robot robot;
//
//    private boolean dumpMode = false;  // toggle for RT dump-all
//    private boolean prevRT = false;
//
//    private ElapsedTime launchTimer = new ElapsedTime();
//
//    private ColorSensor colorSensor;
//
//    private IndicatorLight indicator;
//
//
//    @Override
//    public void runOpMode() {
//
//        telemetry.addLine("Initializing...");
//        telemetry.update();
//
//        robot = new Robot(this);
//
//        colorSensor = new ColorSensor(hardwareMap, "color_sensor");
//
//        indicator = new IndicatorLight(hardwareMap, "rgbServo");
//
//        // --------------------------- PLAY STARTS ---------------------------
//        //Wait for the driver to press play
//        waitForStart();
//
//        // ---------------------- AUTO-HOME AFTER START ----------------------
//        telemetry.addLine("Homing Spindexer...");
//        telemetry.update();
//
//        robot.spindexer.setModeRunUsingEncoder();
//        robot.spindexer.setPower(0.5);
//
//        // Run until magnet is triggered or stop is pressed
//        while (opModeIsActive() && !robot.spindexerMag.isTriggered()) {
//            idle();
//        }
//
//        // Stop and reset
//        robot.spindexer.stop();
//
//        telemetry.addLine("Homing Complete. Start Driving.");
//        telemetry.update();
//
//        // --------------------------- MAIN LOOP --------------------------
//        int dumpCount = 0;
//
//        while (opModeIsActive()) {
//
//            // ------------ READ COLOR SENSOR EACH LOOP ------------
//
//            colorSensor.update();
//            ColorSensor.BallColor color = colorSensor.getBallColor();
//
//            if (color == ColorSensor.BallColor.PURPLE) {
//                indicator.setPurple();
//            }
//            else if (color == ColorSensor.BallColor.GREEN) {
//                indicator.setGreen();
//            }
//            else  {
//                indicator.off();
//            }
//
//            String detectedLabel = (color == ColorSensor.BallColor.UNKNOWN)
//                    ? "No Ball"
//                    : color.toString();
//
//            //telemetry.addData("Color (RGB)", "%d, %d, %d", colorSensor.getRed(), colorSensor.getGreen(),colorSensor.getBlue();
//            telemetry.addData("Detected Color", colorSensor.getBallColor());
//            telemetry.addData("Distance (cm)", "%.2f", colorSensor.getDistanceCm());
//            telemetry.addData("Ball Present", colorSensor.isBallPresent());
//
//
//            // ===================== DUMP-ALL TOGGLE (RT) =====================
//            boolean rt = gamepad2.right_trigger > 0.6;
//            if (rt && !prevRT) {
//                dumpMode = !dumpMode;
//                if(dumpMode){
//                    dumpCount = 0;
//                    // make sure we wait at least the delay needed to launch the ball before moving to the next compartment
//                    launchTimer.reset();
//                }
//            }
//            prevRT = rt;
//
//            // --------------------------- DUMP MODE ---------------------------
//            if (dumpMode) {
//                // Keep door open the whole time
//                robot.door.forceOpenLock();
//                // Rotate through 3 compartments one time
//                if( dumpCount >= 3 ){
//                    // End dump mode
//                    dumpMode = false;
//                    robot.door.unlock();
//                    robot.door.forceClose();
//                }
//            }
//
//            // -------------------------- DRIVER 1 DRIVE --------------------------
//            double y = -gamepad1.left_stick_y;  // forward/back
//            double x =  gamepad1.left_stick_x;  // strafe
//            double r =  gamepad1.right_stick_x; // rotate
//
//            robot.mecanumDrive.drive(x, y, r);
//
//            // -------------------------- DRIVER 2 CONTROLS --------------------------
//
//            // *** FIXED: A moves to next compartment ***
//            if (gamepad2.a) {
//                robot.spindexer.nextCompartment();
//            }
//
//            // *** ADDED: B moves to previous compartment ***
//            if (gamepad2.b) {
//                robot.spindexer.previousCompartment();
//            }
//
//            // If we are dumping and we've waited at least the launch delay milliseconds, move to the next compartment
//            if( dumpMode && launchTimer.milliseconds() > LAUNCH_DELAY_MILLISECONDS ){
//                // move to the next compartment
//                robot.spindexer.nextCompartment();
//            }
//
//            //this should not be needed anymore with new combined spindexer but left just in case - AH
////            // we have hit the limit switch so we're at the next compartment
////            if( robot.spindexer.spindexerLimitSwitchCheck()){
////                // If we're in dumpmode...
////                if(dumpMode){
////                    // increment the dumpCount
////                    dumpCount++;
////                    // reset the timer so our call to:  robot.spindexerLogic.nextCompartment() won't
////                    // happen until enough time has elapsed to let the ball fall into the shooter.
////                    launchTimer.reset();
////                    telemetry.addLine("dumpCount: " + dumpCount);
////                }
////                telemetry.addLine("hit limit switch");
////            }
//
//            robot.update();
//
//            // Auto-close door after A-index
//            if (robot.spindexer.justIndexed) {
//                robot.door.forceClose();
//                robot.spindexer.justIndexed = false;
//            }
//
//
//
//            // --------------------------- TELEMETRY ---------------------------
//            telemetry.addData("Dump Mode", dumpMode);
//            telemetry.addData("Intake Comp", robot.spindexer.getIntakeCompartment());
//            telemetry.addData("Shooter Comp", robot.spindexer.getShooterCompartment());
//            telemetry.addData("Next Up", robot.spindexer.getNextUpCompartment());
//            telemetry.addData("Magnet", robot.spindexerMag.isTriggered());
//            telemetry.update();
//
//
//            idle();
//        } // <--- END OF WHILE LOOP
//
//        robot.stopAll();
//    }
//}
