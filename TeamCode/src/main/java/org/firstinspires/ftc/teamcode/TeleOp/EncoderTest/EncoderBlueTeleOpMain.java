//package org.firstinspires.ftc.teamcode.TeleOp.EncoderTest;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode.TeleOp.Hardware.TeleOpBallColorClassifier;
//import org.firstinspires.ftc.teamcode.TeleOp.Sensors.TeleOpGoBildaRgbIndicator;
//
//
//
//@TeleOp(name = "EncoderBlueTeleOpMain", group = "Linear Opmode")
//
//public class EncoderBlueTeleOpMain extends LinearOpMode {
//
//    private static final int LAUNCH_DELAY_MILLISECONDS = 1000;
//
//    private int bluePipeline = 1;   // Blue Pipeline
//
//    private EncoderTeleOpRobot robot;
//
//    private boolean dumpMode = false;  // toggle for RT dump-all
//    private boolean prevRT = false;
//
//    private final ElapsedTime launchTimer = new ElapsedTime();
//
//    private ColorSensor colorSensor;
//    private TeleOpGoBildaRgbIndicator indicator;
//
//    // Button edge-detect
//    private boolean lastA = false;
//    private boolean lastB = false;
//
//    @Override
//    public void runOpMode() {
//
//        telemetry.addLine("Initializing...");
//        telemetry.update();
//
//        robot = new EncoderTeleOpRobot(hardwareMap, this);
//        robot.limelight.setPipeline(bluePipeline);
//        robot.limelight.start();
//
//        colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");
//        indicator = new TeleOpGoBildaRgbIndicator(hardwareMap, "rgbServo");
//
//        waitForStart();
//
//        telemetry.addLine("Spindexer: encoder mode (no magnet homing).");
//        telemetry.update();
//
//        int dumpCount = 0;
//
//        while (opModeIsActive()) {
//
//            // Limelight updates
//            robot.limelight.getAprilTagData();
//            //robot.limelight.updateShooterVelocityBasedOnDistance(robot);
//
//            // ------------ COLOR SENSOR ------------
//            int red = colorSensor.red();
//            int green = colorSensor.green();
//            int blue = colorSensor.blue();
//
//            double distanceCm = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
//
//            boolean ballPresent = distanceCm < 4.0 || colorSensor.alpha() > 150;
//
//            TeleOpBallColorClassifier.BallColor color =
//                    ballPresent
//                            ? TeleOpBallColorClassifier.classify(red, green, blue)
//                            : TeleOpBallColorClassifier.BallColor.UNKNOWN;
//
//            if (color == TeleOpBallColorClassifier.BallColor.PURPLE) {
//                indicator.setPurple();
//            } else if (color == TeleOpBallColorClassifier.BallColor.GREEN) {
//                indicator.setGreen();
//            } else {
//                indicator.off();
//            }
//
//            String detectedLabel = (color == TeleOpBallColorClassifier.BallColor.UNKNOWN)
//                    ? "No Ball"
//                    : color.toString();
//
//            telemetry.addData("Color (RGB)", "%d, %d, %d", red, green, blue);
//            telemetry.addData("Detected Color", detectedLabel);
//            telemetry.addData("Distance (cm)", "%.2f", distanceCm);
//            telemetry.addData("Ball Present", ballPresent);
//
//            // ===================== DUMP-ALL TOGGLE (RT) =====================
//            boolean rt = gamepad2.right_trigger > 0.6;
//            if (rt && !prevRT) {
//                dumpMode = !dumpMode;
//                if (dumpMode) {
//                    dumpCount = 0;
//                    launchTimer.reset();
//                }
//            }
//            prevRT = rt;
//
//            // --------------------------- DUMP MODE ---------------------------
//            if (dumpMode) {
//                robot.door.forceOpenLock();
//
//                if (dumpCount >= 3) {
//                    dumpMode = false;
//                    robot.door.unlock();
//                    robot.door.forceClose();
//                }
//            }
//
//            // -------------------------- DRIVER 1 DRIVE --------------------------
//            double y = -gamepad1.left_stick_y;
//            double x =  gamepad1.left_stick_x;
//            double r =  gamepad1.right_stick_x;
//
//            robot.mecanumDrive.drive(x, y, r);
//
//            // -------------------------- DRIVER 2 CONTROLS --------------------------
//            boolean aNow = gamepad2.a;
//            boolean bNow = gamepad2.b;
//
//            if (aNow && !lastA) {
//                robot.spindexer.nextCompartment();
//            }
//            if (bNow && !lastB) {
//                robot.spindexer.previousCompartment();
//            }
//
//            lastA = aNow;
//            lastB = bNow;
//
//            if (dumpMode && !robot.spindexer.isBusy()
//                    && launchTimer.milliseconds() > LAUNCH_DELAY_MILLISECONDS) {
//                robot.spindexer.nextCompartment();
//            }
//
//            // Update everything (spindexer.update() should be inside robot.update())
//            robot.update();
//
//            // Index complete event
//            if (robot.spindexer.justIndexed) {
//                robot.door.forceClose();
//
//                if (dumpMode) {
//                    dumpCount++;
//                    launchTimer.reset();
//                    telemetry.addLine("dumpCount: " + dumpCount);
//                }
//
//                robot.spindexer.justIndexed = false;
//            }
//
//            // --------------------------- TELEMETRY ---------------------------
//            telemetry.addData("Shooter RPM", "%.0f", robot.shooter.getRpm());
//            telemetry.addData("Spindexer Encoder", robot.spindexer.getCurrentPosition());
//            telemetry.addData("Spindexer Target", robot.spindexer.getTargetPosition());
//            telemetry.addData("Spindexer Busy", robot.spindexer.isBusy());
//
//            if (robot.limelight.result != null) {
//                telemetry.addData("Tag ID", robot.limelight.tagID);
//                telemetry.addData("Angle to Camera (deg) (Tx)", "%.2f", robot.limelight.tagXAngle);
//                telemetry.addData("Angle from camera to center of Target (Ty)", "%.2f", robot.limelight.tagYAngle);
//                telemetry.addData("Exact distance from camera to tag", "%.2f", robot.limelight.tagDistance);
//            } else {
//                telemetry.addLine("No tags in view");
//            }
//
//            telemetry.update();
//            idle();
//        }
//
//        robot.stopAll();
//    }
//}
