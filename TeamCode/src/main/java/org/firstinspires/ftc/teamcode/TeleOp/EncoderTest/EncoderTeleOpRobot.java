//package org.firstinspires.ftc.teamcode.TeleOp.EncoderTest;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.teamcode.TeleOp.Drive.TeleOpMecanumDrive;
//import org.firstinspires.ftc.teamcode.TeleOp.Sensors.TeleOpPinpointLocalizer;
//import org.firstinspires.ftc.teamcode.TeleOp.Sensors.TelopLimelightCamera;
//
//// NOTE: These are your existing subsystem classes.
//// If any of these are NOT in this package, add imports for their real packages.
//import org.firstinspires.ftc.teamcode.TeleOp.Hardware.TeleOpDriveTrain;
//import org.firstinspires.ftc.teamcode.TeleOp.Hardware.TeleOpIntakeActive;
//import org.firstinspires.ftc.teamcode.TeleOp.Hardware.TeleOpShooter;
//import org.firstinspires.ftc.teamcode.TeleOp.Hardware.TeleOpDoor;
//
//public class EncoderTeleOpRobot {
//
//    public final LinearOpMode opMode;
//
//    // Subsystems
//    public final TeleOpDriveTrain driveTrain;
//    public final TeleOpMecanumDrive mecanumDrive;
//    public final TeleOpIntakeActive intakeActive;
//    public final TeleOpShooter shooter;
//    public final TeleOpDoor door;
//    public final TeleOpPinpointLocalizer pinpoint;
//
//    public final TelopLimelightCamera limelight;
//
//    // Spindexer (ENCODER ONLY)
//    public final EncoderTeleOpSpindexer spindexer;
//
//    public EncoderTeleOpRobot(HardwareMap hardwareMap, LinearOpMode opMode) {
//        this.opMode = opMode;
//
//        // Hardware layer
//        driveTrain = new TeleOpDriveTrain(hardwareMap);
//
//        // Drive logic
//        mecanumDrive = new TeleOpMecanumDrive(driveTrain);
//
//        // Subsystems
//        intakeActive = new TeleOpIntakeActive(hardwareMap, opMode);
//        shooter      = new TeleOpShooter(hardwareMap, opMode);
//        door         = new TeleOpDoor(hardwareMap, opMode);
//
//        // Spindexer (encoder)
//        spindexer = new EncoderTeleOpSpindexer(hardwareMap);
//
//        // Odometry
//        pinpoint = new TeleOpPinpointLocalizer(hardwareMap);
//
//        // Limelight
//        limelight = new TelopLimelightCamera(opMode.hardwareMap, "limelight");
//    }
//
//    public void update() {
//        pinpoint.update();
//
//        // Spindexer encoder update (edge-detect justIndexed)
//        spindexer.update();
//
//        intakeActive.run();
////        shooter.run();
//        door.run();
//    }
//
//    public void stopAll() {
//        mecanumDrive.stop();
//    }
//}
