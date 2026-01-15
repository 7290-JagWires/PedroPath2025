package org.firstinspires.ftc.teamcode.TeleOp.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.TeleOp.Drive.TeleOpMecanumDrive;
import org.firstinspires.ftc.teamcode.TeleOp.Sensors.TeleOpMagneticLimitSwitch;
import org.firstinspires.ftc.teamcode.TeleOp.Sensors.TeleOpPinpointLocalizer;

public class TeleOpRobot {

    public final LinearOpMode opMode;

    // Subsystems
    public final TeleOpDriveTrain driveTrain;
    public final TeleOpMecanumDrive mecanumDrive;
    public final TeleOpIntakeActive intakeActive;
    public final TeleOpShooter shooter;
    public final TeleOpDoor door;
    public final TeleOpPinpointLocalizer pinpoint;

    // Spindexer
    public final TeleOpSpindexerMotor spindexerMotor;
    public final TeleOpMagneticLimitSwitch spindexerMag;
    public final TeleOpSpindexerIndexerLogic spindexerLogic;
    private static final int TICKS_PER_COMPARTMENT = 1354;   // <— update with real measured value

    public TeleOpRobot(HardwareMap hardwareMap, LinearOpMode opMode) {
        this.opMode = opMode;

        // Hardware layer
        driveTrain = new TeleOpDriveTrain(hardwareMap);

        // Drive logic
        mecanumDrive = new TeleOpMecanumDrive(driveTrain);

        // Subsystems
        intakeActive = new TeleOpIntakeActive(hardwareMap, opMode);
        shooter      = new TeleOpShooter(hardwareMap, opMode);
        door         = new TeleOpDoor(hardwareMap, opMode);

        // Spindexer + magnetic sensor
        spindexerMotor = new TeleOpSpindexerMotor(hardwareMap);
        spindexerMag   = new TeleOpMagneticLimitSwitch(hardwareMap, "magnetic_limit_sensor");
        spindexerLogic = new TeleOpSpindexerIndexerLogic(
                opMode,
                spindexerMotor,
                spindexerMag,
                TICKS_PER_COMPARTMENT
        );

        // Odometry
        pinpoint = new TeleOpPinpointLocalizer(hardwareMap);
    }

    public void update() {
        pinpoint.update();
        spindexerLogic.update();

        intakeActive.run();
        shooter.run();
        door.run();
    }

    public void stopAll() {
        mecanumDrive.stop();
    }
}
