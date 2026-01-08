package org.firstinspires.ftc.teamcode.pedroPathing.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Logic.SpindexerIndexerLogic;
import org.firstinspires.ftc.teamcode.pedroPathing.Sensors.ColorSensor;
import org.firstinspires.ftc.teamcode.pedroPathing.Sensors.IndicatorLight;
import org.firstinspires.ftc.teamcode.pedroPathing.Sensors.MagneticLimitSwitch;
import org.firstinspires.ftc.teamcode.pedroPathing.Utilities;
import org.firstinspires.ftc.teamcode.pedroPathing.Utilities.DumpManager;
import org.firstinspires.ftc.teamcode.pedroPathing.Utilities.SpindexerRotator;
import org.firstinspires.ftc.teamcode.pedroPathing.Utilities.ManualShootManager;
import org.firstinspires.ftc.teamcode.pedroPathing.Utilities.TeleOpPickupManager;

public class Robot {
    public final OpMode opMode;
    // Subsystems
    public final DriveTrain driveTrain;
    public final MecanumDrive mecanumDrive;
    public final IntakeActive intakeActive;
    public final Shooter shooter;
    public final Door door;
    public final PinpointLocalizer pinpoint;

    public final ColorSensor colorSensor;
    public final IndicatorLight indicator;

    // Spindexer
    public final Spindexer spindexer;
    public final SpindexerMotor spindexerMotor;
    public final SpindexerIndexerLogic spindexerLogic;
    public final DumpManager dumpManager;
    public final SpindexerRotator spindexerRotator;
    public final ManualShootManager manualShootManager;
    public final MagneticLimitSwitch spindexerMag;
    public TeleOpPickupManager teleOpPickupManager;
    private static final int TICKS_PER_COMPARTMENT = 1354;//I would leave this in case we need to go back to encoder counting for compartments - AH

    public Robot(OpMode opMode) {
        this.opMode = opMode;

        driveTrain = new DriveTrain(opMode.hardwareMap);
        mecanumDrive = new MecanumDrive(driveTrain);
        intakeActive = new IntakeActive(opMode);
        shooter      = new Shooter(opMode);
        door         = new Door(opMode);
        spindexerMag   = new MagneticLimitSwitch(opMode.hardwareMap, "magnetic_limit_sensor");
        spindexer = new Spindexer(opMode);
        pinpoint = new PinpointLocalizer(opMode.hardwareMap);
        colorSensor = new ColorSensor(opMode.hardwareMap, "color_sensor");
        indicator = new IndicatorLight(opMode.hardwareMap, "rgbServo");
        spindexerMotor = new SpindexerMotor(opMode.hardwareMap);
        spindexerLogic = new SpindexerIndexerLogic(opMode, spindexerMotor, spindexerMag, TICKS_PER_COMPARTMENT);
        spindexerRotator = new SpindexerRotator(spindexerLogic);
        dumpManager = new DumpManager(opMode, shooter, door, spindexerLogic,spindexerRotator);
        manualShootManager = new ManualShootManager(door, spindexerLogic);
        teleOpPickupManager = new TeleOpPickupManager(door, intakeActive, spindexerLogic, colorSensor, indicator);
    }

    public void update() {
        pinpoint.update();
        spindexer.update();
        intakeActive.run();
        shooter.run();
        dumpManager.update();
        spindexerRotator.update();
    }

    public void stopAll() {
        mecanumDrive.stop();
    }
}
