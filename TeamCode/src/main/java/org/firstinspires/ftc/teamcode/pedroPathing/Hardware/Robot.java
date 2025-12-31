package org.firstinspires.ftc.teamcode.pedroPathing.Hardware;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Door;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.IntakeActive;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Spindexer;
import org.firstinspires.ftc.teamcode.pedroPathing.Sensors.ColorSensor;
import org.firstinspires.ftc.teamcode.pedroPathing.Sensors.IndicatorLight;
import org.firstinspires.ftc.teamcode.pedroPathing.Sensors.MagneticLimitSwitch;

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
    public final MagneticLimitSwitch spindexerMag;
    private static final int TICKS_PER_COMPARTMENT = 1354;//I would leave this in case we need to go back to encoder counting for compartments - AH

    public Robot(OpMode opMode) {
        this.opMode = opMode;

        driveTrain = new DriveTrain(opMode.hardwareMap);
        mecanumDrive = new MecanumDrive(driveTrain);
        intakeActive = new IntakeActive(opMode);
        shooter      = new Shooter(opMode);
        door         = new Door(opMode);
        spindexerMag   = new MagneticLimitSwitch(opMode.hardwareMap, "magnetic_limit_sensor");
        spindexer = new Spindexer(opMode, spindexerMag);
        pinpoint = new PinpointLocalizer(opMode.hardwareMap);
        colorSensor = new ColorSensor(opMode.hardwareMap, "color_sensor");
        indicator = new IndicatorLight(opMode.hardwareMap, "rgbServo");
    }

    public void update() {
        pinpoint.update();
        spindexer.update();
        intakeActive.run();
        shooter.run();
        door.run();
    }

    public void stopAll() {
        mecanumDrive.stop();
    }
}
