package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Door;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.IntakeActive;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.SpindexerMotor;
import org.firstinspires.ftc.teamcode.pedroPathing.Logic.SpindexerIndexerLogic;
import org.firstinspires.ftc.teamcode.pedroPathing.Sensors.ColorSensor;
import org.firstinspires.ftc.teamcode.pedroPathing.Sensors.IndicatorLight;
import org.firstinspires.ftc.teamcode.pedroPathing.Sensors.MagneticLimitSwitch;
import org.firstinspires.ftc.teamcode.pedroPathing.Utilities.DumpManager;
import org.firstinspires.ftc.teamcode.pedroPathing.Utilities.PickupManager;

// This class cannot be run from the Driver Station because it is 'abstract'
public abstract class BaseAutonomous extends OpMode {

    // 1. All COMMON VARIABLES are moved here
    protected Follower follower;
    protected Timer pathTimer, opModeTimer;
    static final double DOOR_TIMER_DELAY = 2.75;
    private static final int TICKS_PER_COMPARTMENT = 1354;

    // Hardware
    protected IntakeActive intakeActive;
    protected Shooter shooter;
    protected Door door;
    protected SpindexerMotor spindexerMotor;
    protected MagneticLimitSwitch spindexerMag;

    // Logic and Utility Managers
    protected SpindexerIndexerLogic spindexerLogic;
    protected PickupManager pickupManager;
    protected DumpManager dumpManager;

    // State Machine
    public enum PathState {
        DRIVE_START_SCORE, SCORE_PRELOAD, SCORE1, SCORE2,
        DRIVE_PICKUP1, DRIVE_PICKUP1BALL2_END, DRIVE_PICKUP1BALL3_END, DRIVE_PICKUP1_END,
        DRIVE_PICKUP2, DRIVE_PICKUP2BALL2_END, DRIVE_PICKUP2BALL3_END, DRIVE_PICKUP2_END,
        END
    }
    protected PathState pathState;

    // 2. ABSTRACT METHODS: These must be implemented by the child classes (Blue/Red)
    protected abstract void buildPaths();
    protected abstract void setStartingPose();
    public abstract void autonomousPathUpdate();


    // 3. COMMON METHODS are implemented here
    @Override
    public void init() {
        // Set initial state
        pathState = PathState.DRIVE_START_SCORE;

        // Initialize timers and follower
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        // Initialize all hardware
        intakeActive = new IntakeActive(this);
        shooter = new Shooter(this);
        door = new Door(this);
        ColorSensor colorSensor = new ColorSensor(hardwareMap, "color_sensor");
        IndicatorLight indicator = new IndicatorLight(hardwareMap, "rgbServo");
        spindexerMotor = new SpindexerMotor(hardwareMap);
        spindexerMag = new MagneticLimitSwitch(hardwareMap, "magnetic_limit_sensor");

        // Initialize logic and utility managers
        spindexerLogic = new SpindexerIndexerLogic(this, spindexerMotor, spindexerMag, TICKS_PER_COMPARTMENT);
        dumpManager = new DumpManager(shooter, door, spindexerLogic);
        pickupManager = new PickupManager(door, shooter, intakeActive, spindexerLogic, colorSensor, indicator);

        // Call the abstract methods that the child class will define
        buildPaths();
        setStartingPose();
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        // Update all managers and state machines
        follower.update();
        dumpManager.update();
        pickupManager.update();
        autonomousPathUpdate(); // This calls the Blue or Red specific implementation

        // Common telemetry
        telemetry.addData("path state", pathState.toString());
        telemetry.addData("dump state", dumpManager.getState());
        telemetry.addData("pickup state", pickupManager.getState());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
    }

    public void setPathState(PathState newState){
        pathState = newState;
        pathTimer.resetTimer();
    }
}
