package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Door;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.IntakeActive;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.LimelightCamera;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Spindexer;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.SpindexerMotor;
import org.firstinspires.ftc.teamcode.pedroPathing.Logic.SpindexerIndexerLogic;
import org.firstinspires.ftc.teamcode.pedroPathing.Sensors.ColorSensor;
import org.firstinspires.ftc.teamcode.pedroPathing.Sensors.IndicatorLight;
import org.firstinspires.ftc.teamcode.pedroPathing.Sensors.MagneticLimitSwitch;
import org.firstinspires.ftc.teamcode.pedroPathing.Utilities.DumpManager;
import org.firstinspires.ftc.teamcode.pedroPathing.Utilities.PickupManager;
import org.firstinspires.ftc.teamcode.pedroPathing.Utilities.SpindexerRotator;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseStorage;

// This class cannot be run from the Driver Station because it is 'abstract'
public abstract class BaseAutonomous extends OpMode {

    // 1. All COMMON VARIABLES are moved here
    protected Follower follower;
    protected Timer pathTimer, opModeTimer;
    static final int STARTING_TAG_ID = 23;  //ROBOT LOADED PURPLE SHOOT, PURPLE UP NEXT, GREEN JUST LOADED COMPARTMENTS
    static final int PICKUP_ROW1_PPG = 23;  //ROBOT LOADED PURPLE SHOOT, PURPLE UP NEXT, GREEN JUST LOADED COMPARTMENTS
    static final int PICKUP_ROW2_PGP = 22;  //ROBOT LOADED PURPLE SHOOT, PURPLE UP NEXT, GREEN JUST LOADED COMPARTMENTS
    static final int PICKUP_ROW3_GPP = 21;  //ROBOT LOADED PURPLE SHOOT, PURPLE UP NEXT, GREEN JUST LOADED COMPARTMENTS
    static final double DOOR_TIMER_DELAY = 2;
    static final double PICKUP_MISSED_BALL_TIMER_DELAY = 2.75;

    private static final int TICKS_PER_COMPARTMENT = 1354;
    public boolean tagDetectionLogicHasRun = false;
    private int startingTagId = -1;

    // Hardware
    protected IntakeActive intakeActive;
    protected Shooter shooter;
    protected Door door;
    protected Spindexer spindexer;

    protected SpindexerMotor spindexerMotor;
    protected MagneticLimitSwitch spindexerMag;
    protected LimelightCamera limelight; // Make sure to add your LimelightCamera


    // Logic and Utility Managers
    protected SpindexerIndexerLogic spindexerLogic;
    protected PickupManager pickupManager;
    protected DumpManager dumpManager;
    protected SpindexerRotator spindexerRotator;

    protected ColorSensor colorSensor;
    protected IndicatorLight indicator;

    // State Machine
    public enum PathState {
        DRIVE_START_SCORE, DETECT_TAG_WHILE_DRIVING, WAIT_FOR_SPIN, SCORE_PRELOAD, SCORE1, SCORE2, SCORE3,
        DRIVE_PICKUP1, DRIVE_PICKUP1BALL2_END, DRIVE_PICKUP1BALL3_END, DRIVE_PICKUP1_END,
        DRIVE_PICKUP2, DRIVE_PICKUP2BALL2_END, DRIVE_PICKUP2BALL3_END, DRIVE_PICKUP2_END,
        DRIVE_PICKUP3, DRIVE_PICKUP3BALL2_END, DRIVE_PICKUP3BALL3_END, DRIVE_PICKUP3_END,
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
        tagDetectionLogicHasRun = false;

        // Initialize timers and follower
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        // Initialize all hardware
        intakeActive = new IntakeActive(this);
        shooter = new Shooter(this);
        door = new Door(this);
        colorSensor = new ColorSensor(hardwareMap, "color_sensor");
        indicator = new IndicatorLight(hardwareMap, "rgbServo");
        spindexerMotor = new SpindexerMotor(hardwareMap);
        spindexerMag = new MagneticLimitSwitch(hardwareMap, "magnetic_limit_sensor");
        limelight = new LimelightCamera(hardwareMap, "limelight");

        // Initialize logic and utility managers
        spindexerLogic = new SpindexerIndexerLogic(this, spindexerMotor, spindexerMag, TICKS_PER_COMPARTMENT);
        spindexerRotator = new SpindexerRotator(spindexerLogic);
        spindexer = new Spindexer(this, spindexerMag);

        dumpManager = new DumpManager(this, shooter, door, spindexerLogic,spindexerRotator);
        pickupManager = new PickupManager(door, shooter, intakeActive, spindexerLogic, colorSensor, indicator);
        limelight.setPipeline(0); // Make sure it's on your AprilTag pipeline
        limelight.start();

        // Call the abstract methods that the child class will define
        buildPaths();
        setStartingPose();
    }
    @Override
    public void init_loop() {
        startingTagId = limelight.getTagId();

        // Add live telemetry for the AprilTag ID
        telemetry.addData("Status", "Ready for Start");
        telemetry.addData("Detected Tag ID", startingTagId);
        telemetry.addLine("Point camera at tag to verify detection.");
        telemetry.update();
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
        spindexerRotator.update();
        pickupManager.update();
        autonomousPathUpdate(); // This calls the Blue or Red specific implementation

        // Common telemetry
        telemetry.addData("path state", pathState.toString());
        telemetry.addData("dump state", dumpManager.getState());
        telemetry.addData("pickup state", pickupManager.getState());
        telemetry.addData("Ball Count", pickupManager.getTotalBallCount());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Shooter Speed",shooter.getVelocity());


    }

    public void setPathState(PathState newState){
        pathState = newState;
        pathTimer.resetTimer();
    }
    public int getStartingTagId() {
        return startingTagId;
    }

    @Override
    public void stop() {
        // Get the robot's current pose from the follower
        Pose lastPose = follower.getPose();

        // Save this pose to a file using our utility class
        PoseStorage.savePoseToFile(hardwareMap, lastPose);
    }

    /**
     * Determines the required spindexer rotation based on the detected AprilTag
     * and starts the rotation. Assumes a specific preload position.
     *
     * @param preloadTagId The ID of the AprilTag that the spindexer is preloaded for.
     */
    public void startSpindexerRotationForTag(int preloadTagId) {
        // Get the tag ID that was detected during the init_loop
        int detectedTagId = getStartingTagId();

        // Failsafe: If no tag was ever seen during init, default to the preload tag.
        if (detectedTagId == -1) {
            detectedTagId = preloadTagId;
            telemetry.addLine("!!! No Tag Seen, Defaulting to Preload: " + preloadTagId + " !!!");
        }

        // Calculate how many rotations are needed.
        // This logic assumes tag IDs are sequential (e.g., 21, 22, 23).
        // The number of spins is the difference between the target and the preload position.
        int rotationsNeeded = detectedTagId - preloadTagId;

        // Start the spindexer rotation with the calculated number of steps.
        // If rotationsNeeded is 0, spindexerRotator.start(0) will do nothing, which is correct.
        spindexerRotator.start(rotationsNeeded);

        telemetry.addData("Preloaded for Tag", preloadTagId);
        telemetry.addData("Detected Tag", detectedTagId);
        telemetry.addData("Spindexer Rotations", rotationsNeeded);
    }
}
