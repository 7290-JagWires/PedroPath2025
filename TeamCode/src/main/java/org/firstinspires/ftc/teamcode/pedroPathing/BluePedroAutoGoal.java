package org.firstinspires.ftc.teamcode.pedroPathing;

import static java.lang.Thread.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Door;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.IntakeActive;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.SpindexerMotor;
import org.firstinspires.ftc.teamcode.pedroPathing.Logic.SpindexerIndexerLogic;
import org.firstinspires.ftc.teamcode.pedroPathing.Sensors.MagneticLimitSwitch;

@Autonomous(name = "Blue Auto Goal", group = "Pedro")
public class BluePedroAutoGoal extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;
    ElapsedTime launchTimer = new ElapsedTime();
    ElapsedTime dumpTimer = new ElapsedTime();
    boolean isDumping = false;
    int dumpCount = 0;
    static final int TOTAL_DUMPS = 3;
    static final int SHOOTER_SPINUP_MS = 1500;
    static final int LAUNCH_DELAY_MS = 500;
    private static final int LAUNCH_DELAY_MILLISECONDS = 1500;
    private static final int TICKS_PER_COMPARTMENT = 1354;   // <— update with real measured value
    public IntakeActive intakeActive;
    public Shooter shooter;
    public Door door;
    public SpindexerMotor spindexerMotor;
    public MagneticLimitSwitch spindexerMag;
    public SpindexerIndexerLogic spindexerLogic;

    /* ------ init Poses --------
    Pedro is based on a 0-144 grid, the same size as the FTC field in inches.
    0,0 is bottom left corner. each coordinate is 1 inch over.
    remember, pedro assumes you start at 0,0 CENTRE of your ROBOT
    initial heading (0) is facing towards goals
    */
    /*
    Think of Poses like a set position on the field. we have several "important" positions to keep track of here
    so we write them down here.
     */
    private final Pose startPose = new Pose(17, 115, Math.toRadians(25));
    private final Pose shootPoint = new Pose(59, 85, Math.toRadians(155));
    private final Pose pickup1Pose = new Pose(42, 83, Math.toRadians(180));
    private final Pose endPickup1Pose = new Pose(23, 83, Math.toRadians(180));
    private final Pose pickup2Pose = new Pose(42, 60, Math.toRadians(180));
    private final Pose endPickup2Pose = new Pose(23, 60, Math.toRadians(180));


    // ---------- Paths--------
    // these are individual names for each of our paths that we will eventually follow/create
    private PathChain scorePreload, pickup1, endPickup1, score1, pickup2, endPickup2, score2, endPoint;

    public enum PathState {
        // These are the various states inside of our auto machine.
        DRIVE_START_SCORE,
        SCORE_PRELOAD,
        SCORE1,
        SCORE2,
        DRIVE_PICKUP1,
        DRIVE_PICKUP1_END,
        DRIVE_PICKUP2,
        DRIVE_PICKUP2_END,
        END
    };
    PathState pathState;
    public enum DumpState {
        IDLE,
        SPIN_UP,
        DUMPING,
        FINISHED
    }
    DumpState dumpState;

    /**
     * Constructs the various autonomous paths the robot will follow.
     * This method initializes all the {@link PathChain} objects used in the autonomous routine.
     * It uses a {@link com.pedropathing.follower.PathBuilder} to define a sequence of movements
     * between predefined {@link Pose} coordinates on the field.
     * <p>
     * Paths are built for the following actions:
     * <ul>
     *     <li>{@code scorePreload}: Move from the starting position to the shooting point.</li>
     *     <li>{@code pickup1} & {@code endPickup1}: Move from the shooting point to the first pickup location.</li>
     *     <li>{@code score1}: Return from the first pickup location to the shooting point.</li>
     *     <li>{@code pickup2} & {@code endPickup2}: Move from the shooting point to the second pickup location.</li>
     *     <li>{@code score2}: Return from the second pickup location to the shooting point.</li>
     *     <li>{@code endPoint}: A final parking path.</li>
     * </ul>
     * Each path segment uses {@link BezierLine} for movement and {@code setLinearHeadingInterpolation}
     * to control the robot's orientation during the path. This method should be called during initialization
     * after the {@code follower} has been created.
     */
    public void buildPaths() {
        endPoint = follower.pathBuilder() // returns the robot pickup1Pose to end off shooting line
                .addPath(new BezierLine(shootPoint, pickup1Pose))
                .setLinearHeadingInterpolation(shootPoint.getHeading(), pickup1Pose.getHeading())
                .build();
        scorePreload = follower.pathBuilder() // moves from from start > scoring position
                .addPath(new BezierLine(startPose, shootPoint))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPoint.getHeading())
                .build();
        pickup1 = follower.pathBuilder() // moves from scoring position  > pickup 1 point, 1/2 speed
                .addPath(new BezierLine(shootPoint, pickup1Pose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), pickup1Pose.getHeading())
               .build();
        endPickup1 = follower.pathBuilder() // moves from scoring position  > pickup 1 point, 1/2 speed
                .addPath(new BezierLine(pickup1Pose, endPickup1Pose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), pickup1Pose.getHeading())
                .build();
        score1 = follower.pathBuilder() // moves from pickup 1 > scoring position
                .addPath(new BezierLine(endPickup1Pose, shootPoint))
                .setLinearHeadingInterpolation(endPickup1Pose.getHeading(), shootPoint.getHeading())
                .build();
        pickup2 = follower.pathBuilder() // moves from scoring position > pickup 2
                // use a curve to we can line up better for the balls
                .addPath(new BezierLine(shootPoint, pickup2Pose))
                .setLinearHeadingInterpolation( pickup2Pose.getHeading(), pickup2Pose.getHeading())
                .build();
        endPickup2 = follower.pathBuilder() // moves from scoring position > pickup 2
                // use a curve to we can line up better for the balls
                .addPath(new BezierLine(pickup2Pose, endPickup2Pose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), endPickup2Pose.getHeading())
                .build();
        score2 = follower.pathBuilder() // moves from pickup 2 > scoring position
                .addPath(new BezierLine(endPickup2Pose, shootPoint))
                .setLinearHeadingInterpolation(endPickup2Pose.getHeading(), shootPoint.getHeading())
                .build();
        }

    /**
     * Manages the robot's movement and actions during the autonomous period using a state machine.
     * This method is called repeatedly in the {@code loop()} method to progress the robot through
     * a sequence of predefined paths and actions. The state machine transitions between driving to
     * different locations (like the scoring position and pickup zones) and executing tasks such as
     * scoring balls. The current state is tracked by the {@code pathState} variable.
     *
     * <p>The states handle the following logic:
     * <ul>
     *     <li><b>DRIVE_START_SCORE:</b> Initiates the first move from the start pose to the shooting position.</li>
     *     <li><b>SCORE_PRELOAD:</b> Executes the scoring routine for the preloaded balls. Once finished, it transitions to drive to the first pickup location.</li>
     *     <li><b>DRIVE_PICKUP1 & DRIVE_PICKUP1_END:</b> Navigates the robot to the first set of balls on the field.</li>
     *     <li><b>SCORE1:</b> Scores the balls collected from the first pickup. Transitions to drive to the second pickup location.</li>
     *     <li><b>DRIVE_PICKUP2 & DRIVE_PICKUP2_END:</b> Navigates the robot to the second set of balls.</li>
     *     <li><b>SCORE2:</b> Scores the balls collected from the second pickup. Transitions to the final path.</li>
     *     <li><b>END:</b> Drives to a final parking position and concludes the autonomous routine.</li>
     * </ul>
     * Each state transition is typically triggered by the completion of a path (checked with {@code follower.isBusy()})
     * or the completion of an action like dumping balls.
     */
    public void autonomousPathUpdate() {
        switch(pathState) {
            case SCORE_PRELOAD:
                dumpItemsSM();

                if (dumpState == DumpState.FINISHED) {
                    startDump();
                    if (pathTimer.getElapsedTimeSeconds() > 4 && !follower.isBusy()) {
                        // just scored preload, drive to pickup point 1
                        follower.followPath(pickup1);
                        setPathState(PathState.DRIVE_PICKUP1);
                    }
                }
                break;
            case SCORE1:
                dumpItemsSM();

                if (dumpState == DumpState.FINISHED) {
                    startDump();
                    if(pathTimer.getElapsedTimeSeconds() > 5 && !follower.isBusy()) {
                        // scored pickup 1, drive to pickup 2
                        follower.followPath(pickup2);
                        follower.setMaxPower(1);
                        setPathState(PathState.DRIVE_PICKUP2);
                    }
                }
                break;
            case SCORE2:
                dumpItemsSM();

                if (dumpState == DumpState.FINISHED) {
                    shooter.stop();         //Turn off the shooter when we finish auto
                    if(pathTimer.getElapsedTimeSeconds() > 6 && !follower.isBusy()) {
                        // end state machine
                        follower.followPath(endPoint);
                        follower.setMaxPower(1);
                        setPathState(PathState.END);
                    }
                }
                break;
            case DRIVE_START_SCORE:
                startDump();

                // drive from start to scoring position
                follower.followPath(scorePreload, true);
                follower.setMaxPower(1);
                setPathState(PathState.SCORE_PRELOAD);

                break;
            case DRIVE_PICKUP1_END:
                if(!follower.isBusy()) {
                    // picked up at spike 1. drive from pickup1 to score
                    follower.followPath(score1, true);
                    follower.setMaxPower(1);
                    pathState = PathState.SCORE1;
                }
                break;
            case DRIVE_PICKUP2_END:
                if(!follower.isBusy()) {
                    // picked up at spike 2. drive from pickup 2 to score
                    follower.followPath(score2, true);
                    follower.setMaxPower(1);
                    pathState = PathState.SCORE2;
                }
                break;
            case DRIVE_PICKUP1:
                // scored. drive to pickup point 1
                if(!follower.isBusy()) {
                    follower.followPath(endPickup1);
                    follower.setMaxPower(.25);
                    pathState = PathState.DRIVE_PICKUP1_END;
                }
                break;
            case DRIVE_PICKUP2:
                // scored, drive to pickup point 2
                if(!follower.isBusy()) {
                    follower.followPath(endPickup2);
                    follower.setMaxPower(.25);
                    pathState = PathState.DRIVE_PICKUP2_END;
                }
                break;

            default:
                break;
        }
    }

    /**
     * Sets the current state of the autonomous pathing state machine and resets the path timer.
     * This method should be called whenever transitioning from one path segment or action to another
     * to ensure the timer for the new state starts at zero.
     *
     * @param newState The {@link PathState} to transition to.
     */
    public void setPathState(PathState newState){
        pathState = newState;
        pathTimer.resetTimer();
    }
    @Override

    public void init() {
        // set initial state
        pathState = PathState.DRIVE_START_SCORE;
        dumpState = DumpState.IDLE;

        pathTimer = new Timer();
        opModeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);

        //TODO: add in any other Init statements here for other hardware, shooters, etc.
        intakeActive = new IntakeActive(this);
        shooter      = new Shooter(this);
        door         = new Door(this);

        // Spindexer + magnetic sensor
        spindexerMotor = new SpindexerMotor(hardwareMap);
        spindexerMag   = new MagneticLimitSwitch(hardwareMap, "magnetic_limit_sensor");
        spindexerLogic = new SpindexerIndexerLogic(
                this,
                spindexerMotor,
                spindexerMag,
                TICKS_PER_COMPARTMENT
        );

        buildPaths();
        follower.setStartingPose(startPose);
    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState); // update state machine with starting state
    }

    @Override
    public void loop() {
        // update state machine + odometry
        follower.update();
        autonomousPathUpdate();

        // give data back to drivers
        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());
    }

    /**
     * Initiates the automated process of dumping all 3 balls into the goal.
     * This method sets the dump state to {@code SPIN_UP}, resets the dump counter and timer,
     * and starts the shooter motor to bring it up to speed. The actual dumping logic is
     * handled by the {@code dumpItemsSM()} state machine, which is called subsequently
     * in the main loop.
     */
    public void startDump() {
        dumpState = DumpState.SPIN_UP;
        dumpCount = 0;
        dumpTimer.reset();
        shooter.run();
    }

    /**
     * Manages the state machine for dumping balls into the goal.
     * This method should be called repeatedly in the main loop to progress through the states.
     * <p>
     * The process follows these states:
     * <ul>
     *     <li><b>IDLE:</b> The default state, does nothing.</li>
     *     <li><b>SPIN_UP:</b> Waits for the shooter motor to reach the required speed.</li>
     *     <li><b>DUMPING:</b> Opens the door and cycles through the spindexer compartments to launch items one by one. It continues until the target number of items (TOTAL_DUMPS) has been launched.</li>
     *     <li><b>FINISHED:</b> The dumping sequence is complete. It closes the door, unlocks it, and stops the shooter motor.</li>
     * </ul>
     */
    public void dumpItemsSM() {

        switch (dumpState) {
            case IDLE:
                break;
            case SPIN_UP:
                if (dumpTimer.milliseconds() >= SHOOTER_SPINUP_MS) {
                    launchTimer.reset();
                    dumpState = DumpState.DUMPING;
                }
                break;
            case DUMPING:
                door.forceOpenLock();
                if (launchTimer.milliseconds() > LAUNCH_DELAY_MS) {
                    spindexerLogic.nextCompartment();
                }
                if (spindexerLogic.spindexerLimitSwitchCheck()) {
                    dumpCount++;
                    launchTimer.reset();

                    telemetry.addData("Dump Count", dumpCount);
                    if (dumpCount >= TOTAL_DUMPS) {
                        dumpState = DumpState.FINISHED;
                    }
                }
                break;
            case FINISHED:
                door.unlock();
                door.forceClose();
                break;
        }
    }


}

