package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Door;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.IntakeActive;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.SpindexerMotor;
import org.firstinspires.ftc.teamcode.pedroPathing.Logic.SpindexerIndexerLogic;
import org.firstinspires.ftc.teamcode.pedroPathing.Sensors.ColorSensor;
import org.firstinspires.ftc.teamcode.pedroPathing.Sensors.IndicatorLight;
import org.firstinspires.ftc.teamcode.pedroPathing.Sensors.MagneticLimitSwitch;
import org.firstinspires.ftc.teamcode.pedroPathing.Utilities.PickupManager;
import org.firstinspires.ftc.teamcode.pedroPathing.Utilities.DumpManager;

@Autonomous(name = "Blue Auto Goal", group = "Pedro")
public class BluePedroAutoGoal extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;
    static final double DOOR_TIMER_DELAY = 2.75;  //Just a delay to make sure robot is set before opening door
    private static final int TICKS_PER_COMPARTMENT = 1354;   // <— update with real measured value
    public IntakeActive intakeActive;
    public Shooter shooter;
    public Door door;
    public SpindexerMotor spindexerMotor;
    public MagneticLimitSwitch spindexerMag;
    public SpindexerIndexerLogic spindexerLogic;
    private PickupManager pickupManager;
    private DumpManager dumpManager;
    private BluePaths paths;


    public enum PathState {
        // These are the various states inside of our auto machine.
        DRIVE_START_SCORE,
        SCORE_PRELOAD,
        SCORE1,
        SCORE2,
        DRIVE_PICKUP1,
        DRIVE_PICKUP1BALL2_END,
        DRIVE_PICKUP1BALL3_END,
        DRIVE_PICKUP1_END,
        DRIVE_PICKUP2,
        DRIVE_PICKUP2BALL2_END,
        DRIVE_PICKUP2BALL3_END,
        DRIVE_PICKUP2_END,
        END
    }
    PathState pathState;

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
            case DRIVE_START_SCORE:
                dumpManager.start();

                // drive from start to scoring position
                follower.followPath(paths.scorePreload, true);
                follower.setMaxPower(1);
                setPathState(PathState.SCORE_PRELOAD);
                break;

            case SCORE_PRELOAD:
                if (pathTimer.getElapsedTimeSeconds() > DOOR_TIMER_DELAY && !follower.isBusy()) {
                    if (dumpManager.isFinished()) {
                        if (pathTimer.getElapsedTimeSeconds() > 6 && !follower.isBusy()) {
                            // just scored preload, drive to pickup point 1
                            pickupManager.start();
                            pickupManager.setTotalBallCount(0);
                            follower.followPath(paths.pickup1);
                            setPathState(PathState.DRIVE_PICKUP1);
                        }
                    }
                }
                break;
            case SCORE1:
                if (pathTimer.getElapsedTimeSeconds() > DOOR_TIMER_DELAY && !follower.isBusy()) {
                    if (dumpManager.isFinished()) {
                        if (pathTimer.getElapsedTimeSeconds() > 7 && !follower.isBusy()) {
                            // scored pickup 1, drive to pickup 2
                            pickupManager.start();
                            pickupManager.setTotalBallCount(0);
                            follower.followPath(paths.pickup2);
                            follower.setMaxPower(1);
                            setPathState(PathState.DRIVE_PICKUP2);
                        }
                    }
                }
                break;
            case SCORE2:
                if (pathTimer.getElapsedTimeSeconds() > DOOR_TIMER_DELAY && !follower.isBusy()) {
                    if (dumpManager.isFinished()) {
                        shooter.stop();         //Turn off the shooter when we finish auto
                        if (pathTimer.getElapsedTimeSeconds() > 7 && !follower.isBusy()) {
                            // end state machine
                            follower.followPath(paths.endPoint);
                            follower.setMaxPower(1);
                            intakeActive.intakeOff();
                            setPathState(PathState.END);
                        }
                    }
                }
                break;
           case DRIVE_PICKUP1:
                if(!follower.isBusy()) {
                    follower.followPath(paths.pickup1Ball2);
                    follower.setMaxPower(1);
                    pathState = PathState.DRIVE_PICKUP1BALL2_END;
                }
               break;
            case DRIVE_PICKUP1BALL2_END:
                pickupManager.update(); // Make sure SM is running
                if (pickupManager.isFinished()) { // Check state with the new method
                    if(!follower.isBusy()) {
                        // picked up at spike 1. drive from pickup1 to score
                        follower.followPath(paths.pickup1Ball3, true);
                        follower.setMaxPower(1);
                        pathState = PathState.DRIVE_PICKUP1BALL3_END;
                        pickupManager.start(); // Restart for the next pickup
                    }
                }
                break;
            case DRIVE_PICKUP1BALL3_END:
                pickupManager.update(); // Make sure SM is running
                if (pickupManager.isFinished()) { // Check state with the new method
                    if(!follower.isBusy()) {
                        // picked up at spike 1. drive from pickup1 to score
                        follower.followPath(paths.endPickup1, true);
                        follower.setMaxPower(1);
                        pathState = PathState.DRIVE_PICKUP1_END;
                        pickupManager.start(); // Restart for the next pickup
                    }
                }
                break;
            case DRIVE_PICKUP1_END:
                pickupManager.update(); // Make sure SM is running
                if (pickupManager.isFinished()) { // Check state with the new method
                    if(!follower.isBusy()) {
                        // picked up at spike 1. drive from pickup1 to score
                        follower.followPath(paths.score1, true);
                        follower.setMaxPower(1);
                        pathState = PathState.SCORE1;
                        intakeActive.intakeOff();
                        dumpManager.start();
                    }
                }
                break;
            case DRIVE_PICKUP2:
                if(!follower.isBusy()) {
                    follower.followPath(paths.pickup2Ball2);
                    follower.setMaxPower(1);
                    pathState = PathState.DRIVE_PICKUP2BALL2_END;
                }
                break;
            case DRIVE_PICKUP2BALL2_END:
                pickupManager.update(); // Make sure SM is running
                if (pickupManager.isFinished()) { // Check state with the new method
                    if(!follower.isBusy()) {
                        // picked up at spike 1. drive from pickup1 to score
                        follower.followPath(paths.pickup2Ball3, true);
                        follower.setMaxPower(1);
                        pathState = PathState.DRIVE_PICKUP2BALL3_END;
                        pickupManager.start(); // Restart for the next pickup
                    }
                }
                break;
            case DRIVE_PICKUP2BALL3_END:
                pickupManager.update(); // Make sure SM is running
                if (pickupManager.isFinished()) { // Check state with the new method
                    if(!follower.isBusy()) {
                        // picked up at spike 1. drive from pickup1 to score
                        follower.followPath(paths.endPickup2, true);
                        follower.setMaxPower(1);
                        pathState = PathState.DRIVE_PICKUP2_END;
                        pickupManager.start(); // Restart for the next pickup
                    }
                }
                break;
            case DRIVE_PICKUP2_END:
                pickupManager.update(); // Make sure SM is running
                if (pickupManager.isFinished()) { // Check state with the new method
                    if(!follower.isBusy()) {
                        // picked up at spike 1. drive from pickup1 to score
                        follower.followPath(paths.score2, true);
                        follower.setMaxPower(1);
                        pathState = PathState.SCORE2;
                        intakeActive.intakeOff();
                        dumpManager.start();
                    }
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

        pathTimer = new Timer();
        opModeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);

        //TODO: add in any other Init statements here for other hardware, shooters, etc.
        intakeActive = new IntakeActive(this);
        shooter      = new Shooter(this);
        door         = new Door(this);
        ColorSensor colorSensor = new ColorSensor(hardwareMap, "color_sensor");
        IndicatorLight indicator = new IndicatorLight(hardwareMap, "rgbServo");


        // Spindexer + magnetic sensor
        spindexerMotor = new SpindexerMotor(hardwareMap);
        spindexerMag   = new MagneticLimitSwitch(hardwareMap, "magnetic_limit_sensor");
        spindexerLogic = new SpindexerIndexerLogic(
                this,
                spindexerMotor,
                spindexerMag,
                TICKS_PER_COMPARTMENT
        );

        dumpManager = new DumpManager(shooter, door, spindexerLogic);

        pickupManager = new PickupManager(
                door,
                shooter,
                intakeActive,
                spindexerLogic,
                colorSensor,
                indicator
        );

        // Initialize the Paths class AFTER the follower is created
        paths = new BluePaths(follower);
        paths.buildPaths(); // Build all the paths

        // Set the starting pose using the static Pose from the Paths class
        follower.setPose(BluePaths.startPose);
    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState); // update state machine with starting state
    }

    @Override
    public void loop() {
        // update state machine + odometry
        follower.update();
        dumpManager.update();
        pickupManager.update();
        autonomousPathUpdate();

        // give data back to drivers
        telemetry.addData("path state", pathState.toString());
        telemetry.addData("dump state", dumpManager.getState());
        telemetry.addData("Dump Count", dumpManager.getDumpCount());
        telemetry.addData("Total Ball Count", pickupManager.getTotalBallCount());
        telemetry.addData("pickup state", pickupManager.getState()); // Update telemetry
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());
    }

}

