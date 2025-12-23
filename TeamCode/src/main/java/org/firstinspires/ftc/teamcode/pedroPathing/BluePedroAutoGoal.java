package org.firstinspires.ftc.teamcode.pedroPathing;

import static java.lang.Thread.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
    ElapsedTime dumpTimer = new ElapsedTime();
    boolean isDumping = false;
    int dumpCount = 0;
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
    private final Pose pickup1Pose = new Pose(37, 83, Math.toRadians(180));
    private final Pose endPickup1Pose = new Pose(21, 83, Math.toRadians(180));
    private final Pose pickup2Pose = new Pose(37, 60, Math.toRadians(180));
    private final Pose endPickup2Pose = new Pose(21, 60, Math.toRadians(180));


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

    public void buildPaths() {
        endPoint = follower.pathBuilder() // returns the robot pickup1Pose to end off shooting line
                .addPath(new BezierLine(shootPoint, endPickup1Pose))
                .setLinearHeadingInterpolation(shootPoint.getHeading(), endPickup1Pose.getHeading())
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

    public void autonomousPathUpdate() {
        switch(pathState) {
            case SCORE_PRELOAD:
                //TODO: run standard scoring state machine here
                if (!isDumping) {
                    startDumping();
                }

                // The dumpItems() method is now called from the main loop()
                // so we just wait until it's finished.
                if (!isDumping && !follower.isBusy()){
                    // just scored preload, drive to pickup point 1
                    follower.followPath(pickup1);
                    setPathState(PathState.DRIVE_PICKUP1);
                }

                if (pathTimer.getElapsedTimeSeconds() > 4 && !follower.isBusy()){
                    // just scored preload, drive to pickup point 1
                    follower.followPath(pickup1);
                    setPathState(PathState.DRIVE_PICKUP1);
                }
                break;
            case SCORE1:
                //TODO: run standard scoring state machine here
                if(pathTimer.getElapsedTimeSeconds() > 5 && !follower.isBusy()){
                    // scored pickup 1, drive to pickup 2
                    follower.followPath(pickup2);
                    follower.setMaxPower(1);
                    setPathState(PathState.DRIVE_PICKUP2);
                }
                break;
            case SCORE2:
                //TODO: run standard scoring state machine here
                if(pathTimer.getElapsedTimeSeconds() > 6 && !follower.isBusy()) {
                    // end state machine
                    follower.followPath(endPoint);
                    follower.setMaxPower(1);
                    setPathState(PathState.END);
                }
                break;
            case DRIVE_START_SCORE:
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
        opModeTimer.resetTimer();

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

        // Call the updated dump logic from the main loop
        if (isDumping) {
            dumpItems();
        }
        // give data back to drivers
        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());
    }

    // Call this method to START the dumping process
    void startDumping() {
        shooter.run(); // Start the shooter motor
        dumpTimer.reset();
        isDumping = true;
        dumpCount = 0;
    }
    void dumpItems() {
        // Step 1: Wait for the shooter to spin up
        if (dumpTimer.milliseconds() < LAUNCH_DELAY_MILLISECONDS) {
            // Still waiting for the shooter to get to speed, do nothing else
            return;
        }

        // Step 2: Sequentially launch items
        if (dumpCount < 3) {
            door.forceOpenLock(); // Keep the door open

            // If the spindexer is at the correct position (limit switch pressed)
            if (spindexerLogic.spindexerLimitSwitchCheck()) {
                // and enough time has passed to let the item settle and shoot...
                if (dumpTimer.milliseconds() > 500) { // 500ms delay to shoot and settle
                    dumpCount++;
                    telemetry.addLine("dumpCount: " + dumpCount);
                    if (dumpCount < 3) {
                        // Move to the next compartment if we are not done
                        spindexerLogic.nextCompartment();
                        dumpTimer.reset(); // Reset timer for the next item
                    }
                }
            }
        } else {
            // Step 3: We are done dumping
            isDumping = false;
            shooter.stopFast(); // Stop the shooter
            door.closeDoor();   // Close the door
            // Now you can proceed to the next state in your autonomous path
        }
    }

}

