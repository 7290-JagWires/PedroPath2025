package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "PedroAutoBlue", group = "Pedro")
public class BluePedroAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;


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
    private final Pose startPose = new Pose(17, 115, Math.toRadians(30));
    private final Pose shootPoint = new Pose(59, 85, Math.toRadians(155));
    private final Pose pickup1Pose = new Pose(42, 83, Math.toRadians(180));
    private final Pose endPickup1Pose = new Pose(24, 83, Math.toRadians(180));
    private final Pose pickup2Pose = new Pose(41, 59, Math.toRadians(180));
    private final Pose endPickup2Pose = new Pose(25, 60, Math.toRadians(180));
    private final Pose endAuto1Pose = new Pose(24, 85, Math.toRadians(180));
    //Point=Pose


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
        endPoint = follower.pathBuilder() // returns the robot to start
                .addPath(new BezierLine(shootPoint, pickup1Pose))
                .setLinearHeadingInterpolation(shootPoint.getHeading(), pickup1Pose.getHeading())
                .build();
        scorePreload = follower.pathBuilder() // moves from from start > scoring position
                .addPath(new BezierLine(startPose, shootPoint))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPoint.getHeading())
                .build();
        pickup1 = follower.pathBuilder() // moves from scoring position  > pickup 1 point, 1/2 speed
                .addPath(new BezierLine(shootPoint, pickup1Pose))
                .setLinearHeadingInterpolation(shootPoint.getHeading(), pickup1Pose.getHeading(), 0.4) // get angle sooner
                //.setGlobalDeceleration(0.5)
                //.setBrakingStart(0.4)
                .build();
        endPickup1 = follower.pathBuilder() // moves from scoring position  > pickup 1 point, 1/2 speed
                .addPath(new BezierLine(pickup1Pose, endPickup1Pose))
                .setLinearHeadingInterpolation(shootPoint.getHeading(), pickup1Pose.getHeading(), 0.4) // get angle sooner
//                .setGlobalDeceleration(0.5)
//                .setBrakingStart(0.4)
                .build();
        score1 = follower.pathBuilder() // moves from pickup 1 > scoring position
                .addPath(new BezierLine(endPickup1Pose, shootPoint))
                .setLinearHeadingInterpolation(endPickup1Pose.getHeading(), shootPoint.getHeading(),0.8) //0.8 is suggested
                .build();
        pickup2 = follower.pathBuilder() // moves from scoring position > pickup 2
                // use a curve to we can line up better for the balls
                .addPath(new BezierLine(shootPoint, pickup2Pose))
                .setLinearHeadingInterpolation(shootPoint.getHeading(), pickup2Pose.getHeading(),0.4) // get angle sooner
                .build();
        endPickup2 = follower.pathBuilder() // moves from scoring position > pickup 2
                // use a curve to we can line up better for the balls
                .addPath(new BezierLine(pickup2Pose, endPickup2Pose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), endPickup2Pose.getHeading(),0.4) // get angle sooner
                .build();
        score2 = follower.pathBuilder() // moves from pickup 2 > scoring position
                .addPath(new BezierLine(endPickup2Pose, shootPoint))
                .setLinearHeadingInterpolation(endPickup2Pose.getHeading(), shootPoint.getHeading(), 0.8)
                .build();
        }

    public void autonomousPathUpdate() {
        switch(pathState) {
            case SCORE_PRELOAD:
                //TODO: run standard scoring state machine here
                if(pathTimer.getElapsedTimeSeconds() > 4 && !follower.isBusy()){
                    // just scored preload, drive to pickup point 1
                    follower.followPath(pickup1);
//                    follower.setMaxPower(0.5); // slows down permanently
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
}

