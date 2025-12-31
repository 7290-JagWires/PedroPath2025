package org.firstinspires.ftc.teamcode.pedroPathing.examples;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

// Import your mechanism class
// import org.firstinspires.ftc.teamcode.mechanisms.FlywheelExampleServo;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
@Disabled
public class SampleActions extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    // 1. Define your mechanism
    private FlywheelExampleServo shooter = new FlywheelExampleServo();

    // Flag to ensure we only pull the trigger once per state
    private boolean shotsTriggered = false;

    public enum PathState {
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD,
        DRIVE_SHOOTPOS_ENDPOS
    }

    PathState pathState;

    private final Pose startPose = new Pose(20.386209877877445, 122.39783853885227, Math.toRadians(138));
    private final Pose shootPose = new Pose(46.415043769588245, 96.90020533880903,Math.toRadians(138));
    private final Pose endPose = new Pose(63.76759969739543, 105.75355019993515, Math.toRadians(90));

    private PathChain driveStartPosShootPos, driveShootPosEndPos;

    public void buildPaths() {
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        driveShootPosEndPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch(pathState) {
            case DRIVE_STARTPOS_SHOOT_POS:
                // Follow the path
                follower.followPath(driveStartPosShootPos, true);
                // Move to the next state logic immediately (Pedro style)
                setPathState(PathState.SHOOT_PRELOAD);
                break;

            case SHOOT_PRELOAD:
                // 1. Check if the robot has reached the destination (Follower not busy)
                if (!follower.isBusy()) {

                    // 2. If we haven't pulled the trigger yet, fire 3 shots
                    if (!shotsTriggered) {
                        shooter.fireShots(3);
                        shotsTriggered = true;
                    }

                    // 3. Wait until the shooter is finished AND we have actually triggered it
                    else if (shotsTriggered && !shooter.isBusy()) {
                        // Shots are done, move to next path
                        follower.followPath(driveShootPosEndPos, true);
                        setPathState(PathState.DRIVE_SHOOTPOS_ENDPOS);
                    }
                }
                break;

            case DRIVE_SHOOTPOS_ENDPOS:
                if (!follower.isBusy()) {
                    telemetry.addLine("Done all Paths");
                }
                break; // Added break for safety

            default:
                telemetry.addLine("No State Commanded");
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
        // Reset the trigger flag whenever we change states
        shotsTriggered = false;
    }

    @Override
    public void init() {
        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        // 2. Initialize the shooter mechanism
        // Make sure config names "servo" and "cr_servo" exist in Driver Hub
        shooter.init(hardwareMap);

        buildPaths();
        follower.setPose(startPose);
    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();

        // 3. CRITICAL: Update the mechanism every loop cycle
        shooter.update();

        statePathUpdate();

        telemetry.addData("Path State", pathState.toString());
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Shooter Busy", shooter.isBusy());
        telemetry.update();
    }
}