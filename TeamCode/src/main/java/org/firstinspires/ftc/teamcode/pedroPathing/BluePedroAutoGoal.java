package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Shooter.SHOOT_GOAL_VELOCITY;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Shooter;

@Autonomous(name = "Blue Auto Goal", group = "Pedro")
public class BluePedroAutoGoal extends BaseAutonomous {

    // This class only needs to contain what is UNIQUE to the Blue side.
    private BluePaths paths;

    // 1. Implement the required abstract methods
    @Override
    protected void buildPaths() {
        // The BluePaths class handles creating the unique paths for the blue side
        paths = new BluePaths(follower);
        paths.buildPaths();
    }

    @Override
    protected void setStartingPose() {
        // Set the specific starting pose for the blue side
        follower.setPose(BluePaths.startPose);
    }

    public void autonomousPathUpdate() {
        shooter.setExplicitVelocity(SHOOT_GOAL_VELOCITY);
        switch (pathState) {
            case DRIVE_START_SCORE:
                // Get the tag ID that was detected during the init_loop
                int detectedTagId = getStartingTagId();

                // Failsafe: If no tag was ever seen, default to one. 23 is our preload
                if (detectedTagId == -1) {
                    detectedTagId = 23; // Default to Right Tag
                    telemetry.addLine("!!! No Tag Seen During Init, Defaulting to 23 !!!");
                }

                // ASSUMING WE preload for TAG_ID 23
                if (detectedTagId == 21) { // Left Tag
                    // Preloaded for 23, need Left. Spin 2 times.
                    spindexerRotator.start(2);
                } else if (detectedTagId == 22) { // Center Tag
                    // Preloaded for 23, need Center. Spin 1 time.
                    spindexerRotator.start(1);
                } else { // Tag is 23 (our preload)
                    // Preloaded for 23, need Right. Do nothing.
                    spindexerRotator.start(0);
                }

                // 4. Start driving the path (non-blocking which is the falst command)
                follower.followPath(paths.scorePreload, false);
                follower.setMaxPower(1);

                // 5. Immediately go to the waiting state.
                setPathState(PathState.WAIT_FOR_SPIN);
                break;
            case WAIT_FOR_SPIN:
                if (!follower.isBusy() && spindexerRotator.isFinished()) {
                    dumpManager.start();
                    setPathState(PathState.SCORE_PRELOAD);
                }
                break;
            case SCORE_PRELOAD:
                // This state's job is unchanged: wait for the dumper to finish.
                if (dumpManager.isFinished()) {
                    // Preload scored. Move to the next action.
                    pickupManager.start();
                    pickupManager.setTotalBallCount(0);
                    follower.followPath(paths.pickup1);
                    setPathState(PathState.DRIVE_PICKUP1);
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
                        shooter.stopShooter();         //Turn off the shooter when we finish auto
                        if (pathTimer.getElapsedTimeSeconds() > 7 && !follower.isBusy()) {
                            // end state machine
                            follower.followPath(paths.endPoint);
                            follower.setMaxPower(1);
                            intakeActive.intakeOff();
//                            setPathState(PathState.END);
                        }
                    }
                }
                break;
            case SCORE3:
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
            case DRIVE_PICKUP1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.pickup1Ball2);
                    follower.setMaxPower(1);
                    pathState = PathState.DRIVE_PICKUP1BALL2_END;
                }
                break;
            case DRIVE_PICKUP1BALL2_END:
                pickupManager.update(); // Make sure SM is running
                if (pickupManager.isFinished()) { // Check state with the new method
                    if (!follower.isBusy()) {
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
                    if (!follower.isBusy()) {
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
                    if (!follower.isBusy()) {
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
                if (!follower.isBusy()) {
                    follower.followPath(paths.pickup2Ball2);
                    follower.setMaxPower(1);
                    pathState = PathState.DRIVE_PICKUP2BALL2_END;
                }
                break;
            case DRIVE_PICKUP2BALL2_END:
                pickupManager.update(); // Make sure SM is running
                if (pickupManager.isFinished()) { // Check state with the new method
                    if (!follower.isBusy()) {
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
                    if (!follower.isBusy()) {
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
                    if (!follower.isBusy()) {
                        // picked up at spike 1. drive from pickup1 to score
                        follower.followPath(paths.score2, true);
                        follower.setMaxPower(1);
                        pathState = PathState.SCORE2;
                        intakeActive.intakeOff();
                        dumpManager.start();
                    }
                }
                break;
            case DRIVE_PICKUP3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.pickup3Ball2);
                    follower.setMaxPower(1);
                    pathState = PathState.DRIVE_PICKUP3BALL2_END;
                }
                break;
            case DRIVE_PICKUP3BALL2_END:
                if (pickupManager.isFinished()) { // Check state with the new method
                    if (!follower.isBusy()) {
                        // picked up at spike 1. drive from pickup1 to score
                        follower.followPath(paths.pickup3Ball3, true);
                        follower.setMaxPower(1);
                        pathState = PathState.DRIVE_PICKUP3BALL3_END;
                        pickupManager.start(); // Restart for the next pickup
                    }
                }
                break;
            case DRIVE_PICKUP3BALL3_END:
                pickupManager.update(); // Make sure SM is running
                if (pickupManager.isFinished()) { // Check state with the new method
                    if (!follower.isBusy()) {
                        // picked up at spike 1. drive from pickup1 to score
                        follower.followPath(paths.endPickup3, true);
                        follower.setMaxPower(1);
                        pathState = PathState.DRIVE_PICKUP3_END;
                        pickupManager.start(); // Restart for the next pickup
                    }
                }
                break;
            case DRIVE_PICKUP3_END:
                pickupManager.update(); // Make sure SM is running
                if (pickupManager.isFinished()) { // Check state with the new method
                    if (!follower.isBusy()) {
                        // picked up at spike 1. drive from pickup1 to score
                        follower.followPath(paths.score3, true);
                        follower.setMaxPower(1);
                        pathState = PathState.SCORE3;
                        intakeActive.intakeOff();
                        dumpManager.start();
                    }
                }
                break;
            default:
                break;
        }
    }
}
