package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Shooter.SHOOT_GOAL_VELOCITY;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Auto Goal", preselectTeleOp = "RedTeleOpMain")
public class RedPedroAutoGoal extends BaseAutonomous {

    // This class only needs to contain what is UNIQUE to the Red side.
    private RedPaths paths;



    // 1. Implement the required abstract methods
    @Override
    protected void buildPaths() {
        // The RedPaths class handles creating the unique paths for the Red side
        paths = new RedPaths(follower);
        paths.buildPaths();
    }

    @Override
    protected void setStartingPose() {
        // Set the specific starting pose for the red side
        follower.setPose(RedPaths.startPose);
    }

    public void autonomousPathUpdate() {
        intakeActive.intakeOn();
        shooter.setExplicitVelocity(SHOOT_GOAL_VELOCITY);
        switch (pathState) {
            case DRIVE_START_SCORE:
                // Get the tag ID that was detected during the init_loop
                // and sort the balls in the correct order
                startSpindexerRotationForTag(STARTING_TAG_ID);

                // 4. Start driving the path (non-blocking which is the falst command)
                follower.followPath(paths.scorePreload, false);
                follower.setMaxPower(1);

                // 5. Immediately go to the waiting state.
                setPathState(PathState.WAIT_FOR_SPIN);
                break;
            case WAIT_FOR_SPIN:
                if (!follower.isBusy() && spindexerRotator.isFinished()) {

//                    dumpManager.start();

                    dumpManager.scorePreLoad();


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
//                            follower.followPath(paths.pickup2);



                            follower.followPath(paths.pickup2Curve);



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
                        intakeActive.intakeOff();
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
                            follower.setMaxPower(.8);
                            setPathState(PathState.DRIVE_PICKUP2);
                        }
                    }
                }
                break;
            case DRIVE_PICKUP1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.pickup1Ball2);
                    follower.setMaxPower(1);
                    pathTimer.resetTimer(); // Start the timer for the first pickup attempt
                    pathState = PathState.DRIVE_PICKUP1BALL2_END;
                }
                break;
            case DRIVE_PICKUP1BALL2_END:
                pickupManager.update(); // Make sure SM is running
                if (pickupManager.isFinished()) { // Check state with the new method
                    if (!follower.isBusy()) {
                        // picked up at spike 1. drive from pickup1 to score
                        follower.followPath(paths.pickup1Ball3, false);
                        follower.setMaxPower(1);
                        pathState = PathState.DRIVE_PICKUP1BALL3_END;
                        pathTimer.resetTimer(); // Start the timer for the first pickup attempt
                        pickupManager.start(); // Restart for the next pickup
                    }
                } else if (!pickupManager.isFinished() && pathTimer.getElapsedTimeSeconds() > PICKUP_MISSED_BALL_TIMER_DELAY) {
                    if (!follower.isBusy()) {
                        // Get the tag ID that was detected during the init_loop
                        // and sort the balls in the correct order
                        startSpindexerRotationForTag(PICKUP_ROW1_PPG);

                        // picked up at spike 1. drive from pickup1 to score
                        follower.followPath(paths.score1, false);
                        follower.setMaxPower(1);
                        pathState = PathState.SCORE1;
                        dumpManager.start();
                        pathTimer.resetTimer(); // Start the timer for the first pickup attempt
                    }
                }
                break;
            case DRIVE_PICKUP1BALL3_END:
                pickupManager.update(); // Make sure SM is running
                if (pickupManager.isFinished()) { // Check state with the new method
                    if (!follower.isBusy()) {
                        // picked up at spike 1. drive from pickup1 to score
                        follower.followPath(paths.endPickup1, false);
                        follower.setMaxPower(1);
                        pathState = PathState.DRIVE_PICKUP1_END;
                        pathTimer.resetTimer(); // Start the timer for the first pickup attempt
                        pickupManager.start(); // Restart for the next pickup
                    }
                } else if (!pickupManager.isFinished() && pathTimer.getElapsedTimeSeconds() > PICKUP_MISSED_BALL_TIMER_DELAY) {
                    if (!follower.isBusy()) {
                        // Get the tag ID that was detected during the init_loop
                        // and sort the balls in the correct order
                        startSpindexerRotationForTag(PICKUP_ROW1_PPG);

                        // picked up at spike 1. drive from pickup1 to score
                        follower.followPath(paths.score1, false);
                        follower.setMaxPower(1);
                        pathState = PathState.SCORE1;
                        dumpManager.start();
                        pathTimer.resetTimer(); // Start the timer for the first pickup attempt
                    }
                }
                break;
            case DRIVE_PICKUP1_END:
                pickupManager.update(); // Make sure SM is running
                if (pickupManager.isFinished()) { // Check state with the new method
                    if (!follower.isBusy()) {
                        // Get the tag ID that was detected during the init_loop
                        // and sort the balls in the correct order
                        startSpindexerRotationForTag(PICKUP_ROW1_PPG);

                        // picked up at spike 1. drive from pickup1 to score
                        follower.followPath(paths.score1, false);
                        follower.setMaxPower(1);
                        pathState = PathState.SCORE1;
                        dumpManager.start();
                        pathTimer.resetTimer(); // Start the timer for the first pickup attempt
                    }
                } else if (!pickupManager.isFinished() && pathTimer.getElapsedTimeSeconds() > PICKUP_MISSED_BALL_TIMER_DELAY) {
                    if (!follower.isBusy()) {
                        // Get the tag ID that was detected during the init_loop
                        // and sort the balls in the correct order
                        startSpindexerRotationForTag(PICKUP_ROW1_PPG);

                        // picked up at spike 1. drive from pickup1 to score
                        follower.followPath(paths.score1, false);
                        follower.setMaxPower(1);
                        pathState = PathState.SCORE1;
                        dumpManager.start();
                        pathTimer.resetTimer(); // Start the timer for the first pickup attempt
                    }
                }
                break;
            case DRIVE_PICKUP2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.pickup2Ball2);
                    follower.setMaxPower(1);
                    pathTimer.resetTimer(); // Start the timer for the first pickup attempt
                    pathState = PathState.DRIVE_PICKUP2BALL2_END;
                }
                break;
            case DRIVE_PICKUP2BALL2_END:
                pickupManager.update(); // Make sure SM is running
                if (pickupManager.isFinished()) { // Check state with the new method
                    if (!follower.isBusy()) {
                        // picked up at spike 1. drive from pickup1 to score
                        follower.followPath(paths.pickup2Ball3, false);
                        follower.setMaxPower(1);
                        pathState = PathState.DRIVE_PICKUP2BALL3_END;
                        pickupManager.start(); // Restart for the next pickup
                        pathTimer.resetTimer(); // Start the timer for the first pickup attempt
                    }
                } else if (pathTimer.getElapsedTimeSeconds() > PICKUP_MISSED_BALL_TIMER_DELAY) {
                    // Get the tag ID that was detected during the init_loop
                    // and sort the balls in the correct order
                    startSpindexerRotationForTag(PICKUP_ROW2_PGP);

                    // picked up at spike 1. drive from pickup1 to score
                    follower.followPath(paths.score2, false);
                    follower.setMaxPower(1);
                    pathState = PathState.SCORE2;
                    dumpManager.start();
                    pathTimer.resetTimer(); // Start the timer for the first pickup attempt
                }
                break;
            case DRIVE_PICKUP2BALL3_END:
                pickupManager.update(); // Make sure SM is running
                if (pickupManager.isFinished()) { // Check state with the new method
                    if (!follower.isBusy()) {
                        // picked up at spike 1. drive from pickup1 to score
                        follower.followPath(paths.endPickup2, false);
                        follower.setMaxPower(1);
                        pathState = PathState.DRIVE_PICKUP2_END;
                        pathTimer.resetTimer(); // Start the timer for the first pickup attempt
                        pickupManager.start(); // Restart for the next pickup
                    }
                } else if (pathTimer.getElapsedTimeSeconds() > PICKUP_MISSED_BALL_TIMER_DELAY) {
                    // Get the tag ID that was detected during the init_loop
                    // and sort the balls in the correct order
                    startSpindexerRotationForTag(PICKUP_ROW2_PGP);

                    // picked up at spike 1. drive from pickup1 to score
                    follower.followPath(paths.score2, false);
                    follower.setMaxPower(1);
                    pathState = PathState.SCORE2;
                    dumpManager.start();
                    pathTimer.resetTimer(); // Start the timer for the first pickup attempt
                }
                break;
            case DRIVE_PICKUP2_END:
                pickupManager.update(); // Make sure SM is running
                if (pickupManager.isFinished()) { // Check state with the new method
                    if (!follower.isBusy()) {
                        // Get the tag ID that was detected during the init_loop
                        // and sort the balls in the correct order
                        startSpindexerRotationForTag(PICKUP_ROW2_PGP);

                        // picked up at spike 1. drive from pickup1 to score
                        follower.followPath(paths.score2, false);
                        follower.setMaxPower(1);
                        pathState = PathState.SCORE2;
                        dumpManager.start();
                        pathTimer.resetTimer(); // Start the timer for the first pickup attempt
                    }
                }
                break;
            case DRIVE_PICKUP3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.pickup3Ball2);
                    follower.setMaxPower(1);
                    pathTimer.resetTimer(); // Start the timer for the first pickup attempt
                    pathState = PathState.DRIVE_PICKUP3BALL2_END;
                }
                break;
            case DRIVE_PICKUP3BALL2_END:
                if (pickupManager.isFinished()) { // Check state with the new method
                    if (!follower.isBusy()) {
                        // picked up at spike 1. drive from pickup1 to score
                        follower.followPath(paths.pickup3Ball3, false);
                        pathState = PathState.DRIVE_PICKUP3BALL3_END;
                        pickupManager.start(); // Restart for the next pickup
                        pathTimer.resetTimer(); // Start the timer for the first pickup attempt
                    }
                } else if (pathTimer.getElapsedTimeSeconds() > PICKUP_MISSED_BALL_TIMER_DELAY) {
                    // Get the tag ID that was detected during the init_loop
                    // and sort the balls in the correct order
                    startSpindexerRotationForTag(PICKUP_ROW3_GPP);

                    // picked up at spike 1. drive from pickup1 to score
                    follower.followPath(paths.score3, false);
                    follower.setMaxPower(1);
                    pathState = PathState.SCORE3;
                    dumpManager.start();
                    pathTimer.resetTimer(); // Start the timer for the first pickup attempt
                }
                break;
            case DRIVE_PICKUP3BALL3_END:
                pickupManager.update(); // Make sure SM is running
                if (pickupManager.isFinished()) { // Check state with the new method
                    if (!follower.isBusy()) {
                        // picked up at spike 1. drive from pickup1 to score
                        follower.followPath(paths.endPickup3, false);
                        pathState = PathState.DRIVE_PICKUP3_END;
                        pickupManager.start(); // Restart for the next pickup
                        pathTimer.resetTimer(); // Start the timer for the first pickup attempt
                    }
                } else if (pathTimer.getElapsedTimeSeconds() > PICKUP_MISSED_BALL_TIMER_DELAY) {
                    // Get the tag ID that was detected during the init_loop
                    // and sort the balls in the correct order
                    startSpindexerRotationForTag(PICKUP_ROW3_GPP);

                    // picked up at spike 1. drive from pickup1 to score
                    follower.followPath(paths.score3, false);
                    follower.setMaxPower(1);
                    pathState = PathState.SCORE3;
                    dumpManager.start();
                    pathTimer.resetTimer(); // Start the timer for the first pickup attempt
                }
                break;
            case DRIVE_PICKUP3_END:
                pickupManager.update(); // Make sure SM is running
                if (pickupManager.isFinished()) { // Check state with the new method
                    if (!follower.isBusy()) {
                        // Get the tag ID that was detected during the init_loop
                        // and sort the balls in the correct order
                        startSpindexerRotationForTag(PICKUP_ROW3_GPP);

                        // picked up at spike 1. drive from pickup1 to score
                        follower.followPath(paths.score3, false);
                        follower.setMaxPower(1);
                        pathState = PathState.SCORE3;
                        dumpManager.start();
                        pathTimer.resetTimer(); // Start the timer for the first pickup attempt
                    }
                }
                break;
            default:
                break;
        }
    }
}
