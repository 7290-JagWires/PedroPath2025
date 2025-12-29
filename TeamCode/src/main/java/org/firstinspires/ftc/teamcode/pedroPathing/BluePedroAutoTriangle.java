package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Auto Triangle", group = "Pedro")
public class BluePedroAutoTriangle extends BaseAutonomous {

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
        switch (pathState) {
            case DRIVE_START_SCORE:
                dumpManager.start();

                // drive from start to scoring position
                follower.followPath(paths.scorePreloadTriangle, true);
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
                            follower.followPath(paths.pickup3Triangle);
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
                            follower.followPath(paths.pickup2Triangle);
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
                            follower.followPath(paths.endPointTriangle);
                            follower.setMaxPower(1);
                            intakeActive.intakeOff();
                            setPathState(PathState.END);
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
                        follower.followPath(paths.score1Triangle, true);
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
                        follower.followPath(paths.score2Triangle, true);
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
}
