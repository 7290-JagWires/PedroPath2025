package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

/**
 * This class contains all the predefined Poses and PathChains for autonomous routines.
 * It centralizes path creation, making OpModes cleaner and paths reusable.
 */
public class BluePaths {

    // Keep the Follower instance from the OpMode
    private Follower follower;

    // Poses can be public and static so they can be accessed from anywhere
    public static final Pose startPose = new Pose(17, 115, Math.toRadians(25));
    public static final Pose shootPoint = new Pose(59, 85, Math.toRadians(155));
    public static final Pose pickup1Pose = new Pose(37, 83, Math.toRadians(180));
    public static final Pose pickup1Pose2 = new Pose(32, 83, Math.toRadians(180));
    public static final Pose pickup1Pose3 = new Pose(27, 83, Math.toRadians(180));
    public static final Pose endPickup1Pose = new Pose(21, 83, Math.toRadians(180));
    public static final Pose pickup2Pose = new Pose(37, 60, Math.toRadians(180));
    public static final Pose pickup2Pose2 = new Pose(32, 60, Math.toRadians(180));
    public static final Pose pickup2Pose3 = new Pose(27, 60, Math.toRadians(180));
    public static final Pose endPickup2Pose = new Pose(21, 60, Math.toRadians(180));

    // PathChain objects - these will be initialized by buildPaths()
    public PathChain scorePreload, pickup1, pickup1Ball2, pickup1Ball3, endPickup1, score1;
    public PathChain pickup2, pickup2Ball2, pickup2Ball3, endPickup2, score2, endPoint;

    /**
     * Constructor for the Paths class.
     * @param follower The Follower instance from your OpMode.
     */
    public BluePaths(Follower follower) {
        this.follower = follower;
    }

    /**
     * Constructs the various autonomous paths the robot will follow.
     * This method initializes all the PathChain objects.
     */
    public void buildPaths() {
        endPoint = follower.pathBuilder()
                .addPath(new BezierLine(shootPoint, endPickup1Pose))
                .setLinearHeadingInterpolation(shootPoint.getHeading(), endPickup1Pose.getHeading())
                .build();

        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPoint))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPoint.getHeading())
                .build();
        pickup1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPoint, pickup1Pose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), pickup1Pose.getHeading())
                .build();
        pickup1Ball2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, pickup1Pose2))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), pickup1Pose2.getHeading())
                .build();
        pickup1Ball3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose2, pickup1Pose3))
                .setLinearHeadingInterpolation(pickup1Pose2.getHeading(), pickup1Pose3.getHeading())
                .build();
        endPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose3, endPickup1Pose))
                .setLinearHeadingInterpolation(pickup1Pose3.getHeading(), pickup1Pose.getHeading())
                .build();
        score1 = follower.pathBuilder()
                .addPath(new BezierLine(endPickup1Pose, shootPoint))
                .setLinearHeadingInterpolation(endPickup1Pose.getHeading(), shootPoint.getHeading())
                .build();
        pickup2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPoint, pickup2Pose))
                .setLinearHeadingInterpolation( pickup2Pose.getHeading(), pickup2Pose.getHeading())
                .build();
        pickup2Ball2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, pickup2Pose2))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), pickup2Pose2.getHeading())
                .build();
        pickup2Ball3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose2, pickup2Pose3))
                .setLinearHeadingInterpolation(pickup2Pose2.getHeading(), pickup2Pose3.getHeading())
                .build();
        endPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose3, endPickup2Pose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), endPickup2Pose.getHeading())
                .build();
        score2 = follower.pathBuilder()
                .addPath(new BezierLine(endPickup2Pose, shootPoint))
                .setLinearHeadingInterpolation(endPickup2Pose.getHeading(), shootPoint.getHeading())
                .build();
    }
}
