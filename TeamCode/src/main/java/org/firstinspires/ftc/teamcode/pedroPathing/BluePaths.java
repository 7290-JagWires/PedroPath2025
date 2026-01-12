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
    public static final Pose startPose = new Pose(16, 115, Math.toRadians(25));
    public static final Pose startPoseTriangle = new Pose(56, 9, Math.toRadians(90));
    public static final Pose shootPoint = new Pose(59, 97, Math.toRadians(152));
    public static final Pose shootPointTriangle = new Pose(56, 18, Math.toRadians(121));

    //Common poses for all programs
    public static final Pose pickup1Pose = new Pose(42, 82, Math.toRadians(180));
    public static final Pose pickup1Pose2 = new Pose(33, 82, Math.toRadians(180));
    public static final Pose pickup1Pose3 = new Pose(28, 82, Math.toRadians(180));
    public static final Pose endPickup1Pose = new Pose(20, 82, Math.toRadians(180));
    public static final Pose pickup2Pose = new Pose(56, 59, Math.toRadians(180));
    public static final Pose pickup2Pose2 = new Pose(33, 59, Math.toRadians(180));
    public static final Pose pickup2Pose3 = new Pose(26, 59, Math.toRadians(180));
    public static final Pose endPickup2Pose = new Pose(20, 59, Math.toRadians(180));
    public static final Pose pickup3Pose = new Pose(47, 37, Math.toRadians(180));
    public static final Pose pickup3Pose2 = new Pose(36, 37, Math.toRadians(180));
    public static final Pose pickup3Pose3 = new Pose(31, 37, Math.toRadians(180));
    public static final Pose endPickup3Pose = new Pose(20, 37, Math.toRadians(180));
    

    // PathChain objects - these will be initialized by buildPaths()
    public PathChain scorePreload, pickup1, pickup1Ball2, pickup1Ball3, endPickup1, score1;
    public PathChain pickup2, pickup2Ball2, pickup2Ball3, endPickup2, score2, pickup3, score3, endPoint;
    public PathChain pickup1Triangle, score1Triangle, pickup2Triangle, score2Triangle, scorePreloadTriangle, pickup3Triangle, pickup3Ball2;
    public PathChain pickup3Ball3, endPickup3, score3Triangle, endPointTriangle, myBezierCurvePath;

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
        /*
        Anything with Pickup 1 - Ball pattern in the order of pickup is Purple, Purple, Green
        Anything with Pickup 2 - Ball pattern in the order of pickup is Purple, Green, Purple
        Anything with Pickup 2 - Ball pattern in the order of pickup is Green ,Purple, Purple
        */

        //  Common paths
        //  Common paths
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
        pickup3Ball2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, pickup3Pose2))
                .setLinearHeadingInterpolation( pickup3Pose.getHeading(), pickup3Pose2.getHeading())
                .build();
        pickup3Ball3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose2, pickup3Pose3))
                .setLinearHeadingInterpolation( pickup3Pose2.getHeading(), pickup3Pose3.getHeading())
                .build();
        endPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose3, endPickup3Pose))
                .setLinearHeadingInterpolation( pickup3Pose3.getHeading(), endPickup3Pose.getHeading())
                .build();

        //  From Goal Paths
        //  From Goal Paths
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPoint))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPoint.getHeading())
                .build();
        pickup1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPoint, pickup1Pose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), pickup1Pose.getHeading())
                .build();
        score1 = follower.pathBuilder()
                .addPath(new BezierLine(endPickup1Pose, shootPoint))
                .setLinearHeadingInterpolation(endPickup1Pose.getHeading(), shootPoint.getHeading())
                .build();
        pickup2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPoint, pickup2Pose))
                .setLinearHeadingInterpolation( pickup2Pose.getHeading(), pickup2Pose.getHeading())
                .build();
        score2 = follower.pathBuilder()
                .addPath(new BezierLine(endPickup2Pose, shootPoint))
                .setLinearHeadingInterpolation(endPickup2Pose.getHeading(), shootPoint.getHeading())
                .build();
        pickup3 = follower.pathBuilder()
                .addPath(new BezierLine(shootPoint, pickup3Pose))
                .setLinearHeadingInterpolation( shootPoint.getHeading(), pickup3Pose.getHeading())
                .build();
        score3 = follower.pathBuilder()
                .addPath(new BezierLine(endPickup3Pose, shootPoint))
                .setLinearHeadingInterpolation(endPickup3Pose.getHeading(), shootPoint.getHeading())
                .build();
        endPoint = follower.pathBuilder()
                .addPath(new BezierLine(shootPoint, endPickup1Pose))
                .setLinearHeadingInterpolation(shootPoint.getHeading(), endPickup1Pose.getHeading())
                .build();

        // Triangle Paths
        // Triangle Paths
        scorePreloadTriangle = follower.pathBuilder()
                .addPath(new BezierLine(startPoseTriangle, shootPointTriangle))
                .setLinearHeadingInterpolation(startPoseTriangle.getHeading(), shootPointTriangle.getHeading())
                .build();
        pickup1Triangle = follower.pathBuilder()
                .addPath(new BezierLine(shootPointTriangle, pickup1Pose))
                .setLinearHeadingInterpolation( shootPointTriangle.getHeading(), pickup2Pose.getHeading())
                .build();
        score1Triangle = follower.pathBuilder()
                .addPath(new BezierLine(endPickup1Pose, shootPointTriangle))
                .setLinearHeadingInterpolation(endPickup2Pose.getHeading(), shootPointTriangle.getHeading())
                .build();
        pickup2Triangle = follower.pathBuilder()
                .addPath(new BezierLine(shootPointTriangle, pickup2Pose))
                .setLinearHeadingInterpolation( shootPointTriangle.getHeading(), pickup2Pose.getHeading())
                .build();
        score2Triangle = follower.pathBuilder()
                .addPath(new BezierLine(endPickup2Pose, shootPointTriangle))
                .setLinearHeadingInterpolation(endPickup2Pose.getHeading(), shootPointTriangle.getHeading())
                .build();
        pickup3Triangle = follower.pathBuilder()
                .addPath(new BezierLine(shootPointTriangle, pickup3Pose))
                .setLinearHeadingInterpolation( shootPointTriangle.getHeading(), pickup3Pose.getHeading())
                .build();
        score3Triangle = follower.pathBuilder()
                .addPath(new BezierLine(endPickup3Pose, shootPointTriangle))
                .setLinearHeadingInterpolation(endPickup3Pose.getHeading(), shootPointTriangle.getHeading())
                .build();
        endPointTriangle = follower.pathBuilder()  //Finish spot
                .addPath(new BezierLine(shootPointTriangle, pickup3Pose))
                .setLinearHeadingInterpolation(shootPointTriangle.getHeading(), pickup3Pose.getHeading())
                .build();
    }
}
