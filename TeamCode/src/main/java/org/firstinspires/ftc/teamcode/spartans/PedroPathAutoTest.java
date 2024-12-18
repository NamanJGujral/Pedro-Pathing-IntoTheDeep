package org.firstinspires.ftc.teamcode.spartans;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Config
@Autonomous(name = "Test: Pedro Path Custom", group = "Tests")
public class PedroPathAutoTest extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private Telemetry telemetryA;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    public static double DISTANCE = 40;

    /** Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
//    private final Pose startPose = new Pose(9, 111, Math.toRadians(270));
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));

    private final Pose pose1 = new Pose(12, 0, Math.toRadians(0));

    private final Pose pose2 = new Pose(12, 12, Math.toRadians(0));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
//    private final Pose scorePose = new Pose(14, 129, Math.toRadians(315));

    /** Lowest (First) Sample from the Spike Mark */
//    private final Pose pickup1Pose = new Pose(37, 121, Math.toRadians(0));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, path2, park;
    private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(pose1)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), pose1.getHeading());

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        grabPickup1 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
//                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        scorePickup1 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
//                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
//                .build();

        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
//        park = new Path(new BezierCurve(new Point(scorePose), /* Control Point */ new Point(parkControlPose), new Point(parkPose)));
//        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());

        path2 = new Path(new BezierLine(new Point(pose1), new Point(pose2)));
        path2.setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading());
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                /* You could check for
                - Follower State: "if(!follower.isBusy() {}" (Though, I don't recommend this because it might not return due to holdEnd
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */
                if(follower.getPose().getX() > (pose1.getX() - 1) && follower.getPose().getY() > (pose1.getY() - 1)) {
                    /* Score Preload */
//                    claw.scoringClaw();
//                    claw.openClaw();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(path2,true);
                    setPathState(2);
                }

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if(follower.getPose().getX() > (scorePose.getX() - 1) && follower.getPose().getY() > (scorePose.getY() - 1)) {
//                    /* Score Preload */
//                    claw.scoringClaw();
//                    claw.openClaw();
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                    follower.followPath(grabPickup1,true);
//                    setPathState(2);
//                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
//                if(follower.getPose().getX() > (pickup1Pose.getX() - 1) && follower.getPose().getY() > (pickup1Pose.getY() - 1)) {
//                    /* Grab Sample */
//                    claw.groundClaw();
//                    claw.closeClaw();
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
//                    follower.followPath(scorePickup1,true);
//                    setPathState(3);
//                }
                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if(follower.getPose().getX() > (parkPose.getX() - 1) && follower.getPose().getY() > (parkPose.getY() - 1)) {
//                    /* Put the claw in position to get a level 1 ascent */
//                    claw.startClaw();
//                    claw.closeClaw();
//
//                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
//                    setPathState(-1);
//                }
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        follower = new Follower(hardwareMap);

        pathTimer = new Timer();
        opmodeTimer = new Timer();

        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will run a custom path");
        telemetryA.update();
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void loop() {
        follower.update();
//        if (!follower.isBusy()) {
//            double xPos = 24;
//            double yPos = 0;
//            Path pathA = new Path(new BezierLine(new Point(0,0, Point.CARTESIAN), new Point(xPos, yPos, Point.CARTESIAN)));
//            pathA.setConstantHeadingInterpolation(0);
//
//            follower.followPath(pathA);
//            telemetryA.addData("going to ", pathA);
//            follower.telemetryDebug(telemetryA);

//            yPos = yPos - 12;
//            Path pathB = new Path(new BezierLine(new Point(0,0, Point.CARTESIAN), new Point(xPos, yPos, Point.CARTESIAN)));
//            pathB.setConstantHeadingInterpolation(0);
//
//            follower.followPath(pathB);
//            telemetryA.addData("going to ", pathB);
//            follower.telemetryDebug(telemetryA);
//        }

        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
