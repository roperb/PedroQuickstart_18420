package org.firstinspires.ftc.teamcode.pedroPathing.examples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Config
@Autonomous(name = "TestAuto")
public class TestAuto extends OpMode {

    private int pathState;
    private Follower follower;
    private Timer pathTimer, opmodeTimer;

    private final Pose startPose = new Pose(12,84,Math.toDegrees(0));
    private final Pose endLine1 = new Pose(36,60,Math.toDegrees(270));

    private final Pose controlPose = new Pose(36,84,Math.toDegrees(90));

    private PathChain curve1,curve2;

    public void buildPaths() {
        curve1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(startPose.getX(),startPose.getY(), Point.CARTESIAN),
                        new Point(controlPose.getX(),controlPose.getY(), Point.CARTESIAN),
                        new Point(endLine1.getX(),endLine1.getY(), Point.CARTESIAN)))
                .setLinearHeadingInterpolation(startPose.getHeading(),endLine1.getHeading())
                .build();
        curve2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(endLine1.getX(),(endLine1.getY()),(Point.CARTESIAN)),
                        new Point(controlPose.getX(),(controlPose.getY()),(Point.CARTESIAN)),
                        new Point(startPose.getX(),(startPose.getY()),Point.CARTESIAN)))
                .setLinearHeadingInterpolation(endLine1.getHeading(),startPose.getHeading())
                .build();
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(curve1, true);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    follower.followPath(curve2, true);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    setPathState(-3);
                }
                break;

        }
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }


    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();

    }
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        buildPaths();
    }
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
}
