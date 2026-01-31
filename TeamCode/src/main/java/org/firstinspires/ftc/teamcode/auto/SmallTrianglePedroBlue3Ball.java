package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.PedroConstants;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.Robot_Auto;
import org.firstinspires.ftc.teamcode.util.Turret;

@Autonomous
public class SmallTrianglePedroBlue3Ball extends OpMode {
    private Follower follower;
    private Robot_Auto robot;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    public PathChain startToParking;

    public static Pose startToParkingStartPose = new Pose(56,8,Math.toRadians(90));
    public static Pose startToParkingEndPose = new Pose(60,12);



    public static double startHeading = Math.toRadians(90);
    public static double rowOfBallsHeading = Math.toRadians(180);
    public static double triangleHeading = Math.toRadians(200);
    public static double gateHeading = Math.toRadians(140);



    public void buildPaths(Follower follower) {
        startToParking = follower.pathBuilder().addPath(
                new BezierLine(
                        startToParkingStartPose,
                        startToParkingEndPose
                ))
                .setTangentHeadingInterpolation()
                .build();
    }

    public void autonomousUpdate(){
        switch (pathState) {
            case 0:
                if (pathTimer.getElapsedTimeSeconds()>1.5)
                    robot.turret.setHorizontalPositionFromAngle(Math.toRadians(16.4));

                if (pathTimer.getElapsedTimeSeconds()>2) {
                    robot.sorting.respectMotif = robot.sorting.getNumberOfBalls() == 3 && robot.sorting.getGreen() != 0;
                    robot.sorting.setNextState(Constants.MOVING_STATES.SHOOTING);
                    setPathState(1);
                }
                break;
            case 1:
                if (pathTimer.getElapsedTimeSeconds()>3.2) {
                    follower.followPath(startToParking);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()){
                    Robot.poseBetweenTeleops = follower.getPose();
                    setPathState(-1);
                }
                break;
        }
    }



    @Override
    public void loop() {
        follower.update();
        robot.update();
        autonomousUpdate();
    }

    @Override
    public void init() {
        follower = PedroConstants.createFollower(hardwareMap);
        robot = new Robot_Auto(hardwareMap,telemetry);
        robot.sorting.fillMagazine();
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        buildPaths(follower);
        robot.sorting.respectMotif = true;
        follower.setStartingPose(startToParkingStartPose);


    }
    public void setPathState(int pState){
        pathState = pState;
        pathTimer.resetTimer();
    }
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        robot.turret.setVerticalPositionFromAngle(Math.toRadians(30));
        Turret.setTarget_rotation(58);
        setPathState(0);
    }
}
