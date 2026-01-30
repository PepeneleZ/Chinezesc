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
public class SmallTrianglePedroRed6Ball extends OpMode {
    private Follower follower;
    private Robot_Auto robot;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    public PathChain startToSemiMiddle;
    public PathChain semiMiddleToFirstRowOfBall;
    public PathChain firstRowOfBallToTriangle;

    public PathChain triangleToParking;


    public static Pose startToSemiMiddleStartPose = new Pose(56,8,Math.toRadians(90)).mirror();
    public static Pose startToSemiMiddleEndPose = new Pose(43, 36.000).mirror();
    public static Pose semiMiddleToFirstRowOfBallEndPose = new Pose(10,36).mirror();

    public static Pose firstRowOfBallToTriangleEndPose = new Pose(72.000, 72.000).mirror();

    public static Pose triangleToParkingEndPose = new Pose(52,54.5).mirror();

    public static double startHeading = Math.toRadians(90);
    public static double rowOfBallsHeading = Math.toRadians(0);
    public static double triangleHeading = Math.toRadians(20);
    public static double gateHeading = Math.toRadians(-140);



    public void buildPaths(Follower follower) {
        startToSemiMiddle = follower.pathBuilder()
                .addPath(new BezierCurve(
                        startToSemiMiddleStartPose,
                        new Pose(87, 40.100111234705224).mirror(),
                        startToSemiMiddleEndPose
                ))
                .setTangentHeadingInterpolation()
                .build();
        semiMiddleToFirstRowOfBall = follower.pathBuilder()
                .addPath(new BezierLine(
                        startToSemiMiddleEndPose,
                        semiMiddleToFirstRowOfBallEndPose
                ))
                .setTangentHeadingInterpolation()
                .build();

        firstRowOfBallToTriangle = follower.pathBuilder()
                .addPath(new BezierCurve(
                        semiMiddleToFirstRowOfBallEndPose,
                        new Pose(25.500, 51.000).mirror(),
                        new Pose(67.000, 49.500).mirror(),
                        firstRowOfBallToTriangleEndPose
                ))
                .setLinearHeadingInterpolation(follower.getHeading(), triangleHeading)
                .build();

        triangleToParking = follower.pathBuilder()
                .addPath(new BezierLine(
                        firstRowOfBallToTriangleEndPose,
                        triangleToParkingEndPose
                ))
                .setTangentHeadingInterpolation()
                .build();
    }

    public void autonomousUpdate(){
        switch (pathState) {
            case 0:
                if (pathTimer.getElapsedTimeSeconds()>2) {
                    robot.sorting.respectMotif = robot.sorting.getNumberOfBalls() == 3 && robot.sorting.getGreen() != 0;
                    robot.sorting.setNextState(Constants.MOVING_STATES.SHOOTING);
                    setPathState(1);
                }
                break;
            case 1:
                if (pathTimer.getElapsedTimeSeconds()>3.2) {
                    follower.followPath(startToSemiMiddle);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()){
                    robot.sorting.setNextState(Constants.MOVING_STATES.WAITING_INTAKE);
                    robot.intake.toggle();
                    setPathState(3);
                }
                break;
            case 3:
                if (pathTimer.getElapsedTimeSeconds()>0.1) {
                    follower.setMaxPower(0.4);
                    follower.followPath(semiMiddleToFirstRowOfBall);
                    setPathState(4);

                }
                break;
            case 4:
                if (pathTimer.getElapsedTimeSeconds()>0.2)
                    setPathState(5);
                break;
            case 5:
                if(!follower.isBusy()){
                    follower.setMaxPower(1);
                    follower.followPath(firstRowOfBallToTriangle);
                    robot.turret.setHorizontalPositionFromAngle(Math.toRadians(40));
                    Turret.setTarget_rotation(52);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy() || pathTimer.getElapsedTimeSeconds()>3) {
                    robot.intake.toggle(Constants.INTAKE_STATES.STOPPED);
                    robot.sorting.respectMotif = robot.sorting.getNumberOfBalls() == 3 && robot.sorting.getGreen() != 0;
                    robot.sorting.setNextState(Constants.MOVING_STATES.SHOOTING);
                    setPathState(7);
                }
                break;
            case 7:
                if (pathTimer.getElapsedTimeSeconds()>4) {
                    follower.followPath(triangleToParking);
                    setPathState(8);
                }
                break;
            case 8:
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
        robot.turret.setHorizontalPositionFromAngle(Math.toRadians(-20));
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        buildPaths(follower);
        robot.sorting.respectMotif = true;
        follower.setStartingPose(startToSemiMiddleStartPose);


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
