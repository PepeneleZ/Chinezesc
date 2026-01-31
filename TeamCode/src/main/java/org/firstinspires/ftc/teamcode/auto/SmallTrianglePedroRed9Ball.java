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
public class SmallTrianglePedroRed9Ball extends OpMode {
    private Follower follower;
    private Robot_Auto robot;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    public PathChain startToSemiMiddle;
    public PathChain semiMiddleToFirstRowOfBall;
    public PathChain firstRowOfBallToTriangle;
    public PathChain triangleToSemiSecondRowOfBall;
    public PathChain semiSecondRowOfBallToSecondRowOfBall;
    public PathChain secondRowOfBallsToTriangle;
    public PathChain triangleToSemiThirdRowOfBalls;
    public PathChain semiThirdRowOfBallsToThirdRowOfBalls;
    public PathChain thirdRowOfBallsToTriangle;
    public PathChain triangleToParking;


    public static Pose startToSemiMiddleStartPose = new Pose(99,8,Math.toRadians(90));
    public static Pose startToSemiMiddleEndPose = new Pose(107, 21);
    public static Pose semiMiddleToFirstRowOfBallEndPose = new Pose(134.5,21);

    public static Pose firstRowOfBallToTriangleEndPose = new Pose(102, 12);
    public static Pose triangleToSemiSecondRowOfBallsEndPose = new Pose(105, 45);
    public static Pose semiSecondRowOfBallsToSecondRowOfBallsEndPose = new Pose(134,45);
    public static Pose secondRowOfBallsToTriangleEndPose = new Pose(89, 71);
    public static Pose triangleToSemiThirdRowOfBallsEndPose= new Pose(103,71);
    public static Pose semiThirdRowOfBallsToThirdRowOfBallsEndPose= new Pose(125,70);
    public static Pose thirdRowOfBallsEndToTriangleEndPose = new Pose(89,71);
    public static Pose triangleToParkingEndPose = new Pose(119,69);


    public static double startHeading = Math.toRadians(90);
    public static double rowOfBallsHeading = Math.toRadians(0);
    public static double triangleHeading = Math.toRadians(20);
    public static double gateHeading = Math.toRadians(-140);


    public void buildPaths(Follower follower) {
        startToSemiMiddle = follower.pathBuilder()
                .addPath(new BezierLine(
                        startToSemiMiddleStartPose,
                        startToSemiMiddleEndPose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(90),Math.toRadians(0))
                .build();
        semiMiddleToFirstRowOfBall = follower.pathBuilder()
                .addPath(new BezierLine(
                        startToSemiMiddleEndPose,
                        semiMiddleToFirstRowOfBallEndPose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0),Math.toRadians(1))
                .build();

        firstRowOfBallToTriangle = follower.pathBuilder()
                .addPath(new BezierLine(
                        semiMiddleToFirstRowOfBallEndPose,
                        firstRowOfBallToTriangleEndPose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(1), Math.toRadians(90))
                .build();

        triangleToSemiSecondRowOfBall = follower.pathBuilder()
                .addPath(new BezierLine(
                        firstRowOfBallToTriangleEndPose,
                        triangleToSemiSecondRowOfBallsEndPose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(90),Math.toRadians(0))
                .build();
        semiSecondRowOfBallToSecondRowOfBall = follower.pathBuilder()
                .addPath(new BezierLine(
                        triangleToSemiSecondRowOfBallsEndPose,
                        semiSecondRowOfBallsToSecondRowOfBallsEndPose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0),Math.toRadians(1))
                .build();

        secondRowOfBallsToTriangle = follower.pathBuilder()
                .addPath(new BezierCurve(
                        semiSecondRowOfBallsToSecondRowOfBallsEndPose,
                        new Pose(100,40),
                        secondRowOfBallsToTriangleEndPose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(1), Math.toRadians(20))

                .build();
        triangleToSemiThirdRowOfBalls = follower.pathBuilder()
                .addPath(new BezierLine(
                        secondRowOfBallsToTriangleEndPose,
                        triangleToSemiThirdRowOfBallsEndPose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(20), Math.toRadians(0))

                .build();
        semiThirdRowOfBallsToThirdRowOfBalls = follower.pathBuilder()
                .addPath(new BezierLine(
                        triangleToSemiThirdRowOfBallsEndPose,
                        semiThirdRowOfBallsToThirdRowOfBallsEndPose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(1))

                .build();
        thirdRowOfBallsToTriangle = follower.pathBuilder()
                .addPath(new BezierLine(
                        semiThirdRowOfBallsToThirdRowOfBallsEndPose,
                        thirdRowOfBallsEndToTriangleEndPose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(1))

                .build();
        triangleToParking = follower.pathBuilder()
                .addPath(new BezierLine(
                        thirdRowOfBallsEndToTriangleEndPose,
                        triangleToParkingEndPose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(10))

                .build();

    }


    public void autonomousUpdate(){
        switch (pathState) {
            case 0:
                if(pathTimer.getElapsedTimeSeconds()>0.5)
                    robot.turret.setHorizontalPositionFromAngle(Math.toRadians(30));
                if (pathTimer.getElapsedTimeSeconds()>1)
                    robot.turret.setHorizontalPositionFromAngle(Math.toRadians(19));
                if (pathTimer.getElapsedTimeSeconds()>1.5)
                    robot.turret.setHorizontalPositionFromAngle(Math.toRadians(-20.5));

                if (pathTimer.getElapsedTimeSeconds()>2) {
                    robot.sorting.respectMotif = robot.sorting.getNumberOfBalls() == 3 && robot.sorting.getGreen() != 0;
                    robot.sorting.setNextState(Constants.MOVING_STATES.SHOOTING);
                    setPathState(1);
                }
                break;
            case 1:
                if (pathTimer.getElapsedTimeSeconds()>2.75) {
                    follower.setMaxPower(0.8);
                    follower.followPath(startToSemiMiddle);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()){
                    robot.sorting.setNextState(Constants.MOVING_STATES.WAITING_INTAKE);
                    robot.intake.toggle(Constants.INTAKE_STATES.COLLECTING);
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
                if(!follower.isBusy()){
                    follower.setMaxPower(0.9);
                    follower.followPath(firstRowOfBallToTriangle);
                    robot.turret.setHorizontalPositionFromAngle(Math.toRadians(-20));
                    Turret.setTarget_rotation(56);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy() || pathTimer.getElapsedTimeSeconds()>3) {
                    robot.intake.toggle(Constants.INTAKE_STATES.STOPPED);
                    robot.sorting.respectMotif = robot.sorting.getNumberOfBalls() == 3 && robot.sorting.getGreen() != 0;
                    robot.sorting.setNextState(Constants.MOVING_STATES.SHOOTING);
                    setPathState(6);
                }
                break;
            case 6:
                if (pathTimer.getElapsedTimeSeconds()>2.8){
                    follower.followPath(triangleToSemiSecondRowOfBall);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()){
                    follower.setMaxPower(0.4);
                    robot.intake.toggle(Constants.INTAKE_STATES.COLLECTING);
                    robot.sorting.setNextState(Constants.MOVING_STATES.WAITING_INTAKE);
                    follower.followPath(semiSecondRowOfBallToSecondRowOfBall);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()){
                    robot.turret.setVerticalPositionFromAngle(Math.toRadians(49));
                    Turret.setTarget_rotation(43);
                    robot.turret.setHorizontalPositionFromAngle(Math.toRadians(16));
                    follower.setMaxPower(0.9);
                    follower.followPath(secondRowOfBallsToTriangle);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()){
                    robot.intake.toggle(Constants.INTAKE_STATES.STOPPED);
                    robot.sorting.respectMotif = robot.sorting.getNumberOfBalls() == 3 && robot.sorting.getGreen() != 0;
                    robot.sorting.setNextState(Constants.MOVING_STATES.SHOOTING);
                    setPathState(10);
                }
                break;
            case 10:
                if (pathTimer.getElapsedTimeSeconds()>2.6){
                    robot.intake.toggle(Constants.INTAKE_STATES.COLLECTING);
                    robot.sorting.setNextState(Constants.MOVING_STATES.WAITING_INTAKE);
                    follower.followPath(triangleToSemiThirdRowOfBalls);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()){
                    follower.setMaxPower(0.4);
                    follower.followPath(semiThirdRowOfBallsToThirdRowOfBalls);
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()){
                    follower.setMaxPower(1);
                    robot.turret.setHorizontalPositionFromAngle(Math.toRadians(45));
                    follower.followPath(thirdRowOfBallsToTriangle);
                    setPathState(13);
                }
                break;
            case 13:
                if (!follower.isBusy()){
                    robot.intake.toggle(Constants.INTAKE_STATES.STOPPED);
                    robot.sorting.respectMotif = robot.sorting.getNumberOfBalls() == 3 && robot.sorting.getGreen() != 0;
                    robot.sorting.setNextState(Constants.MOVING_STATES.SHOOTING);
                    setPathState(14);
                }
                break;
            case 14:
                if (pathTimer.getElapsedTimeSeconds()>0.5)
                    setPathState(15);
            case 15:
                if (pathTimer.getElapsedTimeSeconds()>2.6){
                    follower.followPath(triangleToParking);
                    setPathState(16);
                }
                break;
            case 16:
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
        robot.turret.setHorizontalPositionFromAngle(Math.toRadians(60));
        Turret.setTarget_rotation(55);
        setPathState(0);
    }
}
