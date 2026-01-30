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
public class BigTrianglePedroRed9Ball extends OpMode {
    private Follower follower;
    private Robot_Auto robot;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    public PathChain startToShootingPosition;
    public PathChain shootingPositionToThirdRow;
    public PathChain thirdRowToShootingPosition;
    public PathChain shootingPositionToSemiSecondRowPosition;
    public PathChain semiSecondRowToSecondRowPosition;
    public PathChain secondRowToShootingPosition;
    public PathChain shootingPositionToParking;
    public static double startHeading = Math.toRadians(0);


    public static Pose startToShootingPositionStartPose = new Pose(32.516, 135.9911012235817).mirror();
    public static Pose startToShootingPositionEndPose = new Pose(49.593, 84.053).mirror();
    public static Pose shootingPositionToThirdRowEndPose = new Pose(10, 84.4).mirror();

    public static Pose thirdRowToShootingPositionEndPose = new Pose(49.427, 84.075).mirror();
    public static Pose shootingPositionToSemiSecondRowEndPose = new Pose(42.576, 59.5).mirror();
    public static Pose semiSecondRowToSecondRowEndPose = new Pose(3, 59.5).mirror();
    public static Pose secondRowToShootingPositionEndPose = new Pose (57.819, 74.009).mirror();
    public static Pose shootingPositionToParkingEndPose = new Pose (30, 74.009).mirror();


    public void buildPaths(Follower follower) {
        startToShootingPosition = follower.pathBuilder().addPath(
                        new BezierLine(
                                startToShootingPositionStartPose,

                                startToShootingPositionEndPose
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90),startHeading)

                .build();

        shootingPositionToThirdRow = follower.pathBuilder().addPath(
                        new BezierLine(
                                startToShootingPositionEndPose,

                                shootingPositionToThirdRowEndPose
                        )
                ).setLinearHeadingInterpolation(startHeading,Math.toRadians(11))

                .build();

        thirdRowToShootingPosition = follower.pathBuilder().addPath(
                        new BezierLine(
                                shootingPositionToThirdRowEndPose,

                                thirdRowToShootingPositionEndPose
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(11),Math.toRadians(0))
                .build();

        shootingPositionToSemiSecondRowPosition = follower.pathBuilder().addPath(
                        new BezierCurve(
                                thirdRowToShootingPositionEndPose,
                                new Pose(66.951, 59.635).mirror(),
                                shootingPositionToSemiSecondRowEndPose
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0),startHeading)

                .build();

        semiSecondRowToSecondRowPosition = follower.pathBuilder().addPath(
                        new BezierLine(
                                shootingPositionToSemiSecondRowEndPose,

                                semiSecondRowToSecondRowEndPose
                        )
                ).setLinearHeadingInterpolation(startHeading,Math.toRadians(15))

                .build();
        secondRowToShootingPosition = follower.pathBuilder().addPath(
                        new BezierLine(
                                semiSecondRowToSecondRowEndPose,

                                secondRowToShootingPositionEndPose
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(15),Math.toRadians(0))

                .build();
        shootingPositionToParking = follower.pathBuilder().addPath(
                        new BezierLine(
                                secondRowToShootingPositionEndPose,
                                shootingPositionToParkingEndPose
                        )
                ).setTangentHeadingInterpolation()
                .build();

    }


    public void autonomousUpdate() {
        switch (pathState){
            case 0:
                if (!follower.isBusy()){
                    robot.turret.setHorizontalPositionFromAngle(Math.toRadians(40));
                    setPathState(1);
                }
                break;

            case 1:
                if (pathTimer.getElapsedTimeSeconds()>0.8){
                    robot.sorting.respectMotif = robot.sorting.getNumberOfBalls() == 3 && robot.sorting.getGreen() != 0;
                    robot.sorting.setNextState(Constants.MOVING_STATES.SHOOTING);
                    setPathState(2);
                }
                break;
            case 2:
                if (pathTimer.getElapsedTimeSeconds()>3.2){
                    follower.setMaxPower(0.35);
                    follower.followPath(shootingPositionToThirdRow);
                    robot.sorting.setNextState(Constants.MOVING_STATES.WAITING_INTAKE);
                    robot.intake.toggle(Constants.INTAKE_STATES.COLLECTING);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()){
                    follower.setMaxPower(1);
                    follower.followPath(thirdRowToShootingPosition);
                    Turret.setTarget_rotation(45);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    robot.sorting.respectMotif = robot.sorting.getNumberOfBalls() == 3 && robot.sorting.getGreen() != 0;
                    robot.sorting.setNextState(Constants.MOVING_STATES.SHOOTING);
                    setPathState(5);
                }
                break;
            case 5:
                if (pathTimer.getElapsedTimeSeconds()>3.2){
                    follower.followPath(shootingPositionToSemiSecondRowPosition);
                    robot.sorting.setNextState(Constants.MOVING_STATES.WAITING_INTAKE);
                    robot.intake.toggle(Constants.INTAKE_STATES.COLLECTING);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()){
                    follower.setMaxPower(0.35);
                    follower.followPath(semiSecondRowToSecondRowPosition);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()){
                    follower.setMaxPower(1);
                    Turret.setTarget_rotation(47);
                    follower.followPath(secondRowToShootingPosition);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()){
                    robot.sorting.respectMotif = robot.sorting.getNumberOfBalls() == 3 && robot.sorting.getGreen() != 0;
                    robot.sorting.setNextState(Constants.MOVING_STATES.SHOOTING);
                    setPathState(9);
                }
                break;
            case 9:
                if (pathTimer.getElapsedTimeSeconds()>3.2){
                    follower.followPath(shootingPositionToParking);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
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
        robot = new Robot_Auto(hardwareMap, telemetry);
        robot.sorting.fillMagazine();
        robot.turret.setHorizontalPositionFromAngle(Math.toRadians(-16.4));
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        buildPaths(follower);

        follower.setStartingPose(new Pose(startToShootingPositionStartPose.getX(),startToShootingPositionStartPose.getY(),Math.toRadians(90)));


    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        robot.turret.setVerticalPositionFromAngle(Math.toRadians(45));
        robot.turret.setHorizontalPositionFromAngle(Math.toRadians(60));
//        Turret.setTarget_rotation(58);
        Turret.setTarget_rotation(45);
        follower.followPath(startToShootingPosition);
        setPathState(0);
    }
}