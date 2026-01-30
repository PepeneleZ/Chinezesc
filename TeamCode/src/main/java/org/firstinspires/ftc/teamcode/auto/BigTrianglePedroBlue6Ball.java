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
public class BigTrianglePedroBlue6Ball extends OpMode {
    private Follower follower;
    private Robot_Auto robot;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    public PathChain startToShootingPosition;
    public PathChain shootingPositionToThirdRow;
    public PathChain thirdRowToShootingPosition;
    public PathChain shootingPositionToParking;
    public static double startHeading = Math.toRadians(180);


    public static Pose startToShootingPositionStartPose = new Pose(32.516, 135.9911012235817);
    public static Pose startToShootingPositionEndPose = new Pose(49.593, 84.053);
    public static Pose shootingPositionToThirdRowEndPose = new Pose(9.1, 83.4);

    public static Pose thirdRowToShootingPositionEndPose = new Pose(49.427, 84.075);
    public static Pose shootingPositionToParkingEndPose = new Pose (30, 74.009);


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
                ).setLinearHeadingInterpolation(startHeading,Math.toRadians(169))

                .build();

        thirdRowToShootingPosition = follower.pathBuilder().addPath(
                        new BezierLine(
                                shootingPositionToThirdRowEndPose,

                                thirdRowToShootingPositionEndPose
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(169),Math.toRadians(181))
                .build();

        shootingPositionToParking = follower.pathBuilder().addPath(
                        new BezierLine(
                                thirdRowToShootingPositionEndPose,
                                shootingPositionToParkingEndPose
                        )
                ).setTangentHeadingInterpolation()
                .build();

    }


    public void autonomousUpdate() {
        switch (pathState){
            case 0:
                if (!follower.isBusy()){
                    robot.turret.setHorizontalPositionFromAngle(Math.toRadians(-50));
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
                    follower.followPath(shootingPositionToParking);
                    setPathState(6);
                }
                break;
            case 6:
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
        robot = new Robot_Auto(hardwareMap, telemetry);
        robot.sorting.fillMagazine();
        robot.turret.setHorizontalPositionFromAngle(Math.toRadians(16.4));
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
        robot.turret.setHorizontalPositionFromAngle(Math.toRadians(-60));
//        Turret.setTarget_rotation(58);
        Turret.setTarget_rotation(45);
        follower.followPath(startToShootingPosition);
        setPathState(0);
    }
}