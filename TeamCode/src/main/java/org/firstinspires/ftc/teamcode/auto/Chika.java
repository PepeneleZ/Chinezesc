package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
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
public class Chika extends OpMode {
    private Follower follower;
    private Robot_Auto robot;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    public PathChain startToEnd;
    public PathChain endToStart;

    public static Pose startPose = new Pose(56,8,Math.toRadians(90));
    public static Pose endPose = new Pose(5,8);

    public static double startHeading = Math.toRadians(180);

    public void buildPaths(Follower follower){
        startToEnd = follower.pathBuilder()
                .addPath(new BezierLine(
                        startPose,
                        endPose
                ))
                .setTangentHeadingInterpolation()
                .build();
        endToStart = follower.pathBuilder()
                .addPath(new BezierLine(
                        endPose,
                        startPose
                ))
                .setTangentHeadingInterpolation()
                .build();
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
        follower.setStartingPose(startPose);
    }

    @Override
    public void loop() {
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
                if (!follower.isBusy()){
                    robot.sorting.setNextState(Constants.MOVING_STATES.WAITING_INTAKE);
                    robot.intake.toggle();
                    setPathState(2);
                }
                break;
            case 2:
                if (pathTimer.getElapsedTimeSeconds()>0.1) {
                    follower.setMaxPower(0.4);
                    follower.followPath(startToEnd);
                    setPathState(3);

                }
                break;
            case 3:
                if(!follower.isBusy()){
                    follower.setMaxPower(1);
                    follower.followPath(endToStart);
                    robot.turret.setHorizontalPositionFromAngle(Math.toRadians(-60));
                    Turret.setTarget_rotation(52);
                    setPathState(4);
                }
            case 4:
                if(!follower.isBusy() || pathTimer.getElapsedTimeSeconds()>3) {
                    robot.intake.toggle(Constants.INTAKE_STATES.STOPPED);
                    robot.sorting.respectMotif = robot.sorting.getNumberOfBalls() == 3 && robot.sorting.getGreen() != 0;
                    robot.sorting.setNextState(Constants.MOVING_STATES.SHOOTING);
                    setPathState(5);
                }
        }
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
