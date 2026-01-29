package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.PedroConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Tuning;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Robot_Auto;
import org.firstinspires.ftc.teamcode.util.Turret;

@Autonomous
public class SmallTrianglePedro extends OpMode {
    private Follower follower;
    private Robot_Auto robot;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    public PathChain startToSemiMiddle;
    public PathChain semiMiddleToFirstRowOfBall;
    public PathChain firstRowOfBallToTriangle;
    public PathChain triangleToSecondRowOfBalls;
    public PathChain secondRowOfBallsToTriangle;
    public PathChain triangleToOpenGateCycle;
    public PathChain gateToTriangleCycle;

    public PathChain twoRowsAutonomySegment; // this segment basically combines all of the pathchains above this line into one big PathChain

    public static Pose startToSemiMiddleStartPose = new Pose(56,8,Math.toRadians(90));
    public static Pose startToSemiMiddleEndPose = new Pose(43, 36.000);
    public static Pose semiMiddleToFirstRowOfBallEndPose = new Pose(10,36);

    public static Pose firstRowOfBallToTriangleEndPose = new Pose(72.000, 72.000);
    public static Pose triangleToSecondRowOfBallsEndPose = new Pose(25.200, 60.300);
    public static Pose secondRowOfBallsToTriangleEndPose = new Pose(72.000, 72.000);
    public static Pose triangletoOpenGateCycleEndPose= new Pose(9.900, 60.300);
    public static Pose gateToTriangleCycleEndPose=new Pose(72.000, 72.000);

    public static double startHeading = Math.toRadians(90);
    public static double rowOfBallsHeading = Math.toRadians(180);
    public static double triangleHeading = Math.toRadians(200);
    public static double gateHeading = Math.toRadians(140);



    public void buildPaths(Follower follower) {
        startToSemiMiddle = follower.pathBuilder()
                .addPath(new BezierCurve(
                    startToSemiMiddleStartPose,
                    new Pose(87, 40.100111234705224),
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
                    new Pose(25.500, 51.000),
                    new Pose(67.000, 49.500),
                    firstRowOfBallToTriangleEndPose
                ))
                .setLinearHeadingInterpolation(follower.getHeading(), triangleHeading)
                .build();

        triangleToSecondRowOfBalls = follower.pathBuilder()
                .addPath(new BezierCurve(
                    firstRowOfBallToTriangleEndPose,
                    new Pose(70.400, 90.000),
                    new Pose(50.000, 54.400),
                    triangleToSecondRowOfBallsEndPose
                ))
                .setLinearHeadingInterpolation(triangleHeading, rowOfBallsHeading)

                .build();

        secondRowOfBallsToTriangle = follower.pathBuilder()
                .addPath(new BezierCurve(
                    triangleToSecondRowOfBallsEndPose,
                    new Pose(22.000, 71.700),
                    secondRowOfBallsToTriangleEndPose
                ))
                .setLinearHeadingInterpolation(rowOfBallsHeading, triangleHeading)

                .build();

        triangleToOpenGateCycle = follower.pathBuilder()
                .addPath(new BezierCurve(
                    secondRowOfBallsToTriangleEndPose,
                    new Pose(87.500, 48.200),
                    new Pose(5.200, 45.500),
                    triangletoOpenGateCycleEndPose
                ))
                .setLinearHeadingInterpolation(triangleHeading, gateHeading)

                .build();

        gateToTriangleCycle = follower.pathBuilder()
                .addPath(new BezierCurve(
                    triangletoOpenGateCycleEndPose,
                    new Pose(56.600, 67.000),
                    gateToTriangleCycleEndPose
                ))
                .setLinearHeadingInterpolation(gateHeading, triangleHeading)

                .build();
    }

    public void buildPaths2(Follower follower){ // the big version with the twoRowsAutonomySegment PathChain
        twoRowsAutonomySegment = follower.pathBuilder()
                //startToFirstRowOfBalls
                .addPath(new BezierCurve(
                        new Pose(56.000, 8.000),
                        new Pose(60.700, 38.700),
                        new Pose(21.000, 35.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

                //firstRowOfBallToTriangle
                .addPath(new BezierCurve(
                        new Pose(21.000, 35.000),
                        new Pose(25.500, 51.000),
                        new Pose(67.000, 49.500),
                        new Pose(72.000, 72.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(200))
                .setReversed()

                //triangleToSecondRowOfBalls
                .addPath(new BezierCurve(
                        new Pose(72.000, 72.000),
                        new Pose(70.400, 90.000),
                        new Pose(50.000, 54.400),
                        new Pose(25.200, 60.300)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(180))

                //secondRowOfBallsToTriangle
                .addPath(new BezierCurve(
                        new Pose(25.200, 60.300),
                        new Pose(22.000, 71.700),
                        new Pose(72.000, 72.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(190))


                .build();
    }
    public void autonomousUpdate(){
        switch (pathState) {
            case 0:
                if (pathTimer.getElapsedTimeSeconds()>2) {
                    setPathState(1);
                    robot.sorting.setNextState(Constants.MOVING_STATES.SHOOTING);

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
                    robot.turret.setHorizontalPositionFromAngle(Math.toRadians(65));
                    Turret.setTarget_rotation(52);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy() || pathTimer.getElapsedTimeSeconds()>3) {
                    robot.intake.toggle(Constants.INTAKE_STATES.STOPPED);
                    robot.sorting.setNextState(Constants.MOVING_STATES.SHOOTING);
                    setPathState(7);
                }
                break;
            case 7:
                if (pathTimer.getElapsedTimeSeconds()>4)
                    setPathState(-1);
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
        robot.turret.setHorizontalPositionFromAngle(Math.toRadians(-16.4));
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        buildPaths(follower);
        robot.sorting.respectMotif = false;
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
