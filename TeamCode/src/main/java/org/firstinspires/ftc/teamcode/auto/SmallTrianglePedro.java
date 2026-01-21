package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class SmallTrianglePedro extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    public PathChain startToFirstRowOfBalls;
    public PathChain firstRowOfBallToTriangle;
    public PathChain triangleToSecondRowOfBalls;
    public PathChain secondRowOfBallsToTriangle;
    public PathChain twoRowsAutonomySegment; // this segment basically combines all of the pathchains above this line into one big PathChain


    public PathChain triangleToOpenGateCycle;
    public PathChain gateToTriangleCycle;


    public void buildPaths(Follower follower) {
        startToFirstRowOfBalls = follower.pathBuilder()
                .addPath(new BezierCurve(
                    new Pose(56.000, 8.000),
                    new Pose(60.700, 38.700),
                    new Pose(21.000, 35.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();

        firstRowOfBallToTriangle = follower.pathBuilder()
                .addPath(new BezierCurve(
                    new Pose(21.000, 35.000),
                    new Pose(25.500, 51.000),
                    new Pose(67.000, 49.500),
                    new Pose(72.000, 72.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(225))
                .setReversed()
                .build();

        triangleToSecondRowOfBalls = follower.pathBuilder()
                .addPath(new BezierCurve(
                    new Pose(72.000, 72.000),
                    new Pose(70.400, 90.000),
                    new Pose(50.000, 54.400),
                    new Pose(25.200, 60.300)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(180))

                .build();

        secondRowOfBallsToTriangle = follower.pathBuilder()
                .addPath(new BezierCurve(
                    new Pose(25.200, 60.300),
                    new Pose(22.000, 71.700),
                    new Pose(72.000, 72.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(190))

                .build();

        triangleToOpenGateCycle = follower.pathBuilder()
                .addPath(new BezierCurve(
                    new Pose(72.000, 72.000),
                    new Pose(87.500, 48.200),
                    new Pose(5.200, 45.500),
                    new Pose(9.900, 60.300)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(190), Math.toRadians(140))

                .build();

        gateToTriangleCycle = follower.pathBuilder()
                .addPath(new BezierCurve(
                    new Pose(9.900, 60.300),
                    new Pose(56.600, 67.000),
                    new Pose(72.000, 72.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(190))

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
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(225))
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



    @Override
    public void loop() {

    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        buildPaths(follower);

    }
}
