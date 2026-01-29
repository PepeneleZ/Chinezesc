package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.HardwareConfig;


// model auto
//[(56.0, 8.0), (60.700, 38.700), (21.0, 35.0), (25.500, 51.0), (67.0, 49.500), (72.0, 72.0), (70.400, 90.0), (50.0, 54.400), (25.200, 60.300), (22.0, 71.700), (72.0, 72.0), (87.500, 48.200), (5.200, 45.500), (9.900, 60.300), (56.600, 67.0), (72.0, 72.0)]
public class PedroConstants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(-29)
            .lateralZeroPowerAcceleration(-68)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.3,0,0.03,0))
            .headingPIDFCoefficients(new PIDFCoefficients(1.3,0,0.06,0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.0047,0,0,0.4,0))
            .centripetalScaling(0.0009)
            .mass(16);
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName(HardwareConfig.RF)
            .leftFrontMotorName(HardwareConfig.LF)
            .rightRearMotorName(HardwareConfig.RB)
            .leftRearMotorName(HardwareConfig.LB)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(70)
            .yVelocity(40);

//    public static ThreeWheelIMUConstants localizerConstants = new ThreeWheelIMUConstants()
//            .forwardTicksToInches(1)
//            .strafeTicksToInches(1)
//            .turnTicksToInches(1)
//            .leftPodY(5.688976378d)
//            .rightPodY(5.688976378d)
//            .strafePodX(3.72047244d)
//            .leftEncoder_HardwareMapName(HardwareConfig.RB)
//            .rightEncoder_HardwareMapName(HardwareConfig.back_lifter)
//            .strafeEncoder_HardwareMapName(HardwareConfig.front_lifter)
//            .IMU_HardwareMapName(HardwareConfig.imu)
//            .IMU_Orientation(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.UP));



    public static PathConstraints pathConstraints = new PathConstraints(0.99, 20, 1.8, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                //.threeWheelIMULocalizer(localizerConstants)
                .pinpointLocalizer(Constants.localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
