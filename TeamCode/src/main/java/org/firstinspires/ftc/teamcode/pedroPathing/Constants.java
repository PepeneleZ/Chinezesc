package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.HardwareConfig;


// model auto
//[(56.0, 8.0), (60.700, 38.700), (21.0, 35.0), (25.500, 51.0), (67.0, 49.500), (72.0, 72.0), (70.400, 90.0), (50.0, 54.400), (25.200, 60.300), (22.0, 71.700), (72.0, 72.0), (87.500, 48.200), (5.200, 45.500), (9.900, 60.300), (56.600, 67.0), (72.0, 72.0)]
public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10);
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName(HardwareConfig.RF)
            .leftFrontMotorName(HardwareConfig.LF)
            .rightRearMotorName(HardwareConfig.RB)
            .leftRearMotorName(HardwareConfig.LB)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD);

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

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(5.688976378d)
            .strafePodX(0d)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName(HardwareConfig.pinpoint_calculator)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                //.threeWheelIMULocalizer(localizerConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
