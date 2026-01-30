package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Constants {

    /// /////////////// SORTING /////////////// ///
    public enum TRANSFER_POS{
        UP(1),DOWN(0);
        final double val;
        TRANSFER_POS(double val) {
            this.val = val;
        }
    }
    public enum MOVING_STATES {
        MOVING("moving"),WAITING_INTAKE("waiting_intake"), NOTHING("nothing"), SHOOTING("shooting");
        final String val;
        MOVING_STATES(String val){this.val = val;}
    }

    public enum COLORS{
        GREEN("green"), PURPLE("parpal"), EMPTY("empty");
        public final String val;
        COLORS(String val) {
            this.val = val;
        }
    }




    /// /////////////// TURRET /////////////// ///
    public enum TURRET_LAUNCH_SPEEDS{
        CLOSE(45),FAR(52),STOPPED(0), CUSTOM(40);
        final double val;
        TURRET_LAUNCH_SPEEDS(double val) {
            this.val = val;
        }
    }
    public enum VERTICAL_TURRET_POSITIONS{
        DOWN(0.7),MIDDLE(0.85),UP(1), OTHER(0.9);
        final double val;
        VERTICAL_TURRET_POSITIONS(double val){this.val = val;}
    }

    public enum MOTIF {
        GPP(1), PGP(2), PPG(3);
        final int val;
        MOTIF(int val){this.val = val;}
    }


    /// /////////////// INTAKE /////////////// ///
    public enum INTAKE_STATES{
        STOPPED(0),COLLECTING(1),SPITTING_OUT(-1),SLIGHTLY_MOVING(0.4);
        final double val;
        INTAKE_STATES(double val){this.val = val;}
    }


    /// /////////////// ROBOT CONSTANTS /////////////// ///
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-7.087d)
            .strafePodX(2.559d)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName(HardwareConfig.pinpoint_calculator)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    /// Turret ///
    public static final double turretCenterOffsetY = -2.3d;
    public static final double turretCenterOffsetZ = 5d;
    public static final double turretRadius = 5.09;
    //public static final double deltaTurretHorizontalAngle =Math.toRadians(30);
    public static final double turretHorizontalAngle = Math.toRadians(120);
    public static final double turretVerticalAngle = Math.toRadians(40);
    public static final double turretVerticalMinimumPower = 0.7;
    public static final double deltaTurretVerticalAngle = Math.toRadians(30);
    public static final double turretTotalHorizontalTicks = 8192;
    public static final double full_circle = Math.PI*2;



    /// Field ///
    public static final Pose redAprilTagPose = new Pose(130,130,Math.toRadians(45));
    public static final Pose blueAprilTagPose = new Pose(14,130,Math.toRadians(-45));
    public static double lebronPoseRedX=144, lebronPoseRedY=144;
    public static double lebronPoseBlueX=0, lebronPoseBlueY=144;

    public static  Pose lebronPoseRed = new Pose(lebronPoseRedX,lebronPoseRedY,0);
    public static  Pose lebronPoseBlue = new Pose(lebronPoseBlueX,lebronPoseBlueY,0);
    public static final double lebronHeight = 53;

}
