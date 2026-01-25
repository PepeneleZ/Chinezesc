package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class Robot implements Updateable{
    public Drivetrain driveTrain;
    public Intake intake;
    public Storage storage;
    public Sorting sorting;
    public Turret turret;
    public Limelight limelight;
    public PinpointLocalizer pinpointLocalizer;
    public VoltageSensor voltageSensor;
    public Pose robotPose;
    public Pose aprilPose = Constants.redAprilTagPose;

    public Robot(HardwareMap hwmap, Telemetry telemetry){
        voltageSensor = hwmap.getAll(VoltageSensor.class).get(0);
        driveTrain = new Drivetrain(hwmap);
        intake = new Intake(hwmap);
        sorting = new Sorting(hwmap,telemetry,intake,voltageSensor);
        limelight = new Limelight(hwmap,telemetry);
        pinpointLocalizer = new PinpointLocalizer(hwmap, Constants.localizerConstants);

        //storage = new Storage(hwmap,telemetry, intake);
        //turret = new Turret(hwmap, telemetry,voltageSensor);
    }
    public void setColor(boolean color){ // true for blue, false for red
        if (color){
            aprilPose = Constants.blueAprilTagPose;
            limelight.colorPipeline = limelight.bluePipeline;
            Turret.lebronPose = Constants.lebronPoseBlue;
        }
        else {
            aprilPose = Constants.redAprilTagPose;
            limelight.colorPipeline = limelight.redPipeline;
            Turret.lebronPose = Constants.lebronPoseRed;
        }
    }
    public void adjustPositions(){
        LLResult llresult = limelight.getResult();
        Pose3D llpose3D;
        Pose llpose, turretCenterPose, robotPose, localizerPose;
        if (llresult==null || !llresult.isValid()) {
            return;
        }
        llpose3D = llresult.getBotpose();
        localizerPose = pinpointLocalizer.getPose();
        llpose = new Pose(llpose3D.getPosition().x+aprilPose.getX(),llpose3D.getPosition().y+aprilPose.getY(),turret.getHorizontalAngle());

        double tx,ty;
        tx = Constants.turretRadius * Math.cos(llpose.getHeading());
        ty = Constants.turretRadius * Math.sin(llpose.getHeading());
        turretCenterPose = new Pose(llpose.getX()-tx, llpose.getY()-ty,llpose3D.getOrientation().getYaw(AngleUnit.RADIANS) - Math.toRadians(45) - llpose.getHeading());

        double robotPoseX,robotPoseY;
        robotPoseX = turretCenterPose.getX() - (Constants.turretCenterOffsetX * Math.cos(localizerPose.getHeading()) - Constants.turretCenterOffsetY * Math.sin(localizerPose.getHeading()));
        robotPoseY = turretCenterPose.getY() - (Constants.turretCenterOffsetX * Math.sin(localizerPose.getHeading()) + Constants.turretCenterOffsetY * Math.cos(localizerPose.getHeading()));
        robotPose = new Pose(robotPoseX,robotPoseY,localizerPose.getHeading());

        turret.turret_pose = turretCenterPose;
        pinpointLocalizer.setPose(robotPose);
    }



    @Override
    public void update() {
        //storage.update();
        pinpointLocalizer.update();
        sorting.update();
        turret.update();
        intake.update();
        limelight.update();

        robotPose = pinpointLocalizer.getPose();
    }
}
