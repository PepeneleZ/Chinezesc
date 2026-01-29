package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@Config
public class Robot implements Updateable{
    public Telemetry telemetry;
    public Drivetrain driveTrain;
    public Intake intake;
    //public Storage storage;
    public Sorting sorting;
    public Turret turret;
    public Limelight limelight;
    public PinpointLocalizer pinpointLocalizer;
    public VoltageSensor voltageSensor;
    public Pose robotPose, newRobotPose, llpose;
    public Pose aprilPose = Constants.redAprilTagPose;
    private Pose3D llpose3D;
    private Pose turretCenterPose, localizerPose;
    public static double radius = 4.528,turretoffsetx=-2.9527,turretoffsety=0.1968;
    private double robotPoseX=0,robotPoseY=0,robotPoseHeading=0, turretHorizontalAngle=0;

    public Robot(HardwareMap hwmap, Telemetry telemetry){
        voltageSensor = hwmap.getAll(VoltageSensor.class).get(0);
        turret = new Turret(hwmap,telemetry);
        driveTrain = new Drivetrain(hwmap);
        intake = new Intake(hwmap);
        sorting = new Sorting(hwmap,telemetry,intake,voltageSensor);
        limelight = new Limelight(hwmap,telemetry);
        pinpointLocalizer = new PinpointLocalizer(hwmap, Constants.localizerConstants);

        pinpointLocalizer.setStartPose(new Pose(72,72,0));
        robotPose = new Pose(72,72,0);
        newRobotPose = new Pose(0,0,0);
        llpose = new Pose(0,0,0);
        turretCenterPose = new Pose(0,0,0);
        localizerPose = new Pose(0,0,0);
        this.telemetry = telemetry;

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
    public void adjustPositionsWithLimelight(boolean adjust_angle){ // adjust_angle = true => adjusts the pinpoint heading as well based on the value read by the limelight
        /*
        Pose3D llpose3D;
        Pose llpose, turretCenterPose, newRobotPose, localizerPose;
        double robotPoseX,robotPoseY,robotPoseHeading;
*/


        localizerPose = pinpointLocalizer.getPose();
        turretHorizontalAngle = turret.getHorizontalAngle();

        double tx,ty;
        tx = radius * Math.cos(turretHorizontalAngle);
        ty = radius * Math.sin(turretHorizontalAngle);
        turretCenterPose = new Pose(llpose.getX()-tx, llpose.getY()-ty,llpose.getHeading());

        if (adjust_angle)
            robotPoseHeading = normalizeRadians(turretCenterPose.getHeading() - llpose.getHeading()); // trebuie vazut la scaderea cu 45
        else
            robotPoseHeading = localizerPose.getHeading();

        robotPoseX = turretCenterPose.getX() - (turretoffsetx * Math.cos(robotPoseHeading) - turretoffsety * Math.sin(robotPoseHeading));
        robotPoseY = turretCenterPose.getY() - (turretoffsetx * Math.sin(robotPoseHeading) + turretoffsety * Math.cos(robotPoseHeading));

        newRobotPose = new Pose(robotPoseX,robotPoseY,robotPoseHeading);

        turret.turret_pose = turretCenterPose;
        //pinpointLocalizer.setPose(newRobotPose);
    }

    public void adjustTurretPosition(){
        Pose turretPose;
        double x,y,heading;
        x = robotPose.getX() + (Constants.turretCenterOffsetX * Math.cos(robotPose.getHeading()) - Constants.turretCenterOffsetY * Math.sin(robotPose.getHeading()));
        y = robotPose.getY() + (Constants.turretCenterOffsetX * Math.sin(robotPose.getHeading()) + Constants.turretCenterOffsetY * Math.cos(robotPose.getHeading()));
        heading = robotPose.getHeading() - turret.getHorizontalAngle();
        turretPose = new Pose(x,y,heading);
        turret.turret_pose = turretPose;
    }
    public void aimLebron(){
        adjustTurretPosition();
        turret.aimHorizontalLebron();
    }
    private Pose turnLLCordsToPedroCords(Pose3D limelightpose){
        double x,y,heading;
        y = -limelightpose.getPosition().x*39.3701+72;
        x = limelightpose.getPosition().y*39.3701+72;
        heading = normalizeRadians(limelightpose.getOrientation().getYaw(AngleUnit.RADIANS) + 4.36332313);
        return new Pose(x,y,heading);
    }



    @Override
    public void update() {
        //storage.update();
        pinpointLocalizer.update();
        sorting.update();
        turret.update();
        intake.update();
        limelight.update();
        llpose = turnLLCordsToPedroCords(limelight.llpose3d);
//        limelight.limelight.updateRobotOrientation(robotPose.getHeading());
        robotPose = pinpointLocalizer.getPose();
        adjustPositionsWithLimelight(false);

//        llpose3D = limelight.getResult().getBotpose();
//        llpose = new Pose(llpose3D.getPosition().x+aprilPose.getX(),llpose3D.getPosition().y+aprilPose.getY(),llpose3D.getOrientation().getYaw(AngleUnit.RADIANS));
        sorting.telemetryData();
        turret.update_telemetry();
        telemetryData();

    }

    public void telemetryData(){
        outputPose("Pozitia curenta a robotului", pinpointLocalizer.getPose());
        outputPose("New robot pose: ",newRobotPose);
        outputPose("llpose", llpose);
//        outputPose("Limelight: ",limelight.getResult().getBotpose());
        outputPose("Limelight localizare: ", limelight.llpose3d);
        telemetry.addData("Returned turret horizontal angle: ", turretHorizontalAngle);
    }
    public double normalizeRadians(double radians){
        return Math.atan2(Math.sin(radians),Math.cos(radians));
    }
    public void outputPose(String name,Pose pose){
        if(!pose.initialized()) return;
        telemetry.addData(name+"  X:",pose.getX());
        telemetry.addData(name+"  Y:",pose.getY());
        telemetry.addData(name+"  HEADING:",pose.getHeading());
        telemetry.addLine("--------------------------------------------------"); // separate the mechanisms to make the text easier to read


    }
    public void outputPose(String name,Pose3D pose){
        telemetry.addData(name+"  X:",pose.getPosition().x);
        telemetry.addData(name+"  Y:",pose.getPosition().y);
        telemetry.addData(name+"  HEADING:",pose.getOrientation().getYaw());
        telemetry.addLine("--------------------------------------------------"); // separate the mechanisms to make the text easier to read


    }
}
