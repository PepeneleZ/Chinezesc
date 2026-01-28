package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

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
    public Pose robotPose;
    public Pose aprilPose = Constants.redAprilTagPose;
    private Pose3D llpose3D;
    private Pose llpose, turretCenterPose, newRobotPose, localizerPose;
    private double robotPoseX,robotPoseY,robotPoseHeading, turretHorizontalAngle;

    public Robot(HardwareMap hwmap, Telemetry telemetry){
        voltageSensor = hwmap.getAll(VoltageSensor.class).get(0);
        turret = new Turret(hwmap,telemetry);
        driveTrain = new Drivetrain(hwmap);
        intake = new Intake(hwmap);
        sorting = new Sorting(hwmap,telemetry,intake,voltageSensor);
        limelight = new Limelight(hwmap,telemetry);
        pinpointLocalizer = new PinpointLocalizer(hwmap, Constants.localizerConstants);

        pinpointLocalizer.setStartPose(new Pose(72,72,0));
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
        LLResult llresult = limelight.getResult();

        if (llresult==null || !llresult.isValid()) {
            return;
        }
        llpose3D = llresult.getBotpose();
        localizerPose = pinpointLocalizer.getPose();
        turretHorizontalAngle = turret.getHorizontalAngle();
        llpose = new Pose(llpose3D.getPosition().x+aprilPose.getX(),llpose3D.getPosition().y+aprilPose.getY(),llpose3D.getOrientation().getYaw(AngleUnit.RADIANS));

        double tx,ty;
        tx = Constants.turretRadius * Math.cos(turretHorizontalAngle);
        ty = Constants.turretRadius * Math.sin(turretHorizontalAngle);
        turretCenterPose = new Pose(llpose.getX()-tx, llpose.getY()-ty,llpose.getHeading());

        if (adjust_angle)
            robotPoseHeading = MathFunctions.normalizeAngle(turretCenterPose.getHeading() - Math.toRadians(45) - llpose.getHeading()); // trebuie vazut la scaderea cu 45
        else
            robotPoseHeading = localizerPose.getHeading();

        robotPoseX = turretCenterPose.getX() - (Constants.turretCenterOffsetX * Math.cos(robotPoseHeading) - Constants.turretCenterOffsetY * Math.sin(robotPoseHeading));
        robotPoseY = turretCenterPose.getY() - (Constants.turretCenterOffsetX * Math.sin(robotPoseHeading) + Constants.turretCenterOffsetY * Math.cos(robotPoseHeading));

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
        turret.aimLebron();
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


        sorting.telemetryData();
        turret.update_telemetry();
        //telemetryData();

    }

    public void telemetryData(){
        outputPose("Pozitia curenta a robotului", robotPose);
        outputPose("Pozitia noua a robotului", newRobotPose);
        outputPose("Limelight: ",llpose);
        telemetry.addData("Returned turret horizontal angle: ", turretHorizontalAngle);


    }
    public void outputPose(String name,Pose pose){
        telemetry.addData(name+"  X:",pose.getX());
        telemetry.addData(name+"  Y:",pose.getY());
        telemetry.addData(name+"  HEADING:",pose.getHeading());
        telemetry.addLine("--------------------------------------------------"); // separate the mechanisms to make the text easier to read


    }
}
