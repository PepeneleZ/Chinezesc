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
public class Robot_Auto implements Updateable{
    public Telemetry telemetry;
    public Intake intake;
    //public Storage storage;
    public Sorting sorting;
    public Lifter lifter;
    public Turret turret;
    public Limelight limelight;
    public VoltageSensor voltageSensor;

    public Robot_Auto(HardwareMap hwmap, Telemetry telemetry){
        voltageSensor = hwmap.getAll(VoltageSensor.class).get(0);
        turret = new Turret(hwmap,telemetry);
        intake = new Intake(hwmap);
        lifter= new Lifter(hwmap,telemetry);
        sorting = new Sorting(hwmap,telemetry,intake,voltageSensor);
        limelight = new Limelight(hwmap,telemetry);

        this.telemetry = telemetry;

        //storage = new Storage(hwmap,telemetry, intake);
        //turret = new Turret(hwmap, telemetry,voltageSensor);
    }
    public void setColor(boolean color){ // true for blue, false for red
        if (color){
            //limelight.colorPipeline = limelight.bluePipeline;
            Turret.lebronPose = Constants.lebronPoseBlue;
        }
        else {
            //limelight.colorPipeline = limelight.redPipeline;
            Turret.lebronPose = Constants.lebronPoseRed;
        }
    }



    @Override
    public void update() {
        //storage.update();
        sorting.update();
        lifter.update();
        turret.update();
        intake.update();
        limelight.update();
//        limelight.limelight.updateRobotOrientation(robotPose.getHeading());

//        llpose3D = limelight.getResult().getBotpose();
//        llpose = new Pose(llpose3D.getPosition().x+aprilPose.getX(),llpose3D.getPosition().y+aprilPose.getY(),llpose3D.getOrientation().getYaw(AngleUnit.RADIANS));
       // sorting.telemetryData();
        //turret.update_telemetry();
//        telemetryData();

    }


}
