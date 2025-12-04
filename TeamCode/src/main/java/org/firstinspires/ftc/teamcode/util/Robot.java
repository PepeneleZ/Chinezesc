package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot implements Updateable{
    public Drivetrain driveTrain;
    public Intake intake;
    //public Turret turret;

    public Robot(HardwareMap hwmap, Telemetry telemetry){
        driveTrain = new Drivetrain(hwmap);
        intake = new Intake(hwmap);
        //turret = new Turret(hwmap, telemetry);
    }
    @Override
    public void update() {

    }
}
