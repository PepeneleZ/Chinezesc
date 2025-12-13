package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot implements Updateable{
    public Drivetrain driveTrain;
    public Intake intake;
    public Sorting sorting;
    public Turret turret;

    public Robot(HardwareMap hwmap, Telemetry telemetry){
        driveTrain = new Drivetrain(hwmap);
        intake = new Intake(hwmap);
        sorting = new Sorting(hwmap,telemetry, intake);
        turret = new Turret(hwmap, telemetry);
    }
    @Override
    public void update() {
        intake.update();
        sorting.update();
        turret.update();
    }
}
