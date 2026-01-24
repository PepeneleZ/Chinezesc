package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot implements Updateable{
    public Drivetrain driveTrain;
    public Intake intake;
    public Storage storage;
    public Turret turret;
    public VoltageSensor voltageSensor;

    public Robot(HardwareMap hwmap, Telemetry telemetry){
        voltageSensor = hwmap.getAll(VoltageSensor.class).get(0);
        driveTrain = new Drivetrain(hwmap);
        intake = new Intake(hwmap);
        storage = new Storage(hwmap,telemetry, intake);
        //turret = new Turret(hwmap, telemetry,voltageSensor);
    }
    @Override
    public void update() {
        storage.update();
        turret.update();
        intake.update();
    }
}
