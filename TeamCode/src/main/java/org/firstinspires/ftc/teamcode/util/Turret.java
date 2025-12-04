package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Turret implements Updateable{

    private DcMotorEx turret_launch;
    private CRServo turret_hrot;
    private Telemetry telemetry;


    private static final int full_rotation = 600;

    public double target_rotation = 0;
    public double turret_position = 0;
    public double turret_angle = 0;
    private static final PIDFCoefficients pidfCoefficients= new PIDFCoefficients(0.35,0.02,0.01,0);
    private PIDFController pidfController = new PIDFController(pidfCoefficients);
    public Turret(HardwareMap hwmap, Telemetry telemetry){
        turret_launch = hwmap.get(DcMotorEx.class, HardwareConfig.turret_launch);
        turret_hrot = hwmap.get(CRServo.class,HardwareConfig.turret_hrot);

        this.telemetry = telemetry;

    }
    public void setTarget_rotation(int target){
        pidfController.reset();
        pidfController.setTargetPosition(target);

    }
    public void launch(){
        turret_launch.setPower(1);
    }

    @Override
    public void update() {
        //Functie de citire a pozitiei
        pidfController.updatePosition(turret_position);
        double power = pidfController.run();

        turret_angle = turret_position/full_rotation*360;

        update_telemetry();
    }



    public void update_telemetry(){
        telemetry.addData("Unghiul Turetei", turret_angle);
    }
}
