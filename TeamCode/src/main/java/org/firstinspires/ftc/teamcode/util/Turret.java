package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Turret implements Updateable{

    private DcMotorEx back_turret, front_turret;
    private CRServo turret_hrot, turret_vrot;
    private Telemetry telemetry;


    private static final int full_rotation = 600;

    public double target_rotation = 0;
    public double turret_position = 0;
    public double turret_angle = 0;
    private static final PIDFCoefficients pidfCoefficients= new PIDFCoefficients(0.35,0.02,0.01,0);
    private PIDFController pidfController = new PIDFController(pidfCoefficients);
    public Turret(HardwareMap hwmap, Telemetry telemetry){
        back_turret = hwmap.get(DcMotorEx.class, HardwareConfig.back_turret);
        front_turret = hwmap.get(DcMotorEx.class, HardwareConfig.front_turret);
        turret_hrot = hwmap.get(CRServo.class,HardwareConfig.launcher_hrot);
        turret_vrot = hwmap.get(CRServo.class,HardwareConfig.launcher_vrot);

        back_turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        front_turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        this.telemetry = telemetry;

    }
    public void setTarget_rotation(int target){
        pidfController.reset();
        pidfController.setTargetPosition(target);

    }
    public void launch(){
        back_turret.setPower(1);
        front_turret.setPower(1);
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
