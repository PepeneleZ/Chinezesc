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
    public TURRET_LAUNCH_SPEEDS turret_launcher_state=TURRET_LAUNCH_SPEEDS.STOPPED;

    private static final PIDFCoefficients pidfCoefficients= new PIDFCoefficients(0.35,0.02,0.01,0);
    private PIDFController pidfController = new PIDFController(pidfCoefficients);
    public double power_of_motor=0.18;

    public Turret(HardwareMap hwmap, Telemetry telemetry){
        turret_launch = hwmap.get(DcMotorEx.class, HardwareConfig.turret_launch);
        turret_hrot = hwmap.get(CRServo.class,HardwareConfig.turret_hrot);

        turret_launch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret_launch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        turret_launch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        this.telemetry = telemetry;

    }
    public void setTarget_rotation(int target){
        pidfController.reset();
        pidfController.setTargetPosition(target);
    }
    public void increasePower(){
        if(power_of_motor+0.1<1){
            power_of_motor+=0.1;
        }
    }
    public void decreasePower(){
        if(power_of_motor-0.1>-1){
            power_of_motor-=0.1;
        }
    }
    public enum TURRET_LAUNCH_SPEEDS{
        CLOSE(1000),FAR(1200),STOPPED(0);
        double val;
        TURRET_LAUNCH_SPEEDS(double val) {
            this.val = val;
        }
    }
    public void shoot_far(){
        if(turret_launcher_state!=TURRET_LAUNCH_SPEEDS.FAR)
            turret_launcher_state = TURRET_LAUNCH_SPEEDS.FAR;
        else
            turret_launcher_state = TURRET_LAUNCH_SPEEDS.STOPPED;
        turret_launch.setVelocity(turret_launcher_state.val);
    }
    public void shoot_close(){
        if (turret_launcher_state!=TURRET_LAUNCH_SPEEDS.CLOSE)
            turret_launcher_state = TURRET_LAUNCH_SPEEDS.CLOSE;
        else
            turret_launcher_state = TURRET_LAUNCH_SPEEDS.STOPPED;
        turret_launch.setVelocity(turret_launcher_state.val);
    }


    @Override
    public void update() {
        //Functie de citire a pozitiei
//        pidfController.updatePosition(turret_position);
//        double power = pidfController.run();

        turret_angle = turret_position/full_rotation*360;


        update_telemetry();
    }



    public void update_telemetry(){
        //telemetry.addData("Unghiul Turetei", turret_angle);
        telemetry.addData("TURRET_POWER",power_of_motor);
        telemetry.addData("TURRET POS: ",turret_launch.getCurrentPosition());
    }
}
