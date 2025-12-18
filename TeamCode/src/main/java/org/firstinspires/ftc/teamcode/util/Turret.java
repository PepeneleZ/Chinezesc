package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.VelocityPID;

@Config
public class Turret implements Updateable{

    private DcMotorEx turret_launch;
    private CRServo turret_hrot;
    private Telemetry telemetry;


    private static final int full_rotation = 600;

    public double target_rotation = 0;
    public double turret_launch_position = 0;
    public double turret_angle = 0;
    public static TURRET_LAUNCH_SPEEDS turret_launcher_state=TURRET_LAUNCH_SPEEDS.STOPPED;

    public static VelocityPID velocityPID = new VelocityPID(0.000157d,0.000007d,0.000003d);
    public double power_of_launch;
    public double power_of_hrot;

    public Turret(HardwareMap hwmap, Telemetry telemetry){
        turret_launch = hwmap.get(DcMotorEx.class, HardwareConfig.turret_launch);
        turret_hrot = hwmap.get(CRServo.class,HardwareConfig.turret_hrot);

        turret_launch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret_launch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turret_launch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        this.telemetry = telemetry;

    }
    public void setTarget_rotation(TURRET_LAUNCH_SPEEDS state){
        if (turret_launcher_state == state)
            turret_launcher_state = TURRET_LAUNCH_SPEEDS.STOPPED;
        else
            turret_launcher_state = state;
        velocityPID.setTargetVelocity(turret_launcher_state.val);
    }


    public static enum TURRET_LAUNCH_SPEEDS{
        CLOSE(2000),FAR(3000),STOPPED(0);
        double val;
        TURRET_LAUNCH_SPEEDS(double val) {
            this.val = val;
        }
    }
    public void setTarget_rotation(double target_rotation){
        velocityPID.setTargetVelocity(target_rotation);
    }


    @Override
    public void update() {
        turret_launch_position = turret_launch.getCurrentPosition();
        power_of_launch = velocityPID.update(turret_launch_position);

        turret_launch.setPower(power_of_launch);
        turret_angle = turret_launch_position /full_rotation*360;


        update_telemetry();
    }



    public void update_telemetry(){
        //telemetry.addData("Unghiul Turetei", turret_angle);
        telemetry.addData("TURRET VELOCITY: ",velocityPID.currentVelocity);
        telemetry.addData("TURRET_POWER",power_of_launch);
        telemetry.addData("TURRET POS: ", turret_launch_position);
        telemetry.addData("TURRET TARGET VELOCITY: ",velocityPID.targetVelocity);

    }
}
