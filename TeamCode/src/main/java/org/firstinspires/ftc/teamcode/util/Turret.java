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
    public CRServo turret_hrot;
    private Telemetry telemetry;
    public DcMotorEx od1, od2;


    private static final int full_rotation = 600;

    public double target_rotation = 0;
    public double turret_launch_position = 0;
    public double turret_angle = 0;
    public static TURRET_LAUNCH_SPEEDS turret_launcher_state=TURRET_LAUNCH_SPEEDS.STOPPED;

    public static VelocityPID velocityPID = new VelocityPID(0.0026d,0.0005d,0.0000004d);
    public double power_of_launch;
    public double power_of_hrot;
    public static double changeable_target=1100;
    public static boolean runPid = false;
    public static final double admissible_error = 10;
    public static long ticks=0;

    public Turret(HardwareMap hwmap, Telemetry telemetry){
        turret_launch = hwmap.get(DcMotorEx.class, HardwareConfig.turret_launch);
        turret_hrot = hwmap.get(CRServo.class,HardwareConfig.turret_hrot);

        turret_launch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret_launch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turret_launch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        turret_launcher_state = TURRET_LAUNCH_SPEEDS.STOPPED;
        velocityPID.resetPid();
        setTarget_rotation(turret_launcher_state);
        od1 = hwmap.get(DcMotorEx.class,HardwareConfig.front_lifter);
        od2 = hwmap.get(DcMotorEx.class,HardwareConfig.RB);


        this.telemetry = telemetry;

    }
    public static void setTarget_rotation(TURRET_LAUNCH_SPEEDS state){
        if (turret_launcher_state == state)
            turret_launcher_state = TURRET_LAUNCH_SPEEDS.STOPPED;
        else
            turret_launcher_state = state;
        velocityPID.setTargetVelocity(turret_launcher_state.val);
        runPid = true;
        ticks = 0;
    }


    public static enum TURRET_LAUNCH_SPEEDS{
        CLOSE(1100),FAR(1645),STOPPED(0);
        final double val;
        TURRET_LAUNCH_SPEEDS(double val) {
            this.val = val;
        }
    }
    public static void setTarget_rotation(double target_rotation){
        velocityPID.setTargetVelocity(target_rotation);
        runPid = true;
        ticks=0;

    }
    public static void setSpecialTarget_rotation(){
        velocityPID.setTargetVelocity(changeable_target);
        runPid = true;
        ticks=0;
    }
    public static void increaseChangeableTarget(){
        changeable_target +=50;
    }
    public static void decreaseChangeableTarget(){
        changeable_target -=50;
    }


    @Override
    public void update() {
        turret_launch_position = turret_launch.getCurrentPosition();
        power_of_launch = velocityPID.update(turret_launch_position);

        if (runPid) {
            turret_launch.setPower(power_of_launch);
        }
        else
            turret_launch.setPower(0);
        if (velocityPID.targetVelocity== TURRET_LAUNCH_SPEEDS.STOPPED.val && Math.abs(velocityPID.currentVelocity)<admissible_error && ticks>10){
            runPid = false;
        }
        ticks++;



        turret_angle = turret_launch_position /full_rotation*360;

        update_telemetry();
    }



    public void update_telemetry(){
        //telemetry.addData("Unghiul Turetei", turret_angle);
        telemetry.addData("TURRET VELOCITY: ",velocityPID.currentVelocity);
        telemetry.addData("RUN TURRET PID",runPid);
        telemetry.addData("TURRET TARGET VELOCITY: ",velocityPID.targetVelocity);
        telemetry.addData("TURRET CHANGEABLE TARGET:", changeable_target);
        telemetry.addData("ODOMETRY 1: ",od1.getCurrentPosition());
        telemetry.addData("ODOMETRY 2: ",od2.getCurrentPosition());


    }
}
