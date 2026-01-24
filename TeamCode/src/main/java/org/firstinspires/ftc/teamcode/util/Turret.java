package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.lib.Feedforward;
import org.firstinspires.ftc.teamcode.util.Constants_Enums.TRANSFER_POS;
import org.firstinspires.ftc.teamcode.util.Constants_Enums.TURRET_LAUNCH_SPEEDS;
import org.firstinspires.ftc.teamcode.util.Constants_Enums.VERTICAL_TURRET_POSITIONS;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.VelocityPID;

@Config
public class Turret implements Updateable{

    public DcMotorEx turret_launch;
    private static Servo vertical_angle_servo;
   // private final VoltageSensor voltageSensor;
    private Telemetry telemetry;



    private static final int full_rotation = 600;

    public double target_rotation = 0;
    public double turret_launch_position = 0;
    public double turret_angle = 0;
    public static TURRET_LAUNCH_SPEEDS turret_launcher_state=TURRET_LAUNCH_SPEEDS.STOPPED;
    public static VERTICAL_TURRET_POSITIONS turret_vertical_state = VERTICAL_TURRET_POSITIONS.DOWN;
    public static double turret_vertical_position = 0;

    public static Feedforward feedforwardController = new Feedforward(0,0,0,0,0); // should be better without kD -> old kD: 0.0000004d
    public double power_of_launch;
    public double power_of_hrot;
    public static double changeable_target=1100;
    public static boolean runPid = false;
    public static final double admissible_error = 10;
    public static long ticks=0;

    // Discussion with the Big Alex -> Notes
    // (ticks/second * 2 * pi)/motor_resolution - this is omega (angular velocity)
    // multiply  omega with R - radius
    // euristical logic - to test things
    public Turret(HardwareMap hwmap, Telemetry telemetry){//, VoltageSensor voltageSensor){
        turret_launch = hwmap.get(DcMotorEx.class, HardwareConfig.turret_launch);
        vertical_angle_servo = hwmap.get(Servo.class, HardwareConfig.vertical_angle_servo);

        turret_launch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret_launch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret_launch.setDirection(DcMotorSimple.Direction.REVERSE);

        turret_launch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        turret_launcher_state = TURRET_LAUNCH_SPEEDS.STOPPED;
        //vertical_angle_servo.setPosition(turret_vertical_state.val);
        feedforwardController.reset();
        //this.voltageSensor = voltageSensor;
        setTarget_rotation(turret_launcher_state);


        this.telemetry = telemetry;

    }
    public static void setTarget_rotation(TURRET_LAUNCH_SPEEDS state){
        if (turret_launcher_state == state)
            turret_launcher_state = TURRET_LAUNCH_SPEEDS.STOPPED;
        else
            turret_launcher_state = state;
        feedforwardController.setTarget(turret_launcher_state.val);
        runPid = true;
        ticks = 0;
    }
    public static void setTarget_rotation(int target){
        if (target == TURRET_LAUNCH_SPEEDS.FAR.val)
            turret_launcher_state = TURRET_LAUNCH_SPEEDS.FAR;
        else if (target == TURRET_LAUNCH_SPEEDS.STOPPED.val)
            turret_launcher_state = TURRET_LAUNCH_SPEEDS.STOPPED;
        else if (target == TURRET_LAUNCH_SPEEDS.CLOSE.val)
            turret_launcher_state = TURRET_LAUNCH_SPEEDS.CLOSE;
        else
            turret_launcher_state = TURRET_LAUNCH_SPEEDS.CUSTOM;
        feedforwardController.setTarget(target);
        runPid = true;
        ticks = 0;
    }

    public static void setVertical_position(double target_position){
        vertical_angle_servo.setPosition(target_position);
        turret_vertical_position = target_position;
        turret_vertical_state = VERTICAL_TURRET_POSITIONS.OTHER;
    }
    public static void setVertical_position(VERTICAL_TURRET_POSITIONS state){
        vertical_angle_servo.setPosition(state.val);
        turret_vertical_state = state;
        turret_vertical_position = state.val;
    }
    public static void setVertical_position(){
        if (turret_vertical_state == VERTICAL_TURRET_POSITIONS.DOWN) {
            vertical_angle_servo.setPosition(VERTICAL_TURRET_POSITIONS.UP.val);
            turret_vertical_state = VERTICAL_TURRET_POSITIONS.UP;
        }
        else {
            vertical_angle_servo.setPosition(VERTICAL_TURRET_POSITIONS.DOWN.val);
            turret_vertical_state = VERTICAL_TURRET_POSITIONS.DOWN;
        }
        turret_vertical_position = turret_vertical_state.val;
    }
    public static void increaseChangeableTarget(){
        changeable_target +=50;
    }
    public static void decreaseChangeableTarget(){
        changeable_target -=50;
    }
//28

    @Override
    public void update() {
        turret_launch_position = turret_launch.getCurrentPosition();
        power_of_launch = feedforwardController.update(turret_launch_position);
        //power_of_launch = power_of_launch * (14/voltageSensor.getVoltage());

        if (runPid) {
            turret_launch.setPower(power_of_launch);
        }
        else
            turret_launch.setPower(0);
//        if (feedforwardController.targetVelocity== TURRET_LAUNCH_SPEEDS.STOPPED.val && Math.abs(feedforwardController.currentVelocity)<admissible_error && ticks>10){
//            runPid = false;
//        }
        ticks++;



        turret_angle = turret_launch_position /full_rotation*360;

        update_telemetry();
    }



    public void update_telemetry(){
        //telemetry.addData("TURRET ANGLE: ", turret_angle);
        telemetry.addData("TURRET VELOCITY: ",feedforwardController.currentVelocity);
        telemetry.addData("TURRET ACCELERATION: ",feedforwardController.acceleration);
        telemetry.addData("TURRET POS: ",turret_launch.getCurrentPosition());
        telemetry.addData("RUN TURRET PID",runPid);
        telemetry.addData("TURRET TARGET VELOCITY: ",feedforwardController.targetVelocity);
        telemetry.addData("TURRET CHANGEABLE TARGET:", changeable_target);
        telemetry.addLine("-----------------------------------------------------"); // separate the mechanisms to make the text easier to read


    }
}
