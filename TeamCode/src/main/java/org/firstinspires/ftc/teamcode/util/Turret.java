package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.lib.Feedforward;
import org.firstinspires.ftc.teamcode.util.Constants.TURRET_LAUNCH_SPEEDS;
import org.firstinspires.ftc.teamcode.util.Constants.VERTICAL_TURRET_POSITIONS;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Turret implements Updateable{

    public DcMotorEx turret_launch;
    public DcMotorEx launch_encoder;
    public Pose turret_pose;
    private static Servo vertical_angle_servo;
    private static Servo left_hrot_servo, right_hrot_servo;
    //private static DcMotorEx hrot_encoder;
    private final VoltageSensor voltageSensor;
    private Telemetry telemetry;
    public double delta_horizontal_angle;
    public double angleBetweenRobotAndLebron;


    public double turret_launch_position = 0;
    public static TURRET_LAUNCH_SPEEDS turret_launcher_state=TURRET_LAUNCH_SPEEDS.STOPPED;
    public static VERTICAL_TURRET_POSITIONS turret_vertical_state = VERTICAL_TURRET_POSITIONS.UP;
    public static Pose lebronPose = Constants.lebronPoseRed;
    public static double turret_vertical_position = 0;
    public static final double ticksPerLaunchRotation=8192;


    //public static Feedforward feedforwardController = new Feedforward(0,0.0071d,0.2d,0.0316,5); // should be better without kD -> old kD: 0.0000004d
    public static Feedforward feedforwardController = new Feedforward(0.02,0.01d,0.015,0.131,40); // should be better without kD -> old kD: 0.0000004d
    public double power_of_launch;
    public static boolean runPid = false;
    public static final double admissible_error = 15;
    public static long ticks=0;

    // Discussion with the Big Alex -> Notes
    // (ticks/second * 2 * pi)/motor_resolution - this is omega (angular velocity)
    // multiply  omega with R - radius
    // euristical logic - to test things
    public Turret(HardwareMap hwmap, Telemetry telemetry, VoltageSensor voltageSensor){
        turret_launch = hwmap.get(DcMotorEx.class, HardwareConfig.turret_launch);
        launch_encoder = hwmap.get(DcMotorEx.class,HardwareConfig.RB);
        //hrot_encoder = hwmap.get(DcMotorEx.class, HardwareConfig.back_lifter);
        vertical_angle_servo = hwmap.get(Servo.class, HardwareConfig.vertical_angle_servo);
        left_hrot_servo = hwmap.get(Servo.class, HardwareConfig.turret_hrot1);
        right_hrot_servo = hwmap.get(Servo.class,HardwareConfig.turret_hrot2);

        turret_launch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret_launch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret_launch.setDirection(DcMotorSimple.Direction.FORWARD);

        //launch_encoder.setDirection(DcMotorSimple.Direction.REVERSE);
        launch_encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launch_encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //vertical_angle_servo.setDirection(Servo.Direction.REVERSE);

        turret_launch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        turret_launcher_state = TURRET_LAUNCH_SPEEDS.STOPPED;
        vertical_angle_servo.setPosition(turret_vertical_state.val);

        left_hrot_servo.setDirection(Servo.Direction.REVERSE);
        right_hrot_servo.setDirection(Servo.Direction.REVERSE);
        left_hrot_servo.setPosition(0.5);
        right_hrot_servo.setPosition(0.5);
        feedforwardController.reset();
        this.voltageSensor = voltageSensor;
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
    public double getVerticalAngle(){
        return verticalPositionToPercentage(vertical_angle_servo.getPosition())
                * Constants.turretVerticalAngle
                + Constants.deltaTurretVerticalAngle;
    }
    public double getVerticalPositionFromAngle(double angle){
        angle = normalizeAngle(angle);
        return Range.clip(percentageToVerticalPosition(
                Range.clip((angle-Constants.deltaTurretVerticalAngle)/Constants.turretVerticalAngle,0,1)),
                VERTICAL_TURRET_POSITIONS.DOWN.val, VERTICAL_TURRET_POSITIONS.UP.val);
    }
    public double verticalPositionToPercentage(double power){
        return (power-Constants.turretVerticalMinimumPower)/(1-Constants.turretVerticalMinimumPower);
    }
    public double percentageToVerticalPosition(double percentage){ //between 0 and 1
        return percentage*(1-Constants.turretVerticalMinimumPower)+Constants.turretVerticalMinimumPower;
    }
    public void setVerticalPositionFromAngle(double angle){
        vertical_angle_servo.setPosition(Range.clip(getVerticalPositionFromAngle(angle), VERTICAL_TURRET_POSITIONS.DOWN.val,VERTICAL_TURRET_POSITIONS.UP.val));
    }


///////////////////////// HORIZONTAL
    public double getHorizontalPositionFromAngle(double angle){ //in radians
        angle = normalizeAngle(angle);
        return Range.clip(angle/Constants.turretHorizontalAngle + (1/2.0),0,1);
    }
    public double getHorizontalAngleFromPosition(double position){
        return (position - (1/2.0))
                * Constants.turretHorizontalAngle;
    }
    public double getHorizontalAngle(){ // should return a normalized angle
        return (left_hrot_servo.getPosition() -(1/2.0)) * Constants.turretHorizontalAngle ;
    }
    public void setHorizontalPositionFromAngle(double angle) {
        angle = getHorizontalPositionFromAngle(angle);
        left_hrot_servo.setPosition(angle);
        right_hrot_servo.setPosition(angle);
    }




//////// LEBRON
    public void aimLebron() {
        aimVerticalLebron();
        aimHorizontalLebron();
    }
    public void aimVerticalLebron(){
        double cateta = Constants.lebronHeight-Constants.turretCenterOffsetZ;
        double ipotenuza = turret_pose.distanceFrom(lebronPose);
        double vertical_angle = Math.asin(cateta/ipotenuza);
        setVerticalPositionFromAngle(vertical_angle);
    }

    public void aimHorizontalLebron(){
        double dx = lebronPose.getX() - turret_pose.getX();
        double dy = lebronPose.getY() - turret_pose.getY();
        angleBetweenRobotAndLebron = Math.atan2(dy, dx);


        delta_horizontal_angle = angleBetweenRobotAndLebron - turret_pose.getHeading();
        double horizontal_target = Range.clip(getHorizontalPositionFromAngle(delta_horizontal_angle),0,1);
        right_hrot_servo.setPosition(horizontal_target);
        left_hrot_servo.setPosition(horizontal_target);
    }

    private double normalizeAngle(double angle){
        // 1. Force the angle into the -pi to pi range first
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle <= -Math.PI) angle += 2 * Math.PI;

        // 2. Now run your original formula
        return angle;
    }


//28

    @Override
    public void update() {
        turret_launch_position = launch_encoder.getCurrentPosition()/ticksPerLaunchRotation;
        power_of_launch = feedforwardController.update(turret_launch_position);
        power_of_launch = power_of_launch * (12.4/voltageSensor.getVoltage());
        if (runPid) {
            turret_launch.setPower(power_of_launch);
        }
        else
            turret_launch.setPower(0);
        if (feedforwardController.targetVelocity==TURRET_LAUNCH_SPEEDS.STOPPED.val && Math.abs(feedforwardController.currentVelocity)<admissible_error && ticks>10){
            if (runPid)
                feedforwardController.reset();

            runPid = false;
        }
        ticks++;
        //update_telemetry();
    }



    public void update_telemetry(){
        //telemetry.addData("TURRET ANGLE: ", turret_angle);
        telemetry.addData("TURRET VELOCITY: ",feedforwardController.velocitySum/feedforwardController.list_size);
        telemetry.addData("TURRET ACCELERATION: ",feedforwardController.acceleration);
        telemetry.addData("TURRET POS: ",turret_launch_position);
        telemetry.addData("RUN TURRET PID",runPid);
        telemetry.addData("TURRET TARGET VELOCITY: ",feedforwardController.targetVelocity);
        telemetry.addData("power of launch: ",power_of_launch);
        telemetry.addData("Profiled acceleration: ",feedforwardController.profiledAcceleration);
        telemetry.addData("Profiled velocity: ",feedforwardController.profiledVelocity);
        telemetry.addData("LebroPoseX",lebronPose.getX());
        telemetry.addLine("--------------------------------------------------"); // separate the mechanisms to make the text easier to read


    }
}
