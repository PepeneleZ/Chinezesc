package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.lib.Feedforward;
import org.firstinspires.ftc.teamcode.util.Constants.TURRET_LAUNCH_SPEEDS;
import org.firstinspires.ftc.teamcode.util.Constants.VERTICAL_TURRET_POSITIONS;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Turret implements Updateable{

    public DcMotorEx turret_launch;
    public Pose turret_pose;
    private static Servo vertical_angle_servo;
    private static CRServo left_hrot_servo, right_hrot_servo;
    private static DcMotorEx hrot_encoder;
   // private final VoltageSensor voltageSensor;
    private Telemetry telemetry;


    public double turret_launch_position = 0;
    public double turret_horizontal_angle = 0;
    public double horizontal_target=0;
    public static TURRET_LAUNCH_SPEEDS turret_launcher_state=TURRET_LAUNCH_SPEEDS.STOPPED;
    public static VERTICAL_TURRET_POSITIONS turret_vertical_state = VERTICAL_TURRET_POSITIONS.DOWN;
    public static Pose lebronPose = Constants.lebronPoseRed;
    public static double turret_vertical_position = 0;

    public static Feedforward feedforwardController = new Feedforward(0,0,0,0,0); // should be better without kD -> old kD: 0.0000004d
    public double power_of_launch;
    public static boolean runPid = false;
    public static final double admissible_error = Constants.turretTotalHorizontalTicks/36;
    public static long ticks=0;

    // Discussion with the Big Alex -> Notes
    // (ticks/second * 2 * pi)/motor_resolution - this is omega (angular velocity)
    // multiply  omega with R - radius
    // euristical logic - to test things
    public Turret(HardwareMap hwmap, Telemetry telemetry){//, VoltageSensor voltageSensor){
        turret_launch = hwmap.get(DcMotorEx.class, HardwareConfig.turret_launch);
        hrot_encoder = hwmap.get(DcMotorEx.class, HardwareConfig.back_lifter);
        vertical_angle_servo = hwmap.get(Servo.class, HardwareConfig.vertical_angle_servo);
        left_hrot_servo = hwmap.get(CRServo.class, HardwareConfig.turret_hrot1);
        right_hrot_servo = hwmap.get(CRServo.class,HardwareConfig.turret_hrot2);

        turret_launch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret_launch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret_launch.setDirection(DcMotorSimple.Direction.REVERSE);

        turret_launch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        turret_launcher_state = TURRET_LAUNCH_SPEEDS.STOPPED;
        vertical_angle_servo.setPosition(turret_vertical_state.val);
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
    public double getVerticalAngle(){
        return (vertical_angle_servo.getPosition()*Constants.turretVerticalAngle + Constants.deltaTurretVerticalAngle);
    }
    public double getVerticalPositionFromAngle(double angle){
        return (angle-Constants.deltaTurretVerticalAngle)/Constants.turretVerticalAngle;
    }


    public double getHorizontalPositionFromAngle(double angle){ //in radians
        return (angle/(Constants.full_circle))
                * Constants.turretTotalHorizontalTicks;
    }
    public double getHorizontalAngleFromPosition(double position){
        return (position/Constants.turretTotalHorizontalTicks)
                *Constants.full_circle;
    }
    public double getHorizontalAngle(){
        return (turret_horizontal_angle/Constants.turretTotalHorizontalTicks)
                *Constants.full_circle;
    }
    public void aimLebron() {
        aimVerticalLebron();
        aimHorizontalLebron();
    }

    public void aimVerticalLebron(){
        double cateta = Constants.lebronHeight-Constants.turretCenterOffsetZ;
        double ipotenuza = turret_pose.distanceFrom(lebronPose);
        double vertical_angle = Math.asin(cateta/ipotenuza);
        double vertical_target = Range.clip(getVerticalPositionFromAngle(vertical_angle),0,1);
        vertical_angle_servo.setPosition(vertical_target);
    }

    public void aimHorizontalLebron(){
        double deltaX = lebronPose.getX()-turret_pose.getX();
        double deltaY = lebronPose.getY()-turret_pose.getY();
        double horizontal_angle = Math.atan2(deltaY,deltaX);
        horizontal_target = turret_horizontal_angle+horizontal_angle;

        if (horizontal_target>Constants.turretTotalHorizontalTicks){
            horizontal_target%=Constants.turretHorizontalAngle;
        }
    }

    public void unwind(){
        horizontal_target = 0;
    }

//28

    @Override
    public void update() {
        turret_launch_position = turret_launch.getCurrentPosition();
        turret_horizontal_angle = hrot_encoder.getCurrentPosition();
        power_of_launch = feedforwardController.update(turret_launch_position);
        //power_of_launch = power_of_launch * (14/voltageSensor.getVoltage());
        if (Math.abs(horizontal_target-turret_horizontal_angle)>admissible_error){
            double power = (horizontal_target>turret_horizontal_angle?1:-1);
            left_hrot_servo.setPower(power);
            right_hrot_servo.setPower(power);
        }
        if (turret_horizontal_angle>Constants.turretTotalHorizontalTicks*1.05)
            horizontal_target = 0;

        if (runPid) {
            turret_launch.setPower(power_of_launch);
        }
        else
            turret_launch.setPower(0);
//        if (feedforwardController.targetVelocity== TURRET_LAUNCH_SPEEDS.STOPPED.val && Math.abs(feedforwardController.currentVelocity)<admissible_error && ticks>10){
//            runPid = false;
//        }
        ticks++;
        update_telemetry();
    }



    public void update_telemetry(){
        //telemetry.addData("TURRET ANGLE: ", turret_angle);
        telemetry.addData("TURRET VELOCITY: ",feedforwardController.currentVelocity);
        telemetry.addData("TURRET ACCELERATION: ",feedforwardController.acceleration);
        telemetry.addData("TURRET POS: ",turret_launch.getCurrentPosition());
        telemetry.addData("RUN TURRET PID",runPid);
        telemetry.addData("TURRET TARGET VELOCITY: ",feedforwardController.targetVelocity);
        telemetry.addLine("-----------------------------------------------------"); // separate the mechanisms to make the text easier to read


    }
}
