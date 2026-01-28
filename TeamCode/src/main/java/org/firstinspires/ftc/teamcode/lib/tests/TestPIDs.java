package org.firstinspires.ftc.teamcode.lib.tests;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.lib.Controller;
import org.firstinspires.ftc.teamcode.lib.PIDF;
import org.firstinspires.ftc.teamcode.util.HardwareConfig;

@TeleOp
@Config
public class TestPIDs extends LinearOpMode {

    public static int ballsNumber = 1;
    public static double kF = 0.125;
    public static PIDF pidf1 = new PIDF(0.0001d,0.000002d,0.000000d,() -> ballsNumber * kF); // for quarter rotations
    public static PIDF pidf2 = new PIDF(0.0001d,0.000002d,0.000000d,() -> ballsNumber * kF);
    public static double kF_forbothPids = 0;

    public Controller controller1, controller2;
    public CRServo servo_motor1, servo_motor2; // motor used
    public DcMotorEx motor_encoder; //encoder used
    public DcMotorEx motor;
    public VoltageSensor voltageSensor;
    public static double position=0,target, pidError=0;
    public static double delta_target1 = 8192/3.0, delta_target2 = 8192/6.0;
    public static int whichPid = 1;

    @Override
    public void runOpMode() throws InterruptedException {
       // servo_motor = hardwareMap.get(CRServo.class, HardwareConfig.sorting);
        servo_motor1 = hardwareMap.get(CRServo.class, HardwareConfig.sorting1);
        servo_motor2 = hardwareMap.get(CRServo.class, HardwareConfig.sorting2);
        motor_encoder = hardwareMap.get(DcMotorEx.class, HardwareConfig.right_lifter);
        //motor = hardwareMap.get(DcMotorEx.class, HardwareConfig.turret_launch);
        voltageSensor = hardwareMap.getAll(VoltageSensor.class).get(0);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);

        servo_motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        servo_motor2.setDirection(DcMotorSimple.Direction.FORWARD);
        motor_encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

      /*
        motor_encoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor_encoder.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_encoder.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

 */
        /*

        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

         */

        whichPid = 1; target = 0; position = 0; pidError=0;

        pidf1.resetPid();
        pidf2.resetPid();

        pidf1.setTargetPosition(target);
        pidf2.setTargetPosition(target);

        telemetry.addData("encoder", motor_encoder.getCurrentPosition());
        telemetry.update();

        waitForStart();
        while (opModeIsActive() && !isStopRequested()){
            //position = motor_encoder.getCurrentPosition();
            position = motor_encoder.getCurrentPosition();
            double power=0;

            if (controller1.triangle.isPressed() || controller2.triangle.isPressed()){
                pidf1.resetPid();
                pidf2.resetPid();
                if (whichPid==1) {
                    target += delta_target1;
                    pidf1.setTargetPosition(target);
                    pidf2.setTargetPosition(target);

                }
                else {
                    target += delta_target2;
                    pidf2.setTargetPosition(target);
                    pidf1.setTargetPosition(target);

                }
            }
            if (controller1.cross.isPressed() || controller2.cross.isPressed()){
                pidf1.resetPid();
                pidf2.resetPid();
                if (whichPid==1) {
                    target -= delta_target1;
                    pidf1.setTargetPosition(target);
                    pidf2.setTargetPosition(target);

                }
                else {
                    target -= delta_target2;
                    pidf2.setTargetPosition(target);
                    pidf1.setTargetPosition(target);

                }
            }
            if (controller1.circle.isPressed() || controller2.circle.isPressed()){
                if (whichPid==1) whichPid=2;
                else whichPid = 1;
                pidf1.resetPid();
                pidf2.resetPid();
            }
            if (whichPid==1) {
                power = pidf1.update(position);
                pidError = target-position;
            }
            else {
                power = pidf2.update(position);
                pidError = target-position;
            }

            if(Math.abs(pidError) > 100) {
                power = power * (14 / voltageSensor.getVoltage());
            } else {
                power = 0;
            }

           // servo_motor.setPower(power);
            servo_motor1.setPower(power);
            servo_motor2.setPower(power); // for now

            telemetry.addData("Position: ",position);
            telemetry.addData("Target ",target);
            telemetry.addData("Pidf error: ",pidError);
            telemetry.addData("Power: ", power);
            telemetry.addData("Which Pid: ", whichPid);
            telemetry.addData("Delta Target1 ", delta_target1);
            telemetry.addData("Delta Target2 ",delta_target2);

            telemetry.update();
            controller1.update();
            controller2.update();
        }
    }
}
