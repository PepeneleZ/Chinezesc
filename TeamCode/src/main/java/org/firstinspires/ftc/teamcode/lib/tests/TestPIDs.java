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
    public static PIDF pidf1 = new PIDF(0.00198,0.0002,2e-5d,0); // for quarter rotations
    public static PIDF pidf2 = new PIDF(0.009d,0.0004d,4e-6d,0);
    public static double kF_forbothPids = 0;

    public Controller controller1, controller2;
    public CRServo servo_motor; // motor used
    public DcMotorEx motor_encoder; //encoder used
    public DcMotorEx motor;
    public VoltageSensor voltageSensor;
    public static double position=0,target, pidError=0;
    public static double delta_target1 = 128.17, delta_target2 = 23;
    public static int whichPid = 1;

    @Override
    public void runOpMode() throws InterruptedException {
       // servo_motor = hardwareMap.get(CRServo.class, HardwareConfig.sorting);
       // motor_encoder = hardwareMap.get(DcMotorEx.class, HardwareConfig.LB);
        motor = hardwareMap.get(DcMotorEx.class, HardwareConfig.turret_launch);
        voltageSensor = hardwareMap.getAll(VoltageSensor.class).get(0);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);

/*
        servo_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor_encoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor_encoder.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_encoder.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

 */

        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        whichPid = 1; target = 0; position = 0; pidError=0;

        pidf1.resetPid();
        pidf2.resetPid();

        pidf1.setTargetPosition(target);
        pidf2.setTargetPosition(target);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()){
            //position = motor_encoder.getCurrentPosition();
            position = motor.getCurrentPosition();
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
            power = power * (14/voltageSensor.getVoltage()) + kF_forbothPids;

           // servo_motor.setPower(power);
            motor.setPower(power);

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
