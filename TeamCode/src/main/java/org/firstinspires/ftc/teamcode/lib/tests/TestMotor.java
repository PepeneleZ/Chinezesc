package org.firstinspires.ftc.teamcode.lib.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.HardwareConfig;

@Config
@TeleOp
public class TestMotor extends LinearOpMode {

    static public double speed = 1;
    DcMotorEx motor;
    CRServo servo1, servo2;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, HardwareConfig.turret_launch);
//        servo1 = hardwareMap.get(CRServo.class,HardwareConfig.sorting1);
//        servo2 = hardwareMap.get(CRServo.class,HardwareConfig.sorting2);
//        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        servo1.setDirection(DcMotorSimple.Direction.FORWARD);
//        servo2.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            motor.setPower(speed);
//            servo1.setPower(1);
//            servo2.setPower(1);
            telemetry.addData("Motor position: ", motor.getCurrentPosition());
            telemetry.update();
        }
    }
}
