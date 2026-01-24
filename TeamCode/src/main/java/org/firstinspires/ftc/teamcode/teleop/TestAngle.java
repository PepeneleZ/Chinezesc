package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.HardwareConfig;

@TeleOp
public class TestAngle extends LinearOpMode {
    public Servo servo;

    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, HardwareConfig.turret_angle);
        waitForStart();
        while(opModeIsActive() && !isStopRequested()){
            servo.setPosition(0);
        }
    }
}
