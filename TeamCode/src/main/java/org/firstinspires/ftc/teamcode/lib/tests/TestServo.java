package org.firstinspires.ftc.teamcode.lib.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TestServo extends LinearOpMode {

    static public double speed = 0;
    Servo servo;

    @Override
    public void runOpMode() throws InterruptedException {
         servo = hardwareMap.get(Servo.class, "A");
         servo.setPosition(0);

        waitForStart();
        servo.setPosition(1);

        while(opModeIsActive() && !isStopRequested()){

        }
    }
}
