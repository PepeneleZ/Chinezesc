package org.firstinspires.ftc.teamcode.lib.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.HardwareConfig;

@TeleOp
public class TestServo extends LinearOpMode {

    static public double speed = 0;
    Servo servo1, servo2;

    @Override
    public void runOpMode() throws InterruptedException {
         servo1 = hardwareMap.get(Servo.class, HardwareConfig.transfer_servo1);
         servo2 = hardwareMap.get(Servo.class, HardwareConfig.transfer_servo2);

         servo1.setDirection(Servo.Direction.REVERSE);

        servo1.setPosition(0);
        servo2.setPosition(0);

        waitForStart();
        servo1.setPosition(1);
        servo2.setPosition(1/5.0);

        while(opModeIsActive() && !isStopRequested()){

        }
    }
}
