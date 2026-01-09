package org.firstinspires.ftc.teamcode.lib.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.lib.Controller;

@Config
@TeleOp
public class TestServo extends LinearOpMode {

    Controller controller1;
    Servo motor;
    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(Servo.class, "A");

        controller1 = new Controller(gamepad1);
        motor.setPosition(0);
        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            if (controller1.cross.isPressed()) {
                if (motor.getPosition() == 0)
                    motor.setPosition(0.4);
                else
                    motor.setPosition(0);
            }
            controller1.update();
        }
    }
}
