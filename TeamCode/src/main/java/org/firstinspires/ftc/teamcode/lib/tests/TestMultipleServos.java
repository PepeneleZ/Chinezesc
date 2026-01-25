package org.firstinspires.ftc.teamcode.lib.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.lib.Controller;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Intake;
import org.firstinspires.ftc.teamcode.util.Turret;

@Config
@TeleOp
public class TestMultipleServos extends LinearOpMode {
    Turret turret;
    Intake intake;
    Controller controller1;
    Servo motor, motor1,motor2;
    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(Servo.class, "servo1");
        motor1 = hardwareMap.get(Servo.class, "servo2");
        motor2 = hardwareMap.get(Servo.class,"servo3");

        controller1 = new Controller(gamepad1);
        motor.setPosition(0);
        motor1.setPosition(0);
        motor2.setPosition(0);
        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            if (controller1.dpadUp.isPressed()) {
                if (motor.getPosition() == 0)
                    motor.setPosition(0.4);
                else
                    motor.setPosition(0);
            }
            else if(controller1.dpadRight.isPressed()){
                if (motor1.getPosition() == 0)
                    motor1.setPosition(0.4);
                else
                    motor1.setPosition(0);
            }
            else if (controller1.dpadLeft.isPressed()){
                if (motor2.getPosition() == 0)
                    motor2.setPosition(0.4);
                else
                    motor2.setPosition(0);
            }
            if (controller1.cross.isPressed()){
                Turret.setTarget_rotation(Constants.TURRET_LAUNCH_SPEEDS.CLOSE);
            }
            if (controller1.triangle.isPressed()){
                intake.toggle();
            }
            controller1.update();
        }
    }
}
