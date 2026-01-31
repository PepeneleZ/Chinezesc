package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class CentrareTureta extends OpMode {
    Servo A, B;

    @Override
    public void init() {
        A = hardwareMap.get(Servo.class, "A");
        B = hardwareMap.get(Servo.class, "B");
        A.setPosition(0.7);
        B.setPosition(0.7);
    }

    @Override
    public void loop() {
        A.setPosition(1);
        B.setPosition(1);
    }
}
