package org.firstinspires.ftc.teamcode.lib.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.HardwareConfig;

@TeleOp
public class TestServo extends LinearOpMode {

    static public double speed = 0;
    static public double position = 0;
    Servo servo1, servo2, servo3;

    @Override
    public void runOpMode() throws InterruptedException {
         servo1 = hardwareMap.get(Servo.class, HardwareConfig.turret_hrot1);
         servo2 = hardwareMap.get(Servo.class, HardwareConfig.turret_hrot2);
         servo3 = hardwareMap.get(Servo.class, HardwareConfig.vertical_angle_servo);

         servo1.setDirection(Servo.Direction.FORWARD);

         servo1.setPosition(0.7);
         servo2.setPosition(0.7);
         servo3.setPosition(0.7);

         waitForStart();
         servo1.setPosition(1);
         servo2.setPosition(1);
         servo3.setPosition(1);

         while(opModeIsActive() && !isStopRequested()){

         }
    }
}
