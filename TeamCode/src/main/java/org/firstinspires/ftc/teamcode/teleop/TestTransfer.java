package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.HardwareConfig;

@TeleOp
public class TestTransfer extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Servo transfer = hardwareMap.get(Servo.class, HardwareConfig.transfer);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            transfer.setPosition(0.25);
        }
    }
}
