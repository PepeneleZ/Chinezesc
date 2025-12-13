package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.lib.Controller;
import org.firstinspires.ftc.teamcode.util.HardwareConfig;

@TeleOp
@Config
public class TestTransfer extends LinearOpMode {
    public static double posUp=0.8;
    public static double posDown=0;
    public Controller controller1;

    public boolean up = false;

    @Override
    public void runOpMode() throws InterruptedException {

        Servo transfer = hardwareMap.get(Servo.class, HardwareConfig.transfer);
        controller1 = new Controller(gamepad1);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {


            if(controller1.cross.isPressed()){
                if(!up){
                    transfer.setPosition(posUp);
                    up = true;
                } else {
                    transfer.setPosition(posDown);
                    up = false;
                }
            }

            controller1.update();
        }
    }
}
