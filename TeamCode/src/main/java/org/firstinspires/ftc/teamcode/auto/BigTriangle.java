package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Robot;

@Autonomous
public class BigTriangle extends LinearOpMode {
    public Robot bot;

    @Override
    public void runOpMode() throws InterruptedException {
        bot = new Robot(hardwareMap,telemetry);
        waitForStart();
        bot.driveTrain.setMotorPower(1,-1,-1,1);
        bot.update();
        sleep(600);
        if(isStopRequested()) return;
        bot.driveTrain.setMotorPower(0,0,0,0);
        bot.update();
    }
}
