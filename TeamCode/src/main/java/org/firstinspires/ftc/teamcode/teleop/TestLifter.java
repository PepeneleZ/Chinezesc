package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.lib.Controller;
import org.firstinspires.ftc.teamcode.util.Lifter;

@TeleOp
public class TestLifter extends LinearOpMode {

    public Controller controller;
    public boolean toggle = true;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        controller = new Controller(gamepad1);

        Lifter lifter = new Lifter(hardwareMap, telemetry);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            if(toggle) {
                lifter.toggle();
                toggle = false;
            }
            lifter.update();
        }
    }
}
