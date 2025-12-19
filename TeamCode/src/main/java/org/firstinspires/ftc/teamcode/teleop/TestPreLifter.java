package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.lib.Controller;
import org.firstinspires.ftc.teamcode.util.Lifter;

@TeleOp
public class TestPreLifter extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        Lifter lifter = new Lifter(hardwareMap, telemetry);
        lifter.back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lifter.front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        boolean test = false;

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            lifter.TelemetryData();
        }
    }
}
