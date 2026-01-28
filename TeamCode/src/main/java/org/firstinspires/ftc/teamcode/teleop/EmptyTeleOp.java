package org.firstinspires.ftc.teamcode.teleop;

import android.util.Pair;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.Controller;
import org.firstinspires.ftc.teamcode.lib.TrapezoidalMotionProfile;

@TeleOp
@Config
public class EmptyTeleOp extends LinearOpMode {
    public static double time = 0;
    public static double vel = 0;
    public static double accel = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        Controller ctrl1 = new Controller(gamepad1);
        TrapezoidalMotionProfile profile = new TrapezoidalMotionProfile(30, 120, 0);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        waitForStart();

        timer.reset();
        while(opModeIsActive()){
            telemetry.addData("time", time);
            telemetry.addData("vel", vel);
            telemetry.addData("accel", accel);
            telemetry.update();
            time = timer.milliseconds();
            Pair<Double, Double> velaccel = profile.get(time);

            vel = velaccel.first;
            accel = velaccel.second;

            if(ctrl1.circle.isPressed()){
                profile.toggleDeceleration(true, vel);
                timer.reset();
            }

            if(ctrl1.cross.isPressed()){
                profile.toggleDeceleration(false, vel);
                timer.reset();
            }

            ctrl1.update();
        }
    }
}
