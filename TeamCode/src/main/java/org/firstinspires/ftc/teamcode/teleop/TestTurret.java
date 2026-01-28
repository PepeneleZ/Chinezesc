package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.Controller;
import org.firstinspires.ftc.teamcode.util.Turret;

import java.io.CharArrayReader;

@TeleOp
@Config
public class TestTurret extends LinearOpMode {

    Turret turret;
    public static int target=20;
    public static boolean runOnPower = false;
    public static double power = 0;
    Controller controller1;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        turret = new Turret(hardwareMap, telemetry);
        controller1 = new Controller(gamepad1);
        waitForStart();
        while(opModeIsActive() && !isStopRequested()){
            if (controller1.triangle.isPressed())
                Turret.setTarget_rotation(target);
            if (controller1.cross.isPressed())
                Turret.setTarget_rotation(0);
            if (controller1.square.isPressed())
                runOnPower = !runOnPower;
            if (runOnPower){
                Turret.runPid = false;
                turret.turret_launch.setPower(power);
            }

            turret.update();
            turret.update_telemetry();
            controller1.update();
            telemetry.update();

        }
    }
}
