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
    public static double deltaHorizontalAngle = 20, deltaVerticalAngle = 10;
    public static double verticalAngle=30, horizontalAngle=0;

    public static boolean runOnPower = false;
    public static double power = 0;
    Controller controller1, controller2;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        turret = new Turret(hardwareMap, telemetry);
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);
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

            if (controller2.dpadUp.isPressed()){
                verticalAngle += deltaVerticalAngle;
                turret.setVerticalPositionFromAngle(Math.toRadians(verticalAngle));
            }
            if (controller2.dpadDown.isPressed()){
                verticalAngle -= deltaVerticalAngle;
                turret.setVerticalPositionFromAngle(Math.toRadians(verticalAngle));
            }
            if(controller2.dpadRight.isPressed()){
                horizontalAngle += deltaHorizontalAngle;
                turret.setHorizontalPositionFromAngle(Math.toRadians(horizontalAngle));
            }
            if(controller2.dpadLeft.isPressed()){
                horizontalAngle -= deltaHorizontalAngle;
                turret.setHorizontalPositionFromAngle(Math.toRadians(horizontalAngle));
            }

            turret.update();
            turret.update_telemetry();
            telemetry.addData("Vertical angle", turret.getVerticalAngle());
            telemetry.addData("Horizontal angle, ", turret.getHorizontalAngle());
            telemetry.addData("30: ",turret.getVerticalPositionFromAngle(Math.toRadians(30)));
            telemetry.addData("40: ",turret.getVerticalPositionFromAngle(Math.toRadians(40)));
            telemetry.addData("50: ",turret.getVerticalPositionFromAngle(Math.toRadians(50)));
            telemetry.addData("70: ",turret.getVerticalPositionFromAngle(Math.toRadians(70)));


            controller1.update();
            controller2.update();
            telemetry.update();

        }
    }
}
