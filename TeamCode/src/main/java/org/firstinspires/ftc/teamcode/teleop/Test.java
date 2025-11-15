package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.Controller;
import org.firstinspires.ftc.teamcode.util.Robot;

@TeleOp
public class Test extends LinearOpMode {
    public Robot robot;
    public Controller controller1;

    @Override
    public void runOpMode() throws InterruptedException {

        controller1 = new Controller(gamepad1);

        robot = new Robot(hardwareMap, telemetry);

        while(opModeIsActive() && !isStopRequested()){
            robot.driveTrain.drive(-controller1.leftStickY, controller1.leftStickX, controller1.rightStickX);
        }

        if(controller1.cross.isDown())
            robot.intake.lower_intake.setPower(1);

        if(controller1.circle.isPressed())
            robot.intake.upper_intake.setPower(1);
    }
}
