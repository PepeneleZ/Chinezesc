package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.Controller;
import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.Sorting;
import org.firstinspires.ftc.teamcode.util.Turret;

@Config
@TeleOp
public class Test extends LinearOpMode {
    public Robot robot;
    public Controller controller1,controller2;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);

        robot = new Robot(hardwareMap, telemetry);
        boolean isCheckingIntake = false;


        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            robot.driveTrain.drive(-controller1.leftStickX, controller1.leftStickY, controller1.rightStickX);

//            if (controller1.circle.isPressed())
//                robot.sorting.rotate_elice(1);
//            if (controller1.dpadRight.isPressed())
//                robot.sorting.rotate_elice(0.5);
//            if (controller1.dpadLeft.isPressed())
//                robot.sorting.rotate_elice(-0.5);


            if (Math.abs(controller2.rightStickX)>0.2){
                robot.sorting.manualBlade(controller2.rightStickX*40);
            }

            if (controller2.triangle.isPressed()) {
                robot.intake.toggle();
                if (robot.intake.running)
                    robot.sorting.setNextState(Sorting.MOVING_STATES.WAITING_INTAKE);
                else
                    robot.sorting.resetStates();
            }
            if (controller2.cross.isPressed())
                robot.sorting.resetEverything();
            if (controller2.circle.isPressed())
                robot.sorting.shoot();

            if (controller2.square.isPressed())
                robot.sorting.rotate_elice(1);

            if (controller2.dpadUp.isPressed()) {
                robot.sorting.transfer_ball();
                Sorting.magazine[4] = Sorting.COLORS.EMPTY;
            }
            if (controller2.dpadDown.isPressed()){
                robot.sorting.sort_balls();
            }
            if (controller2.bumperLeft.isPressed()){
                robot.turret.setTarget_rotation(Turret.TURRET_LAUNCH_SPEEDS.CLOSE);
            }
            if(controller2.bumperRight.isPressed())
                robot.turret.setTarget_rotation(Turret.TURRET_LAUNCH_SPEEDS.FAR);





            controller1.update();
            controller2.update();
            robot.update();
            robot.sorting.telemetryData();
            telemetry.update();
        }
    }
}
