package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.Controller;
import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.Sorting;

@TeleOp
public class Test extends LinearOpMode {
    public Robot robot;
    public Controller controller1,controller2;

    @Override
    public void runOpMode() throws InterruptedException {

        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);

        robot = new Robot(hardwareMap, telemetry);
        boolean isCheckingIntake = false;

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            robot.driveTrain.drive(-controller2.leftStickX, controller2.leftStickY, controller2.rightStickX);

//            if (controller1.circle.isPressed())
//                robot.sorting.rotate_elice(1);
//            if (controller1.dpadRight.isPressed())
//                robot.sorting.rotate_elice(0.5);
//            if (controller1.dpadLeft.isPressed())
//                robot.sorting.rotate_elice(-0.5);

            if (controller1.triangle.isPressed()) {
                robot.intake.toggle();
                if (robot.intake.running)
                    robot.sorting.setNextState(Sorting.MOVING_STATES.WAITING_INTAKE);
                else
                    robot.sorting.resetStates();
            }
            if (controller1.cross.isPressed())
                robot.sorting.sort_balls();
            if (controller1.circle.isPressed())
                robot.sorting.resetEverything();
            if(controller1.dpadLeft.isPressed())
                robot.turret.decreasePower();
            if (controller1.dpadRight.isPressed())
                robot.turret.increasePower();

            if (controller1.square.isPressed())
                robot.sorting.rotate_elice(-1);

            if (controller1.dpadUp.isPressed()) {
                robot.sorting.transfer_ball();
                Sorting.magazine[6] = Sorting.COLORS.EMPTY;
            }
            if (controller1.bumperRight.isPressed()){
                robot.turret.shoot_far();
            }
            if(controller1.bumperLeft.isPressed())
                robot.turret.shoot_close();

            if(robot.sorting.current_moving_state== Sorting.MOVING_STATES.NOTHING)
                robot.sorting.blade.setPower(-controller1.rightStickX*0.7);




            controller1.update();
            controller2.update();
            robot.update();
            robot.sorting.telemetryData();
            telemetry.update();
        }
    }
}
