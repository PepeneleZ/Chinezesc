package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.Controller;
import org.firstinspires.ftc.teamcode.util.Intake;
import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.Sorting;
import org.firstinspires.ftc.teamcode.util.Turret;

@Config
@TeleOp
public class TestSoloDriver extends LinearOpMode {
    public Robot robot;
    public Controller controller1,controller2;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        controller1 = new Controller(gamepad1);

        robot = new Robot(hardwareMap, telemetry);
        boolean isCheckingIntake = false;


        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            robot.driveTrain.drive(-controller1.leftStickX, controller1.leftStickY, controller1.rightStickX);

            if (controller1.cross.isPressed()){
                robot.intake.toggle(Intake.INTAKE_STATES.SPITTING_OUT);
            }
            if (controller1.dpadUp.isPressed()){
                robot.sorting.transfer_ball();
            }
            if (controller1.dpadLeft.isPressed()){
                robot.sorting.resetEverything();
            }
            if (controller1.dpadRight.isPressed()){
                robot.sorting.motif += 2;
                if (robot.sorting.motif >=8) robot.sorting.motif = 2;
            }
            if (controller1.bumperRight.isPressed()){
                Turret.setTarget_rotation(Turret.TURRET_LAUNCH_SPEEDS.FAR);
            }
            if (controller1.bumperLeft.isPressed()){
                Turret.setTarget_rotation(Turret.TURRET_LAUNCH_SPEEDS.CLOSE);
            }


            if (controller1.triangle.isPressed()) {
                robot.intake.toggle();
                if (robot.intake.intake_state == Intake.INTAKE_STATES.COLLECTING)
                    robot.sorting.setNextState(Sorting.MOVING_STATES.WAITING_INTAKE);
                else
                    robot.sorting.resetStates();
            }
            if (controller1.dpadDown.isPressed()&& Turret.turret_launcher_state != Turret.TURRET_LAUNCH_SPEEDS.STOPPED) {
                robot.sorting.shoot();
                if (Turret.turret_launcher_state == Turret.TURRET_LAUNCH_SPEEDS.STOPPED)
                    Turret.setTarget_rotation(Turret.TURRET_LAUNCH_SPEEDS.CLOSE);
            }

            if (controller1.square.isPressed())
                robot.sorting.rotate_elice(1);
            if (controller1.circle.isPressed())
                robot.sorting.rotate_elice(-1);

            controller1.update();
            robot.update();
            robot.sorting.telemetryData();
            telemetry.update();
        }
    }
}
