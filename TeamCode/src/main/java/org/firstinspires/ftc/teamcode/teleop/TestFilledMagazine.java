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
public class TestFilledMagazine extends LinearOpMode {
    public Robot robot;
    public Controller controller1,controller2;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);

        robot = new Robot(hardwareMap, telemetry);
        boolean isCheckingIntake = false;

        robot.sorting.fillMagazine();

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            robot.driveTrain.drive(-controller1.leftStickX, controller1.leftStickY, controller1.rightStickX);

//            if (controller1.circle.isPressed())
//                robot.sorting.rotate_elice(1);
//            if (controller1.dpadRight.isPressed())
//                robot.sorting.rotate_elice(0.5);
//            if (controller1.dpadLeft.isPressed())
//                robot.sorting.rotate_elice(-0.5);
            if (controller1.circle.isPressed()){
                robot.intake.toggle(Intake.INTAKE_STATES.SPITTING_OUT);
            }
            if (controller1.dpadUp.isPressed()){
                robot.sorting.motif = 6;
            }
            if (controller1.dpadLeft.isPressed()){
                robot.sorting.motif = 4;
            }
            if (controller1.dpadRight.isPressed()){
                robot.sorting.motif = 2;
            }
            if (controller1.bumperRight.isPressed()){
                Turret.increaseChangeableTarget();
            }
            if (controller1.bumperLeft.isPressed()){
                Turret.decreaseChangeableTarget();
            }

            if (controller1.rightTrigger>0.2 || controller1.leftTrigger>0.2) {
                if(controller1.rightTrigger>controller1.leftTrigger)
                    robot.turret.turret_hrot.setPower(controller1.rightTrigger);
                else
                    robot.turret.turret_hrot.setPower(-controller1.leftTrigger);
            }
            else
                robot.turret.turret_hrot.setPower(0);


            if (Math.abs(controller2.rightStickX)>0.2){
                robot.sorting.manualBlade(controller2.rightStickX*40);
            }

            if (controller2.triangle.isPressed() || controller1.triangle.isPressed()) {
                robot.intake.toggle();
                if (robot.intake.intake_state == Intake.INTAKE_STATES.COLLECTING)
                    robot.sorting.setNextState(Sorting.MOVING_STATES.WAITING_INTAKE);
                else
                    robot.sorting.resetStates();
            }
            if (controller2.cross.isPressed())
                robot.sorting.resetEverything();
            if (controller2.circle.isPressed())//&& Turret.turret_launcher_state!= Turret.TURRET_LAUNCH_SPEEDS.STOPPED)
                robot.sorting.shoot();

            if (controller2.square.isPressed())
                robot.sorting.rotate_elice(1);

//            if (controller2.dpadDown.isPressed()){
//                robot.sorting.sort_balls();
//            }
//            if (controller2.dpadUp.isPressed()) {
//                robot.sorting.transfer_ball();
//                Sorting.magazine[4] = Sorting.COLORS.EMPTY;
//            }
            if (controller2.dpadLeft.isPressed()) {
                Turret.setSpecialTarget_rotation();
            }
            if (controller2.dpadRight.isPressed()){
                Turret.setTarget_rotation(0);
            }
            if (controller2.dpadUp.isPressed()){
                robot.sorting.transfer_ball();
            }
            if (controller2.dpadDown.isPressed()){

            }
            if (controller2.bumperLeft.isPressed()){
                Turret.setTarget_rotation(Turret.TURRET_LAUNCH_SPEEDS.CLOSE);
            }
            if(controller2.bumperRight.isPressed())
                Turret.setTarget_rotation(Turret.TURRET_LAUNCH_SPEEDS.FAR);



            controller1.update();
            controller2.update();
            robot.update();
            robot.sorting.telemetryData();
            telemetry.update();
        }
    }
}
