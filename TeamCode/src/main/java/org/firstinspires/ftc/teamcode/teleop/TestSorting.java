package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.Controller;
import org.firstinspires.ftc.teamcode.util.Intake;
import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.Sorting;
import org.firstinspires.ftc.teamcode.util.Turret;

import org.firstinspires.ftc.teamcode.util.Constants_Enums.INTAKE_STATES;
import org.firstinspires.ftc.teamcode.util.Constants_Enums.MOVING_STATES;
import org.firstinspires.ftc.teamcode.util.Constants_Enums.TURRET_LAUNCH_SPEEDS;


@Config
@TeleOp
public class TestSorting extends LinearOpMode {
    public Sorting sorting;
    public Intake intake;
    public Controller controller1,controller2;
    public VoltageSensor voltageSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        voltageSensor = hardwareMap.getAll(VoltageSensor.class).get(0);
        intake = new Intake(hardwareMap);
        sorting = new Sorting(hardwareMap,telemetry,intake,voltageSensor);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);



        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {

//            if (controller1.circle.isPressed())
//                robot.sorting.rotate_elice(1);
//            if (controller1.dpadRight.isPressed())
//                robot.sorting.rotate_elice(0.5);
//            if (controller1.dpadLeft.isPressed())
//                robot.sorting.rotate_elice(-0.5);
            if (controller1.dpadUp.isPressed()){
                sorting.motif = 6;
            }
            if (controller1.dpadLeft.isPressed()){
                sorting.motif = 4;
            }
            if (controller1.dpadRight.isPressed()){
                sorting.motif = 2;
            }

            if (controller1.cross.isPressed()){
                sorting.shoot();
            }




            if (Math.abs(controller2.rightStickX)>0.2){
                sorting.manualBlade(controller2.rightStickX*40);
            }

            if (controller2.triangle.isPressed() || controller1.triangle.isPressed()) {
                intake.toggle();
                if (intake.intake_state == INTAKE_STATES.COLLECTING)
                    sorting.setNextState(MOVING_STATES.WAITING_INTAKE);
                else
                    sorting.resetStates();
            }
            if (controller2.cross.isPressed())
                sorting.resetEverything();
            if (controller2.circle.isPressed()&& Turret.turret_launcher_state!= TURRET_LAUNCH_SPEEDS.STOPPED) {
                sorting.shoot();
                if (Turret.turret_launcher_state == TURRET_LAUNCH_SPEEDS.STOPPED)
                    Turret.setTarget_rotation(TURRET_LAUNCH_SPEEDS.CLOSE);
            }

            if (controller2.square.isPressed())
                sorting.rotate_elice(1);

//            if (controller2.dpadDown.isPressed()){
//                robot.sorting.sort_balls();
//            }
//            if (controller2.dpadUp.isPressed()) {
//                robot.sorting.transfer_ball();
//                Sorting.magazine[4] = Sorting.COLORS.EMPTY;
//            }
            if (controller2.dpadLeft.isPressed()) {
                sorting.rotate_elice(-1);
            }

            if (controller2.dpadUp.isPressed()){
                sorting.transfer_ball();
            }
            if (controller2.dpadDown.isPressed()){
                sorting.setNextState(MOVING_STATES.WAITING_HUMAN_PLAYER);
            }



            controller1.update();
            controller2.update();
            sorting.update();
            intake.update();
            telemetry.update();
        }
    }
}
