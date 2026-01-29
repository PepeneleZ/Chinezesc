package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.Controller;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.Sorting;
import org.firstinspires.ftc.teamcode.util.Turret;

import org.firstinspires.ftc.teamcode.util.Constants.INTAKE_STATES;
import org.firstinspires.ftc.teamcode.util.Constants.MOVING_STATES;
import org.firstinspires.ftc.teamcode.util.Constants.TURRET_LAUNCH_SPEEDS;

@Config
@TeleOp
public class Test2 extends OpMode {
    public Robot robot;
    public Controller controller1,controller2;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);

        robot = new Robot(hardwareMap, telemetry);
    }

    @Override
    public void start(){
        robot.limelight.start();
        robot.sorting.respectMotif = false;
        robot.newRobotPose = new Pose(1,1,1);
    }

    @Override
    public void loop() {
        robot.driveTrain.drive(-controller1.leftStickY, -controller1.leftStickX, controller1.rightStickX);

//            if (controller1.circle.isPressed())
//                robot.sorting.rotate_elice(1);
//            if (controller1.dpadRight.isPressed())
//                robot.sorting.rotate_elice(0.5);
//            if (controller1.dpadLeft.isPressed())
//                robot.sorting.rotate_elice(-0.5);
        if (controller1.circle.isPressed()){
            robot.intake.toggle(INTAKE_STATES.SPITTING_OUT);
        }
        if (controller1.dpadUp.isPressed()){
            Sorting.motif = Constants.MOTIF.GPP;
        }
        if (controller1.dpadLeft.isPressed()){
            Sorting.motif = Constants.MOTIF.PGP;
        }
        if (controller1.dpadRight.isPressed()){
            Sorting.motif = Constants.MOTIF.PPG;
        }
        if (controller1.bumperRight.isPressed() || controller2.bumperRight.isPressed()){
            //Turret.increaseChangeableTarget();
            Turret.setTarget_rotation(TURRET_LAUNCH_SPEEDS.FAR);
        }
        if (controller1.bumperLeft.isPressed() || controller2.bumperLeft.isPressed()){
            //Turret.decreaseChangeableTarget();
            Turret.setTarget_rotation(TURRET_LAUNCH_SPEEDS.CLOSE);
        }


        if (controller2.triangle.isPressed() || controller1.triangle.isPressed()) {
            robot.intake.toggle();
            if (robot.intake.intake_state == INTAKE_STATES.COLLECTING)
                robot.sorting.setNextState(MOVING_STATES.WAITING_INTAKE);
        }
        if (controller2.square.isPressed())
            robot.limelight.limelight.pipelineSwitch(2);

//            robot.sorting.rotate_elice(1);
        if (controller2.cross.isPressed()){
//            robot.sorting.resetEverything();
            robot.limelight.limelight.pipelineSwitch(2);
        }
        if (controller2.circle.isPressed() || controller1.cross.isPressed()) {
            if (Turret.turret_launcher_state == TURRET_LAUNCH_SPEEDS.STOPPED)
                Turret.setTarget_rotation(TURRET_LAUNCH_SPEEDS.CLOSE);
            else
                robot.sorting.setNextState(MOVING_STATES.SHOOTING);
        }
//            if (controller2.dpadDown.isPressed()){
//                robot.sorting.sort_balls();
//            }
//            if (controller2.dpadUp.isPressed()) {
//                robot.sorting.transfer_ball();
//                Sorting.magazine[4] = Sorting.COLORS.EMPTY;
//            }

        if (controller2.dpadRight.isPressed()){
            robot.aimLebron();
            //robot.sorting.rotate_elice(1);
        }
        if (controller2.dpadLeft.isPressed()){
            robot.pinpointLocalizer.setPose(robot.llpose);
            //robot.sorting.rotate_elice(-1);
        }
        if (controller2.dpadDown.isPressed()){
            robot.sorting.respectMotif = !robot.sorting.respectMotif;
        }
        if (controller2.dpadUp.isPressed())
            robot.sorting.transfer_ball(true);

        controller1.update();
        controller2.update();
        robot.update();
        telemetry.update();
    }
}
