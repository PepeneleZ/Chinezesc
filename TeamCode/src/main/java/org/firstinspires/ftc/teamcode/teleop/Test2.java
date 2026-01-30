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
    public static double verticalAngle = 40;
    public static int turretSpeed = 50;

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
        robot.setColor(robot.color);
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
            robot.lifter.toggle();
        }
        if (controller1.dpadDown.isPressed()){
            robot.turret.setHorizontalPositionFromAngle(Math.toRadians(0));
        }
        if (controller1.dpadRight.isPressed())
            robot.aimLebron();
        if(controller1.dpadLeft.isPressed())
            robot.sorting.shift = 0;
        if (controller1.bumperRight.isPressed()){
            //Turret.increaseChangeableTarget();
            Turret.setTarget_rotation(TURRET_LAUNCH_SPEEDS.FAR);
        }
        if (controller1.bumperLeft.isPressed()){
            //Turret.decreaseChangeableTarget();
            Turret.setTarget_rotation(TURRET_LAUNCH_SPEEDS.CLOSE);
        }
        if (controller1.square.isPressed()){
            robot.sorting.respectMotif = robot.sorting.getNumberOfBalls() == 3 && robot.sorting.getGreen() != 0;
            robot.sorting.setNextState(Constants.MOVING_STATES.SHOOTING);
        }


        if (controller1.triangle.isPressed()) {
            robot.intake.toggle();
            if (robot.intake.intake_state == INTAKE_STATES.COLLECTING)
                robot.sorting.setNextState(MOVING_STATES.WAITING_INTAKE);
            else if (robot.intake.intake_state == INTAKE_STATES.STOPPED && robot.sorting.current_moving_state==MOVING_STATES.WAITING_INTAKE)
                robot.sorting.setNextState(MOVING_STATES.NOTHING);
        }
        if (controller2.square.isPressed())
            robot.sorting.rotate_elice(1);

//            robot.sorting.rotate_elice(1);
        if (controller2.cross.isPressed()){
            robot.sorting.resetEverything();
        }
        if (controller2.circle.isPressed()){
            robot.sorting.rotate_elice(0.5);
        }
        if (controller2.triangle.isPressed()){
            robot.sorting.shootThreeBalls();
        }
        if (controller2.bumperLeft.isPressed()){
            robot.intake.toggle(INTAKE_STATES.COLLECTING);
        }
        if (controller2.bumperRight.isPressed())
            robot.sorting.shift++;
        if (controller1.cross.isPressed()) {
            if (Turret.turret_launcher_state == TURRET_LAUNCH_SPEEDS.STOPPED)
                Turret.setTarget_rotation(TURRET_LAUNCH_SPEEDS.CLOSE);
            else {
                robot.sorting.respectMotif = true;
                robot.sorting.setNextState(MOVING_STATES.SHOOTING);
            }
        }
//            if (controller2.dpadDown.isPressed()){
//                robot.sorting.sort_balls();
//            }
//            if (controller2.dpadUp.isPressed()) {
//                robot.sorting.transfer_ball();
//                Sorting.magazine[4] = Sorting.COLORS.EMPTY;
//            }

        if (controller2.dpadRight.isPressed()){
//            robot.aimLebron();
            //robot.sorting.rotate_elice(1);
        }
        if (controller2.dpadLeft.isPressed()){
            robot.pinpointLocalizer.setPose(new Pose(72,72,0));
        }
        if (controller2.dpadUp.isPressed())
            robot.sorting.transfer_ball(true);
        if (controller2.dpadDown.isPressed())
            robot.setColor(!robot.color);


        robot.sorting.manualBlade(controller2.rightStickX);

        controller1.update();
        controller2.update();
        robot.update();
        telemetry.update();
    }
}
