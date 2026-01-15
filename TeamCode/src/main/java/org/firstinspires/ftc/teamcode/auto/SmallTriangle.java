package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Constants_Enums;
import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.Turret;

import org.firstinspires.ftc.teamcode.util.Constants_Enums.TURRET_LAUNCH_SPEEDS;

@Autonomous
public class SmallTriangle extends LinearOpMode {
    public Robot robot;
    public ElapsedTime timer;
    public int action = 0;

//(.)(.)
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap,telemetry);
        timer = new ElapsedTime();
        robot.storage.setState(Constants_Enums.MOVING_STATES.WAITING_INTAKE);
        waitForStart();
        sleep(2000);
        Turret.setTarget_rotation(TURRET_LAUNCH_SPEEDS.FAR);
        while (!isStopRequested() && (timer.seconds()<20 || action==0)) {
            if(action==0){
                timer = new ElapsedTime();
                timer.reset();
                timer.startTime();
                action++;
            }
            if (action==1 && timer.seconds() >2){
                robot.storage.setState(Constants_Enums.MOVING_STATES.SHOOTING);
                action++;
            }
            if(action ==2 && timer.seconds()>8){
                robot.storage.setState(Constants_Enums.MOVING_STATES.SHOOTING);
                action++;
            }
            else if (action==3 && timer.seconds()>14){
                robot.driveTrain.setMotorPower(-1, -1, 1, 1);
                action++;
            }
            else if (action ==4 && timer.seconds()>14.6){
                robot.driveTrain.setMotorPower(0, 0, 0, 0);
                Turret.setTarget_rotation(TURRET_LAUNCH_SPEEDS.STOPPED);
                action++;
            }

            telemetry.update();
            robot.update();
        }
    }
}
