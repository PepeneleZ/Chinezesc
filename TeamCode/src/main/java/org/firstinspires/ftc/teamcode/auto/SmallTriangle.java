package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.Turret;

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
        robot.sorting.fillMagazine();
        waitForStart();
        Turret.setTarget_rotation(Turret.TURRET_LAUNCH_SPEEDS.FAR);
        while (!isStopRequested() && (timer.seconds()<15 || action==0)) {
            if(action==0){
                timer = new ElapsedTime();
                timer.reset();
                timer.startTime();
                action++;
            }
            if (action==1 && timer.seconds() >2){
                robot.sorting.shoot();
                action++;
            }
            else if (action==2 && timer.seconds()>6){
                robot.driveTrain.setMotorPower(1, -1, -1, 1);
                action++;
            }
            else if (action ==3 && timer.seconds()>6.6){
                robot.driveTrain.setMotorPower(0, 0, 0, 0);
                Turret.setTarget_rotation(Turret.TURRET_LAUNCH_SPEEDS.STOPPED);
                action++;
            }

            telemetry.update();
            robot.update();
        }
    }
}
