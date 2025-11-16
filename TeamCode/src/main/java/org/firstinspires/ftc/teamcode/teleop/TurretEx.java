package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.lib.Controller;
import org.firstinspires.ftc.teamcode.util.Turret;

public class TurretEx extends LinearOpMode {

    public Turret turret;
    public Controller controller1;

    @Override
    public void runOpMode() throws InterruptedException {

        turret = new Turret(hardwareMap);
        controller1 = new Controller(gamepad1);

        while(opModeIsActive() && !isStopRequested()){

            if(controller1.circle.isPressed()){
                turret.launch();
            }

            controller1.update();
        }
    }
}
