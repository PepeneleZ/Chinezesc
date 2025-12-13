package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.field.Line;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.lib.Controller;
import org.firstinspires.ftc.teamcode.util.HardwareConfig;
import org.firstinspires.ftc.teamcode.util.Sorting;

@TeleOp
public class TestSortare extends LinearOpMode {
   // public Sorting sortare;
    public CRServo sortare_servo;
    @Override
    public void runOpMode() throws InterruptedException {
        Controller controller = new Controller(gamepad1);
        sortare_servo = hardwareMap.get(CRServo.class,HardwareConfig.sorting);







        double power=1;

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            if (controller.cross.isPressed())
                power=power-1;
            sortare_servo.setPower(power);
        }

    }
}
