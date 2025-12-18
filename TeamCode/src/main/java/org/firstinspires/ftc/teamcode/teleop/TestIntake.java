package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.field.Line;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.HardwareConfig;
import org.firstinspires.ftc.teamcode.util.Intake;

@TeleOp
public class TestIntake extends LinearOpMode {
    public Intake intake;
    @Override
    public void runOpMode() throws InterruptedException {
        intake = new Intake(hardwareMap);

        waitForStart();
        intake.toggle();

        while(opModeIsActive() && !isStopRequested()) {
            intake.update();
        }
    }
}
