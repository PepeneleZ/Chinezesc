package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.field.Line;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.HardwareConfig;

@TeleOp
public class TestIntake extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        CRServo upper_intake = hardwareMap.get(CRServo.class, HardwareConfig.upper_intake);
        CRServo lower_intake = hardwareMap.get(CRServo.class, HardwareConfig.lower_intake);
        upper_intake.setDirection(DcMotorSimple.Direction.REVERSE);
        lower_intake.setDirection(DcMotorSimple.Direction.FORWARD);


        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            upper_intake.setPower(1);
            //+lower_intake.setPower(1);
        }
    }
}
