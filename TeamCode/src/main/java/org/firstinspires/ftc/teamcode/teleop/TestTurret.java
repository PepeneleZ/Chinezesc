package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.util.HardwareConfig;

@TeleOp
public class TestTurret extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        CRServo turret_hrot = hardwareMap.get(CRServo.class, HardwareConfig.turret_hrot);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            turret_hrot.setPower(0.4);
        }
    }
}
