package org.firstinspires.ftc.teamcode.teleop;

import static java.lang.Math.abs;
import static java.lang.Math.pow;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.lib.Controller;
import org.firstinspires.ftc.teamcode.util.HardwareConfig;
import org.firstinspires.ftc.teamcode.util.Limelight;

@Config
@TeleOp
public class TestTurretHRot extends OpMode {
    Controller controller1;
    CRServo servo1, servo2;
    Limelight limelight;

    public static double acceptableError = 30f;
    public static double Kp = 0.5;
    public static double lateralOffset = 2.0;

    @Override
    public void init() {
        servo1 = hardwareMap.get(CRServo.class, HardwareConfig.turret_hrot1);
        servo2 = hardwareMap.get(CRServo.class, HardwareConfig.turret_hrot2);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        limelight = new Limelight(hardwareMap, telemetry);

        //telemetry.setMsTransmissionInterval(11);

        controller1 = new Controller(gamepad1);

    }

    @Override
    public void start(){
        limelight.start();
    }

    @Override
    public void loop() {
        if (limelight.result != null && limelight.result.isValid()) {

            double error = limelight.result.getTy() - lateralOffset;

            if(abs(error) > acceptableError) {
                double power = Range.clip(error * Kp, -1.0, 1.0);
                servo1.setPower(power);
                servo2.setPower(power);
            } else{
                stopTurret();
            }
        } else{
            if(controller1.bumperLeft.isPressed()){
                servo1.setPower(1);
                servo2.setPower(1);
            } else if (controller1.bumperRight.isPressed()) {
                servo1.setPower(-1);
                servo2.setPower(-1);
            }
        }



        controller1.update();
        limelight.update();
        telemetry.update();
    }

    private void stopTurret(){
        servo1.setPower(0);
        servo2.setPower(0);
    }
}
