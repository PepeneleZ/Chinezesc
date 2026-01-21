package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.field.Line;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.lib.tests.ColorFunctions;
import org.firstinspires.ftc.teamcode.util.HardwareConfig;
import org.firstinspires.ftc.teamcode.util.Intake;

@Config
@TeleOp
public class TestColor extends LinearOpMode {
    ColorSensor color_human,color_intake;
    @Override
    public void runOpMode() throws InterruptedException {

        color_intake = hardwareMap.get(ColorSensor.class,HardwareConfig.color_sensor);
        color_intake.enableLed(true);
        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Color sensor: ", ColorFunctions.getColor(color_intake.red(),color_intake.green(),color_intake.blue()).val);
            telemetry.addData("Red: ",color_intake.red());
            telemetry.addData("Green: ",color_intake.green());
            telemetry.addData("Blue: ",color_intake.blue());

            telemetry.update();
        }
    }
}