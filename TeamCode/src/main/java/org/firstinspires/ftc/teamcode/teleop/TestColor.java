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
import org.firstinspires.ftc.teamcode.util.Sorting.COLORS;

@Config
@TeleOp
public class TestColor extends LinearOpMode {
    ColorSensor color_human,color_intake;
    @Override
    public void runOpMode() throws InterruptedException {

        color_human = hardwareMap.get(ColorSensor.class,HardwareConfig.color_sensor_human);
        color_intake = hardwareMap.get(ColorSensor.class,HardwareConfig.color_sensor_intake);
        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Color sensor intake: ", ColorFunctions.getColor(color_intake.red(),color_intake.green(),color_intake.blue()).val);
            telemetry.addData("Color sensor human: ", ColorFunctions.getColor(color_human.red(),color_human.green(),color_human.blue()).val);
            telemetry.addData("Red_intake: ",color_intake.red());
            telemetry.addData("Green_intake: ",color_intake.green());
            telemetry.addData("Blue_intake: ",color_intake.blue());

            telemetry.addData("Red_human: ",color_human.red());
            telemetry.addData("Green_human: ",color_human.green());
            telemetry.addData("Blue_human: ",color_human.blue());



            telemetry.update();
        }
    }
}
