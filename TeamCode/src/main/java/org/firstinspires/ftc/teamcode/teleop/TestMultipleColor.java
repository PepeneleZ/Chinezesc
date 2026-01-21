package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.field.Line;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.lib.tests.ColorFunctions;
import org.firstinspires.ftc.teamcode.util.Constants_Enums;
import org.firstinspires.ftc.teamcode.util.HardwareConfig;
import org.firstinspires.ftc.teamcode.util.Intake;
import org.firstinspires.ftc.teamcode.util.Constants_Enums.COLORS;


@Config
@TeleOp
public class TestMultipleColor extends LinearOpMode {
    ColorSensor[] colors = new ColorSensor[4];
    @Override
    public void runOpMode() throws InterruptedException {
        for (int i=1;i<=3;i++){
            colors[i] = hardwareMap.get(ColorSensor.class, HardwareConfig.color_sensor+i);
            colors[i].enableLed(true);
        }

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            for(int i=1;i<=3;i++){
                telemetry.addData("RED -> Color sensor "+i+": ",colors[i].red());
                telemetry.addData("GREEN -> Color sensor "+i+": ",colors[i].green());
                telemetry.addData("BLUE -> Color sensor "+i+": ",colors[i].blue());
                telemetry.addData("RESULT -> Color sensor "+i+": ",ColorFunctions.getColor(colors[i].red(),colors[i].green(),colors[i].blue()));

            }



            telemetry.update();
        }
    }
}
