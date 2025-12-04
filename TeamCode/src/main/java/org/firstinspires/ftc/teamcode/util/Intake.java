package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    public CRServo upper_intake, lower_intake;
    public boolean running = false;

    public Intake(HardwareMap hwmap){
        upper_intake = hwmap.get(CRServo.class, HardwareConfig.upper_intake);
        lower_intake = hwmap.get(CRServo.class, HardwareConfig.lower_intake);

        upper_intake.setDirection(CRServo.Direction.REVERSE);
        lower_intake.setDirection(CRServo.Direction.REVERSE);
    }

    public void toggle(){
        if(!running)
            running = true;
        else
            running = false;
    }

    public void update(){
        if (running) {
            upper_intake.setPower(1);
            //lower_intake.setPower(1);
        }
        else {
            upper_intake.setPower(0);
            //lower_intake.setPower(0);
        }
    }
}
