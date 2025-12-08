package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    public DcMotor intake;
    public boolean running = false;

    public Intake(HardwareMap hwmap){
        intake = hwmap.get(DcMotorEx.class, HardwareConfig.intake);
    }

    public void toggle(){
        if(!running)
            running = true;
        else
            running = false;
    }

    public void update(){
        if (running) {
            intake.setPower(1);
        }
        else {
            intake.setPower(0);
        }
    }
}
