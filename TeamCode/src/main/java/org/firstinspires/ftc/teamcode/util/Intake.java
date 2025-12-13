package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    public DcMotor intake;
    public boolean running = false;

    public Intake(HardwareMap hwmap){
        intake = hwmap.get(DcMotorEx.class, HardwareConfig.intake);
        //intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void toggle(){
        running = !running;
    }

    public void toggle(boolean running){
        this.running = running;
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
