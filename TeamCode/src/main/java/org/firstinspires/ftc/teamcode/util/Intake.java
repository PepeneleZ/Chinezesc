package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    public DcMotor intake;
    public INTAKE_STATES intake_state = INTAKE_STATES.STOPPED;

    public Intake(HardwareMap hwmap){
        intake = hwmap.get(DcMotorEx.class, HardwareConfig.intake);
        //intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public enum INTAKE_STATES{
        STOPPED(0),COLLECTING(1),SPITTING_OUT(-1),SLIGHTLY_MOVING(0.4);
        final double val;
        INTAKE_STATES(double val){this.val = val;}
    }
    public void toggle(){
        if (intake_state == INTAKE_STATES.COLLECTING || intake_state == INTAKE_STATES.SPITTING_OUT)
            intake_state = INTAKE_STATES.STOPPED;
        else
            intake_state = INTAKE_STATES.COLLECTING;
    }

    public void toggle(INTAKE_STATES state){
        intake_state = state;
    }

    public void update(){
        intake.setPower(intake_state.val);
    }
}
