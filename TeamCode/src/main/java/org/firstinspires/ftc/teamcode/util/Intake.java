package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.util.Constants_Enums.INTAKE_STATES;

public class Intake {
    public DcMotor intake;
    public INTAKE_STATES intake_state = INTAKE_STATES.STOPPED;
    public boolean fromCollectingToSlightly = false;

    public Intake(HardwareMap hwmap){
        intake = hwmap.get(DcMotorEx.class, HardwareConfig.intake);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void toggle(){
        if (intake_state == INTAKE_STATES.COLLECTING || intake_state == INTAKE_STATES.SPITTING_OUT) {
            intake_state = INTAKE_STATES.STOPPED;
        }
        else {
            intake_state = INTAKE_STATES.COLLECTING;
        }

    }

    public void toggle(INTAKE_STATES state){
        fromCollectingToSlightly = (intake_state == INTAKE_STATES.COLLECTING && state == INTAKE_STATES.SLIGHTLY_MOVING);
        intake_state = state;
    }

    public void update(){
        intake.setPower(intake_state.val);
    }
}
