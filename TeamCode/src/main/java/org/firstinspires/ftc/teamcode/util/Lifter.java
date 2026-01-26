package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.PIDF;
import org.firstinspires.ftc.teamcode.lib.PIDFCoefficients;

public class Lifter implements Updateable{
    public DcMotorEx front, back;

    public static PIDFCoefficients pidfCoefficients = new PIDFCoefficients(0.00002, 0.00027, 0.00003);
    PIDF pid;
    static int MAX_RANGE = 100000;
    public LIFTER_POSITIONS lifter_state = LIFTER_POSITIONS.DOWN;
    Telemetry telemetry;

    public Lifter(HardwareMap hwmap, Telemetry telemetry){
        front = hwmap.get(DcMotorEx.class, HardwareConfig.left_lifter);
        back = hwmap.get(DcMotorEx.class, HardwareConfig.right_lifter);
        pid = new PIDF(pidfCoefficients);

        front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //back.setDirection();
        //front.setDirection();
        back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.telemetry = telemetry;
    }
    public enum LIFTER_POSITIONS{
        DOWN(0),UP(1250);
        final int val;
        LIFTER_POSITIONS(int val) {
            this.val = val;
        }
    }

    public void toggle(){
        pid.resetPid();
        if(lifter_state == LIFTER_POSITIONS.DOWN)
            lifter_state = LIFTER_POSITIONS.UP;
        else
            lifter_state = LIFTER_POSITIONS.DOWN;
        pid.setTargetPosition(lifter_state.val);
    }


    @Override
    public void update() {
        double power = pid.update(back.getCurrentPosition());
        back.setPower(power);
        front.setPower(power);

        TelemetryData();
    }

    public void TelemetryData(){
        telemetry.addData("Lifter pos", back.getCurrentPosition());
    }
}
