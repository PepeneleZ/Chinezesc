package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.PIDF;
import org.firstinspires.ftc.teamcode.lib.PIDFCoefficients;

public class Lifter implements Updateable{
    public DcMotorEx left, right;

    public static PIDFCoefficients pidfCoefficients = new PIDFCoefficients(2, 0.27, 0.3);
    PIDF pid;
    static int MAX_RANGE = 2250;
    public LIFTER_POSITIONS lifter_state = LIFTER_POSITIONS.DOWN;
    public boolean slightlyRetracted = true;
    public double kT=0.5;
    Telemetry telemetry;

    ElapsedTime timer;

    public Lifter(HardwareMap hwmap, Telemetry telemetry){
        left = hwmap.get(DcMotorEx.class, HardwareConfig.left_lifter);
        right = hwmap.get(DcMotorEx.class, HardwareConfig.right_lifter);
        pid = new PIDF(pidfCoefficients);

        left.setDirection(DcMotorSimple.Direction.FORWARD);
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //right.setDirection();
        //left.setDirection();
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.telemetry = telemetry;
    }
    public enum LIFTER_POSITIONS{
        DOWN(0),UP(900);
        final int val;
        LIFTER_POSITIONS(int val) {
            this.val = val;
        }
    }

    public void toggle(){
        pid.resetPid();
        if(lifter_state == LIFTER_POSITIONS.DOWN) {
            lifter_state = LIFTER_POSITIONS.UP;
            slightlyRetracted = false;
        }
        else {
            lifter_state = LIFTER_POSITIONS.DOWN;
            timer.reset();
            timer.startTime();
        }
        pid.setTargetPosition(lifter_state.val);
    }


    @Override
    public void update() {
        if (!slightlyRetracted) {
            if(lifter_state == LIFTER_POSITIONS.UP){
                double power = pid.update(right.getCurrentPosition());
                right.setPower(power);
                left.setPower(power);
            } else if (lifter_state == LIFTER_POSITIONS.DOWN) {
                double deltaPower = Range.clip(kT*timer.seconds(),0,0.5);
                double power = 1 - deltaPower;
                right.setPower(power);
                left.setPower(power);

            }



            TelemetryData();
        }
        else {
            right.setPower(-0.07);
            left.setPower(-0.07);
        }


    }

    public void TelemetryData(){
        telemetry.addData("Lifter pos", right.getCurrentPosition());
    }
}
