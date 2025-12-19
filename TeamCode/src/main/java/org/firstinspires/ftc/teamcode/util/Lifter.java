package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.PIDF;
import org.firstinspires.ftc.teamcode.lib.PIDFCoefficients;

public class Lifter implements Updateable{
    public DcMotorEx front, back;

    public static PIDFCoefficients pidfCoefficients = new PIDFCoefficients(2, 0.27, 0.3);
    PIDF pid;
    static int MAX_RANGE = 100000;

    private boolean up = false;

    Telemetry telemetry;

    public Lifter(HardwareMap hwmap, Telemetry telemetry){
        front = hwmap.get(DcMotorEx.class, HardwareConfig.front_lifter);
        back = hwmap.get(DcMotorEx.class, HardwareConfig.back_lifter);
        pid = new PIDF(pidfCoefficients);

        front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //back.setDirection();
        //front.setPower();
        back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.telemetry = telemetry;
    }

    public void toggle(){
        pid.resetPid();
        if(!up)
            pid.setTargetPosition(1250); //schimba aici dupa caz
        else
            pid.setTargetPosition(0); //schimbati si aici daca e nevoie
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
