package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Limelight {
    public Limelight3A limelight;
    public LLResult result;

    public Telemetry telemetry;



    public Limelight(HardwareMap hwmap, Telemetry telemetry){
        limelight = hwmap.get(Limelight3A.class, HardwareConfig.limelight);
        this.telemetry = telemetry;

        limelight.pipelineSwitch(0);

    }

    public void start(){
        limelight.start();
    }

    public LLResult getResult(){
        return limelight.getLatestResult();
    }

    public void update(){
        result = getResult();

        if (result.isValid()) {
            double tx = result.getTx();
            double ty = result.getTy();
            double area = result.getTa();

            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
            telemetry.addData("Target Area", area);
        } else {
            telemetry.addData("Limelight", "No Target Found");
        }
    }
    
}
