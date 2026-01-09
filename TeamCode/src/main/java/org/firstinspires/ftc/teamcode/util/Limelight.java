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
    }
    
}
