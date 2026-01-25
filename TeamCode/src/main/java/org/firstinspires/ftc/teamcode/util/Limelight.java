package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Limelight {
    public Limelight3A limelight;
    public LLResult result;

    public Telemetry telemetry;
    public final int motifPipeline = 0;
    public final int bluePipeline=2, redPipeline=1;
    public int colorPipeline = 1;

    public int currentPipeline = motifPipeline;


    public Limelight(HardwareMap hwmap, Telemetry telemetry){
        limelight = hwmap.get(Limelight3A.class, HardwareConfig.limelight);
        this.telemetry = telemetry;

        limelight.pipelineSwitch(motifPipeline);


    }

    public void start(){
        limelight.start();
    }

    public LLResult getResult(){
        return limelight.getLatestResult();
    }

    public void update() {
        result = getResult();
        if (result.isValid()) {
            telemetryData();
        } else {
            telemetry.addData("Limelight", "No Target Found");
        }
        if (currentPipeline == motifPipeline) {
            for (LLResultTypes.FiducialResult f : result.getFiducialResults() ){
                int id = f.getFiducialId();
                if (id==21){
                    Sorting.motif = Constants.MOTIF.GPP;
                }
                else if (id==22){
                    Sorting.motif = Constants.MOTIF.PGP;
                }
                else if (id==23){
                    Sorting.motif = Constants.MOTIF.PPG;
                }
                else return;
                currentPipeline = colorPipeline;
                limelight.pipelineSwitch(currentPipeline);
            }
        }
    }
    public void telemetryData(){
        telemetry.addData("Target X", result.getTx());
        telemetry.addData("Target Y", result.getTy());
        telemetry.addData("Target Area", result.getTa());
    }
    
}
