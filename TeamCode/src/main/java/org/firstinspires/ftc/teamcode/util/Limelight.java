package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Limelight {
    public Limelight3A limelight;
    public LLResult result;

    public Telemetry telemetry;
    public final int motifPipeline = 3;
    public final int bluePipeline=1, redPipeline=2;
    public int colorPipeline = 1;
    public Pose3D llpose3d, zeroPose;

    public int currentPipeline = bluePipeline;


    public Limelight(HardwareMap hwmap, Telemetry telemetry){
        limelight = hwmap.get(Limelight3A.class, HardwareConfig.limelight);
        this.telemetry = telemetry;

        limelight.pipelineSwitch(0);

        zeroPose = new Pose3D(new Position(DistanceUnit.INCH,0,0,0,0),new YawPitchRollAngles(AngleUnit.RADIANS,0,0,0,0));
        llpose3d = zeroPose;
    }

    public void start(){
        limelight.start();
//        limelight.pipelineSwitch(0);
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
            else if (id==20){
                llpose3d = result.getBotpose();
            }
            else if (id==24){
                llpose3d = result.getBotpose();
            }
            else
                llpose3d = zeroPose;

        }
//        if (currentPipeline == motifPipeline) {
//            for (LLResultTypes.FiducialResult f : result.getFiducialResults() ){
//                int id = f.getFiducialId();
//                if (id==21){
//                    Sorting.motif = Constants.MOTIF.GPP;
//                }
//                else if (id==22){
//                    Sorting.motif = Constants.MOTIF.PGP;
//                }
//                else if (id==23){
//                    Sorting.motif = Constants.MOTIF.PPG;
//                }
//                else return;
//                currentPipeline = colorPipeline;
//                limelight.pipelineSwitch(currentPipeline);
//            }
        }

    public void telemetryData(){
        telemetry.addData("Target X", result.getTx());
        telemetry.addData("Target Y", result.getTy());
        telemetry.addData("Target Area", result.getTa());
    }
    
}
