package org.firstinspires.ftc.teamcode.lib;

public class PIDF {
    public double kP=0, kI=0, kD=0, kF=0; ;
    public double targetPos = 0, lastError = 0, sumError = 0, error=0;
    public long lastTime;


    public PIDF(double kP, double kI, double kD, double kF){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }
    public PIDF(double kP, double kI, double kD){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }
    
    public PIDF(PIDFCoefficients coeffs){
        this.kP = coeffs.kP;
        this.kI = coeffs.kI;
        this.kD = coeffs.kD;
        this.kF = coeffs.kF;
    }

    public void resetPid(){
        sumError = 0;
        lastTime = System.nanoTime();
        lastError = 0;
        error = 0;


    }

    public void setTargetPosition(double target){
        resetPid();
        targetPos = target;
    }
    public void setTargetWithoutResetting(double target){
        targetPos = target;
    }

    public double getError(){
        return error;
    }

    public double update(double currentTicks){
        long currentTime = System.nanoTime();
        error = targetPos - currentTicks;
        double delta_e = lastError - error;
        double deltaTime = (currentTime-lastTime)*1e-9;

        sumError += error*deltaTime;
        lastError = error;
        lastTime = currentTime;

        return (error * kP +
                delta_e/deltaTime * kD +
                sumError * kI +
                kF);
    }
}